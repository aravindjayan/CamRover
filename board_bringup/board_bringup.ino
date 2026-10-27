#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp32-hal-rmt.h>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "img_converters.h"

extern "C" int SCCB_Write(uint8_t address, uint8_t reg, uint8_t data);

// Wi-Fi settings inherited from working camera sketch.
const char *WIFI_SSID = "IIT-PALAKKAD";
const char *WIFI_PASSWORD = "";

// OV7670 wiring from camRover PCB.
static constexpr int CAM_D0 = 4;
static constexpr int CAM_D1 = 8;
static constexpr int CAM_D2 = 5;
static constexpr int CAM_D3 = 9;
static constexpr int CAM_D4 = 6;
static constexpr int CAM_D5 = 10;
static constexpr int CAM_D6 = 7;
static constexpr int CAM_D7 = 11;
static constexpr int CAM_XCLK = 13;
static constexpr int CAM_PCLK = 12;
static constexpr int CAM_VSYNC = 21;
static constexpr int CAM_HREF = 14;
static constexpr int CAM_SIOD = 16;
static constexpr int CAM_SIOC = 17;
static constexpr int CAM_RESET = -1;
static constexpr int CAM_PWDN = -1;

// Motor, encoder, and RGB nets from camRover_pcb_v1.
static constexpr int M1_IN1 = 18;
static constexpr int M1_IN2 = 15;
static constexpr int M2_IN1 = 42;
static constexpr int M2_IN2 = 47;
static constexpr int M1_ENC_A = 41;
static constexpr int M1_ENC_B = 40;
static constexpr int M2_ENC_A = 2;
static constexpr int M2_ENC_B = 1;
static constexpr int RGB_DOUT = 48;
static constexpr int RGB_LED_COUNT = 5;

// Camera mounting options. Allowed rotations: 0, 90, 180, 270 degrees clockwise.
// Increase CAMERA_CROP_LEFT if a wider black strip remains at the sensor's left edge.
#define CAMERA_ROTATION 270
#define CAMERA_CROP_LEFT 6
#if CAMERA_ROTATION != 0 && CAMERA_ROTATION != 90 && CAMERA_ROTATION != 180 && CAMERA_ROTATION != 270
#error "CAMERA_ROTATION must be 0, 90, 180, or 270"
#endif

static constexpr uint32_t MOTOR_PWM_HZ = 20000;
static constexpr uint8_t MOTOR_PWM_BITS = 8;
static constexpr uint32_t MOTOR_WATCHDOG_MS = 2000;

WebServer server(80);
static volatile int32_t encoder1 = 0;
static volatile int32_t encoder2 = 0;
static volatile uint32_t encoder1Invalid = 0;
static volatile uint32_t encoder2Invalid = 0;
static volatile uint8_t encoder1State = 0;
static volatile uint8_t encoder2State = 0;
static int32_t encoder1Rate = 0;
static int32_t encoder2Rate = 0;
static int32_t encoder1LastSample = 0;
static int32_t encoder2LastSample = 0;
static uint32_t encoderLastSampleMs = 0;
static int motor1Command = 0;
static int motor2Command = 0;
static uint8_t ledRed = 0;
static uint8_t ledGreen = 0;
static uint8_t ledBlue = 0;
static uint32_t motorsExpireAt = 0;
static uint8_t *cameraRgb = nullptr;
static uint8_t *displayRgb = nullptr;
static bool rgbReady = false;

static void IRAM_ATTR updateEncoder(
    volatile int32_t &count,
    volatile uint32_t &invalid,
    volatile uint8_t &oldState,
    int pinA,
    int pinB
) {
    const uint8_t newState = (digitalRead(pinA) << 1) | digitalRead(pinB);
    const uint8_t transition = (oldState << 2) | newState;
    if (transition == 0x1 || transition == 0x7 || transition == 0xE || transition == 0x8) {
        ++count;
    } else if (transition == 0x2 || transition == 0xB || transition == 0xD || transition == 0x4) {
        --count;
    } else if (newState != oldState) {
        ++invalid;
    }
    oldState = newState;
}

// M1 encoder is physically opposite to M2; swap A/B so rover-forward is positive.
static void IRAM_ATTR encoder1ISR() { updateEncoder(encoder1, encoder1Invalid, encoder1State, M1_ENC_B, M1_ENC_A); }
static void IRAM_ATTR encoder2ISR() { updateEncoder(encoder2, encoder2Invalid, encoder2State, M2_ENC_A, M2_ENC_B); }

static void sampleEncoderRates() {
    const uint32_t now = millis();
    const uint32_t elapsed = now - encoderLastSampleMs;
    if (elapsed < 250) return;
    const int32_t count1 = encoder1;
    const int32_t count2 = encoder2;
    encoder1Rate = (count1 - encoder1LastSample) * 1000L / (int32_t)elapsed;
    encoder2Rate = (count2 - encoder2LastSample) * 1000L / (int32_t)elapsed;
    encoder1LastSample = count1;
    encoder2LastSample = count2;
    encoderLastSampleMs = now;
}

static void resetEncoders() {
    noInterrupts();
    encoder1 = 0;
    encoder2 = 0;
    encoder1Invalid = 0;
    encoder2Invalid = 0;
    encoder1State = (digitalRead(M1_ENC_B) << 1) | digitalRead(M1_ENC_A);
    encoder2State = (digitalRead(M2_ENC_A) << 1) | digitalRead(M2_ENC_B);
    interrupts();
    encoder1Rate = encoder2Rate = 0;
    encoder1LastSample = encoder2LastSample = 0;
    encoderLastSampleMs = millis();
}

static void driveMotor(int in1, int in2, int command) {
    command = constrain(command, -255, 255);
    ledcWrite(in1, command > 0 ? command : 0);
    ledcWrite(in2, command < 0 ? -command : 0);
}

static void setMotors(int left, int right, uint32_t runMs) {
    motor1Command = constrain(left, -255, 255);
    motor2Command = constrain(right, -255, 255);
    driveMotor(M1_IN1, M1_IN2, -motor1Command);
    // M2 is mounted/wired opposite to M1; swap driver inputs for matching direction.
    driveMotor(M2_IN2, M2_IN1, -motor2Command);
    motorsExpireAt = (motor1Command || motor2Command)
        ? millis() + min(runMs, MOTOR_WATCHDOG_MS)
        : 0;
}

static void stopMotors() {
    setMotors(0, 0, 0);
}

static void forceMotorPinsLow() {
    // Load LOW into output latches before enabling outputs to avoid startup edges.
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);
    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);
}

static void setLed(uint8_t red, uint8_t green, uint8_t blue) {
    ledRed = red;
    ledGreen = green;
    ledBlue = blue;
    if (!rgbReady) return;

    rmt_data_t data[RGB_LED_COUNT * 24];
    const uint8_t grb[3] = {green, red, blue};
    size_t symbol = 0;
    for (int led = 0; led < RGB_LED_COUNT; ++led) {
        for (uint8_t value : grb) {
            for (int bit = 7; bit >= 0; --bit) {
                const bool one = value & (1U << bit);
                data[symbol].level0 = 1;
                data[symbol].duration0 = one ? 8 : 4;
                data[symbol].level1 = 0;
                data[symbol].duration1 = one ? 4 : 8;
                ++symbol;
            }
        }
    }
    rmtWrite(RGB_DOUT, data, symbol, RMT_WAIT_FOR_EVER);
}

static bool initialiseCamera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_D0;
    config.pin_d1 = CAM_D1;
    config.pin_d2 = CAM_D2;
    config.pin_d3 = CAM_D3;
    config.pin_d4 = CAM_D4;
    config.pin_d5 = CAM_D5;
    config.pin_d6 = CAM_D6;
    config.pin_d7 = CAM_D7;
    config.pin_xclk = CAM_XCLK;
    config.pin_pclk = CAM_PCLK;
    config.pin_vsync = CAM_VSYNC;
    config.pin_href = CAM_HREF;
    config.pin_sccb_sda = CAM_SIOD;
    config.pin_sccb_scl = CAM_SIOC;
    config.pin_pwdn = CAM_PWDN;
    config.pin_reset = CAM_RESET;
    config.xclk_freq_hz = 16000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 20;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

    const esp_err_t error = esp_camera_init(&config);
    if (error != ESP_OK) {
        Serial.printf("Camera initialization failed: 0x%X\n", error);
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) return false;
    Serial.printf("Camera PID: 0x%04X, color RGB565 320x240\n", sensor->id.PID);

    // Timing proven stable on this OV7670/ESP32-S3 board.
    constexpr uint8_t OV7670_ADDRESS = 0x21;
    SCCB_Write(OV7670_ADDRESS, 0x15, 0x02); // active-low VSYNC
    SCCB_Write(OV7670_ADDRESS, 0x11, 0x03); // sensor clock / 4

    constexpr size_t RGB_BYTES = 320U * 240U * 3U;
    const uint32_t caps = psramFound()
        ? MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        : MALLOC_CAP_8BIT;
    cameraRgb = static_cast<uint8_t *>(heap_caps_malloc(RGB_BYTES, caps));
    displayRgb = static_cast<uint8_t *>(heap_caps_malloc(RGB_BYTES, caps));
    if (!cameraRgb || !displayRgb) {
        Serial.println("Unable to allocate camera transform buffers");
        return false;
    }
    return true;
}

static void cropAndRotate(const uint8_t *source, uint8_t *target, int &outWidth, int &outHeight) {
    constexpr int SOURCE_WIDTH = 320;
    constexpr int SOURCE_HEIGHT = 240;
    constexpr int CROPPED_WIDTH = SOURCE_WIDTH - CAMERA_CROP_LEFT;
    outWidth = (CAMERA_ROTATION == 90 || CAMERA_ROTATION == 270) ? SOURCE_HEIGHT : CROPPED_WIDTH;
    outHeight = (CAMERA_ROTATION == 90 || CAMERA_ROTATION == 270) ? CROPPED_WIDTH : SOURCE_HEIGHT;

    for (int y = 0; y < SOURCE_HEIGHT; ++y) {
        for (int x = 0; x < CROPPED_WIDTH; ++x) {
            int dx = x, dy = y;
            if (CAMERA_ROTATION == 90) {
                dx = SOURCE_HEIGHT - 1 - y; dy = x;
            } else if (CAMERA_ROTATION == 180) {
                dx = CROPPED_WIDTH - 1 - x; dy = SOURCE_HEIGHT - 1 - y;
            } else if (CAMERA_ROTATION == 270) {
                dx = y; dy = CROPPED_WIDTH - 1 - x;
            }
            const size_t src = (static_cast<size_t>(y) * SOURCE_WIDTH + x + CAMERA_CROP_LEFT) * 3U;
            const size_t dst = (static_cast<size_t>(dy) * outWidth + dx) * 3U;
            target[dst] = source[src];
            target[dst + 1] = source[src + 1];
            target[dst + 2] = source[src + 2];
        }
    }
}

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>camRover bring-up</title>
<style>
body{font-family:Arial,sans-serif;background:#171717;color:#eee;text-align:center;margin:0;padding:16px}
.card{max-width:680px;margin:12px auto;padding:14px;background:#292929;border-radius:10px}
img{width:min(96vw,640px);border:2px solid #666;background:#000;image-rendering:auto}
button{font-size:16px;margin:5px;padding:10px 15px;border:0;border-radius:7px;cursor:pointer}
.stop{background:#d33;color:#fff}.led{width:62px;height:38px;padding:2px}.status{font-family:monospace;white-space:pre-wrap}
.test-result{min-height:24px;color:#ffd166;font-family:monospace}
</style></head><body>
<h2>camRover color camera + hardware test</h2>
<img id="camera" alt="OV7670 color frame">
<div class="card"><h3>RGB LED</h3>
<button onclick="led(32,0,0)">Red</button><button onclick="led(0,32,0)">Green</button>
<button onclick="led(0,0,32)">Blue</button><button onclick="led(24,24,24)">White</button>
<button onclick="led(0,0,0)">Off</button></div>
<div class="card"><h3>Motors: 750 ms pulse</h3>
<div>M1 <button onclick="motor(120,0)">Forward</button><button onclick="motor(-120,0)">Reverse</button></div>
<div>M2 <button onclick="motor(0,120)">Forward</button><button onclick="motor(0,-120)">Reverse</button></div>
<button onclick="motor(120,120)">Both forward</button>
<button class="stop" onclick="stop()">STOP</button></div>
<div class="card"><h3>Encoder test</h3>
<p>Lift wheels clear. Each test resets counts, runs one motor for 1 second, then reports result.</p>
<button onclick="encoderTest(1,1)">Test M1 forward</button><button onclick="encoderTest(1,-1)">Test M1 reverse</button><br>
<button onclick="encoderTest(2,1)">Test M2 forward</button><button onclick="encoderTest(2,-1)">Test M2 reverse</button><br>
<button onclick="resetEncoders()">Reset counts</button>
<div id="testResult" class="test-result"></div></div>
<div id="status" class="card status">Connecting...</div>
<script>
const camera=document.getElementById('camera'),statusBox=document.getElementById('status'),testResult=document.getElementById('testResult');let busy=false,testBusy=false;
function frame(){if(busy)return;busy=true;camera.onload=camera.onerror=()=>busy=false;camera.src='/frame.jpg?t='+Date.now()}
async function led(r,g,b){await fetch(`/led?r=${r}&g=${g}&b=${b}`);state()}
async function motor(l,r){await fetch(`/motor?left=${l}&right=${r}&ms=750`);state()}
async function stop(){await fetch('/motor?left=0&right=0&ms=0');state()}
async function resetEncoders(){await fetch('/encoder/reset');testResult.textContent='Counts reset';state()}
async function encoderTest(m,d){if(testBusy)return;testBusy=true;testResult.textContent=`Testing M${m}...`;await fetch('/encoder/reset');let l=m===1?120*d:0,r=m===2?120*d:0;await fetch(`/motor?left=${l}&right=${r}&ms=1000`);setTimeout(async()=>{let s=await(await fetch('/status?t='+Date.now())).json();let c=m===1?s.encoder1:s.encoder2,i=m===1?s.invalid1:s.invalid2;testResult.textContent=`M${m}: ${c} counts, ${i} invalid transitions — ${Math.abs(c)>=4?'PASS':'CHECK WIRING / ENCODER'}`;testBusy=false},1250)}
async function state(){try{let s=await(await fetch('/status?t='+Date.now())).json();statusBox.textContent=`Camera: ${s.width}x${s.height}, rotation ${s.rotation}°\nM1=${s.motor1}  M2=${s.motor2}\nEncoder 1: ${s.encoder1} counts, ${s.rate1}/s, A=${s.a1} B=${s.b1}, invalid=${s.invalid1}\nEncoder 2: ${s.encoder2} counts, ${s.rate2}/s, A=${s.a2} B=${s.b2}, invalid=${s.invalid2}\nLED=${s.led.join(',')}`}catch(e){statusBox.textContent='Connection lost'}}
frame();state();setInterval(frame,1000);setInterval(state,500);
</script></body></html>
)HTML";

static void handleRoot() {
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    server.send_P(200, "text/html", INDEX_HTML);
}

static void handleFrame() {
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) {
        server.send(503, "text/plain", "Camera capture failed");
        return;
    }

    if (!fmt2rgb888(frame->buf, frame->len, frame->format, cameraRgb)) {
        esp_camera_fb_return(frame);
        server.send(500, "text/plain", "RGB565 conversion failed");
        return;
    }
    esp_camera_fb_return(frame);

    int outputWidth = 0;
    int outputHeight = 0;
    cropAndRotate(cameraRgb, displayRgb, outputWidth, outputHeight);
    uint8_t *jpeg = nullptr;
    size_t jpegLength = 0;
    const bool converted = fmt2jpg(
        displayRgb,
        static_cast<size_t>(outputWidth) * outputHeight * 3U,
        outputWidth,
        outputHeight,
        PIXFORMAT_RGB888,
        70,
        &jpeg,
        &jpegLength
    );
    if (!converted || !jpeg) {
        server.send(500, "text/plain", "RGB565 to JPEG conversion failed");
        return;
    }

    WiFiClient client = server.client();
    client.printf(
        "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
        static_cast<unsigned>(jpegLength)
    );
    size_t sent = 0;
    while (sent < jpegLength && client.connected()) {
        const size_t written = client.write(jpeg + sent, jpegLength - sent);
        if (!written) { delay(1); continue; }
        sent += written;
    }
    free(jpeg);
    client.stop();
}

static int requestInt(const char *name, int fallback) {
    return server.hasArg(name) ? server.arg(name).toInt() : fallback;
}

static void handleLed() {
    setLed(
        constrain(requestInt("r", 0), 0, 255),
        constrain(requestInt("g", 0), 0, 255),
        constrain(requestInt("b", 0), 0, 255)
    );
    server.send(200, "text/plain", "OK");
}

static void handleMotor() {
    const int left = constrain(requestInt("left", 0), -255, 255);
    const int right = constrain(requestInt("right", 0), -255, 255);
    const int runMs = constrain(requestInt("ms", 750), 0, (int)MOTOR_WATCHDOG_MS);
    setMotors(left, right, runMs);
    server.send(200, "text/plain", "OK");
}

static void handleEncoderReset() {
    stopMotors();
    resetEncoders();
    server.send(200, "text/plain", "OK");
}

static void handleStatus() {
    constexpr int OUTPUT_WIDTH = (CAMERA_ROTATION == 90 || CAMERA_ROTATION == 270)
        ? 240 : 320 - CAMERA_CROP_LEFT;
    constexpr int OUTPUT_HEIGHT = (CAMERA_ROTATION == 90 || CAMERA_ROTATION == 270)
        ? 320 - CAMERA_CROP_LEFT : 240;
    char json[420];
    snprintf(json, sizeof(json),
        "{\"motor1\":%d,\"motor2\":%d,\"encoder1\":%ld,\"encoder2\":%ld,"
        "\"rate1\":%ld,\"rate2\":%ld,\"invalid1\":%lu,\"invalid2\":%lu,"
        "\"a1\":%d,\"b1\":%d,\"a2\":%d,\"b2\":%d,"
        "\"led\":[%u,%u,%u],\"rotation\":%d,\"width\":%d,\"height\":%d}",
        motor1Command, motor2Command, (long)encoder1, (long)encoder2,
        (long)encoder1Rate, (long)encoder2Rate,
        (unsigned long)encoder1Invalid, (unsigned long)encoder2Invalid,
        digitalRead(M1_ENC_A), digitalRead(M1_ENC_B),
        digitalRead(M2_ENC_A), digitalRead(M2_ENC_B),
        ledRed, ledGreen, ledBlue, CAMERA_ROTATION, OUTPUT_WIDTH, OUTPUT_HEIGHT);
    server.send(200, "application/json", json);
}

static void serviceSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    int a = 0, b = 0, c = 0;
    if (sscanf(line.c_str(), "m %d %d", &a, &b) == 2) {
        setMotors(a, b, 1000);
    } else if (line == "s") {
        stopMotors();
    } else if (sscanf(line.c_str(), "l %d %d %d", &a, &b, &c) == 3) {
        setLed(constrain(a, 0, 255), constrain(b, 0, 255), constrain(c, 0, 255));
    } else if (line == "e") {
        Serial.printf("enc1=%ld enc2=%ld\n", (long)encoder1, (long)encoder2);
    } else {
        Serial.println("Commands: m <left> <right>, s, l <r> <g> <b>, e");
    }
}

static void initialiseBringupIO() {
    // Channels 0/1 and timer 0 stay free for camera XCLK.
    ledcAttachChannel(M1_IN1, MOTOR_PWM_HZ, MOTOR_PWM_BITS, 2);
    ledcAttachChannel(M1_IN2, MOTOR_PWM_HZ, MOTOR_PWM_BITS, 3);
    ledcAttachChannel(M2_IN1, MOTOR_PWM_HZ, MOTOR_PWM_BITS, 4);
    ledcAttachChannel(M2_IN2, MOTOR_PWM_HZ, MOTOR_PWM_BITS, 5);
    stopMotors();

    pinMode(M1_ENC_A, INPUT_PULLUP);
    pinMode(M1_ENC_B, INPUT_PULLUP);
    pinMode(M2_ENC_A, INPUT_PULLUP);
    pinMode(M2_ENC_B, INPUT_PULLUP);
    resetEncoders();
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M1_ENC_B), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), encoder2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_B), encoder2ISR, CHANGE);
    rgbReady = rmtInit(RGB_DOUT, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_3, 10000000);
    setLed(0, 0, 12);
}

void setup() {
    forceMotorPinsLow();
    Serial.begin(115200);
    delay(800);
    Serial.println("\ncamRover color camera + RGB LED + motor test");

    initialiseBringupIO();
    if (!initialiseCamera()) {
        setLed(32, 0, 0);
        while (true) delay(1000);
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 30000) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wi-Fi failed; serial motor/LED commands remain available.");
        setLed(24, 12, 0);
        return;
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/frame.jpg", HTTP_GET, handleFrame);
    server.on("/led", HTTP_GET, handleLed);
    server.on("/motor", HTTP_GET, handleMotor);
    server.on("/encoder/reset", HTTP_GET, handleEncoderReset);
    server.on("/status", HTTP_GET, handleStatus);
    server.begin();
    setLed(0, 24, 0);
    Serial.print("Open http://");
    Serial.println(WiFi.localIP());
}

void loop() {
    server.handleClient();
    serviceSerial();
    sampleEncoderRates();
    if (motorsExpireAt && (int32_t)(millis() - motorsExpireAt) >= 0) stopMotors();
    delay(2);
}
