#include <Arduino.h>
#include <Wire.h>
#include "esp_err.h"

// Arduino-ESP32 uses esp32-camera's SCCB driver. That driver reads
// registers with a repeated START, which some OV7670 modules reject. These
// strong symbols replace it with SCCB's required STOP then START sequence.

namespace {
constexpr uint32_t SCCB_CLOCK_HZ = 50000;

bool selectRegister(uint8_t address, const uint8_t *reg, size_t regLength) {
    Wire.beginTransmission(address);
    Wire.write(reg, regLength);
    return Wire.endTransmission(true) == 0;
}

bool readBytes(
    uint8_t address,
    const uint8_t *reg,
    size_t regLength,
    uint8_t *data,
    size_t dataLength
) {
    if (!selectRegister(address, reg, regLength)) {
        return false;
    }

    if (Wire.requestFrom(address, dataLength, true) != dataLength) {
        return false;
    }

    for (size_t i = 0; i < dataLength; ++i) {
        data[i] = Wire.read();
    }

    return true;
}

int writeBytes(
    uint8_t address,
    const uint8_t *reg,
    size_t regLength,
    const uint8_t *data,
    size_t dataLength
) {
    Wire.beginTransmission(address);
    Wire.write(reg, regLength);
    Wire.write(data, dataLength);
    return Wire.endTransmission(true) == 0 ? 0 : -1;
}
}  // namespace

extern "C" {

int SCCB_Init(int pinSda, int pinScl) {
    const bool started = Wire.begin(pinSda, pinScl, SCCB_CLOCK_HZ);
    Serial.printf(
        "SCCB compatibility bus: SDA=%d SCL=%d %lu Hz (%s)\n",
        pinSda,
        pinScl,
        static_cast<unsigned long>(SCCB_CLOCK_HZ),
        started ? "started" : "failed"
    );
    if (!started) {
        return ESP_FAIL;
    }
    Wire.setTimeOut(1000);
    return ESP_OK;
}

int SCCB_Use_Port(int) {
    // This sketch always supplies SDA/SCL, so esp_camera never calls this.
    return ESP_ERR_NOT_SUPPORTED;
}

int SCCB_Deinit(void) {
    Wire.end();
    return ESP_OK;
}

int SCCB_Probe(uint8_t address) {
    Wire.beginTransmission(address);
    const uint8_t result = Wire.endTransmission(true);
    if (result == 0) {
        Serial.printf("SCCB camera found at 0x%02X\n", address);
    }
    return result == 0 ? ESP_OK : ESP_FAIL;
}

uint8_t SCCB_Read(uint8_t address, uint8_t reg) {
    uint8_t data = 0;
    readBytes(address, &reg, 1, &data, 1);
    return data;
}

int SCCB_Write(uint8_t address, uint8_t reg, uint8_t data) {
    return writeBytes(address, &reg, 1, &data, 1);
}

uint8_t SCCB_Read16(uint8_t address, uint16_t reg) {
    const uint8_t regBytes[] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg)
    };
    uint8_t data = 0;
    readBytes(address, regBytes, 2, &data, 1);
    return data;
}

int SCCB_Write16(uint8_t address, uint16_t reg, uint8_t data) {
    const uint8_t regBytes[] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg)
    };
    return writeBytes(address, regBytes, 2, &data, 1);
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t address, uint16_t reg) {
    const uint8_t regBytes[] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg)
    };
    uint8_t data[] = {0, 0};
    readBytes(address, regBytes, 2, data, 2);
    return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

int SCCB_Write_Addr16_Val16(
    uint8_t address,
    uint16_t reg,
    uint16_t data
) {
    const uint8_t regBytes[] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg)
    };
    const uint8_t dataBytes[] = {
        static_cast<uint8_t>(data >> 8),
        static_cast<uint8_t>(data)
    };
    return writeBytes(address, regBytes, 2, dataBytes, 2);
}

}  // extern "C"
