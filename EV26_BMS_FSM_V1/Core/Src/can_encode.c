#include "can_encode.h"
#include <string.h> // for memset

void Encode_BMS_STATUS(float packVolts, float packAmps, float soc,
                       uint8_t lowVolt, uint8_t highVolt, uint8_t lowTemp,
                       uint8_t highTemp, uint8_t imbalance, uint8_t dischargeOK,
                       uint8_t chargeOK, uint8_t faulted, uint8_t *data)
{
    memset(data, 0, 8);

    uint16_t rawVolts = (uint16_t)(packVolts / 0.01f);
    int16_t  rawAmps  = (int16_t)(packAmps / 0.01f);
    uint8_t  rawSOC   = (uint8_t)(soc / 0.392f);

    data[0] = rawVolts & 0xFF;
    data[1] = rawVolts >> 8;

    data[2] = rawAmps & 0xFF;
    data[3] = rawAmps >> 8;

    data[4] = rawSOC;

    data[5] = (lowVolt << 7) | (highVolt << 6) | (lowTemp << 5) |
              (highTemp << 4) | (imbalance << 3) | (dischargeOK << 2) |
              (chargeOK << 1) | (faulted << 0);
}

void Encode_BMS_QUADVOLTS(float q1, float q2, float q3, float q4, uint8_t *data)
{
    memset(data, 0, 8);
    uint16_t r1 = q1 / 0.01f;
    uint16_t r2 = q2 / 0.01f;
    uint16_t r3 = q3 / 0.01f;
    uint16_t r4 = q4 / 0.01f;

    data[0] = r1; data[1] = r1 >> 8;
    data[2] = r2; data[3] = r2 >> 8;
    data[4] = r3; data[5] = r3 >> 8;
    data[6] = r4; data[7] = r4 >> 8;
}

void Encode_BMS_CELLVOLTS(float v1, float v2, float v3, float v4, uint8_t *data)
{
    memset(data, 0, 8);
    uint16_t r1 = v1 / 0.001f;
    uint16_t r2 = v2 / 0.001f;
    uint16_t r3 = v3 / 0.001f;
    uint16_t r4 = v4 / 0.001f;

    data[0] = r1; data[1] = r1 >> 8;
    data[2] = r2; data[3] = r2 >> 8;
    data[4] = r3; data[5] = r3 >> 8;
    data[6] = r4; data[7] = r4 >> 8;
}

void Encode_BMS_TEMPS(float t1, float t2, float t3, float t4, uint8_t *data)
{
    memset(data, 0, 8);
    uint16_t r1 = t1 / 0.1f;
    uint16_t r2 = t2 / 0.1f;
    uint16_t r3 = t3 / 0.1f;
    uint16_t r4 = t4 / 0.1f;

    data[0] = r1; data[1] = r1 >> 8;
    data[2] = r2; data[3] = r2 >> 8;
    data[4] = r3; data[5] = r3 >> 8;
    data[6] = r4; data[7] = r4 >> 8;
}

void Encode_INVERTER_CTRL(uint8_t enabled, uint8_t direction, float torqueCmd, uint8_t *data)
{
    memset(data, 0, 8);

    data[1] = ((enabled & 0x3) << 6) | ((direction & 0x3) << 4);

    int16_t rawTorque = (int16_t)((torqueCmd + 3212.8f) / 0.1f);
    data[2] = rawTorque & 0xFF;
    data[3] = rawTorque >> 8;
}

void Encode_SYS_STATUS(float actualTorque, float dcVolts, float dcAmps, float shaftSpeed, uint8_t *data)
{
    memset(data, 0, 8);

    int16_t rawTorque = (int16_t)((actualTorque + 3212.8f) / 0.1f);
    int16_t rawVolts  = (int16_t)((dcVolts + 3212.8f) / 0.1f);
    int16_t rawAmps   = (int16_t)((dcAmps + 3212.8f) / 0.1f);
    int16_t rawSpeed  = (int16_t)((shaftSpeed + 16064.0f) / 0.5f);

    data[0] = rawTorque & 0xFF; data[1] = rawTorque >> 8;
    data[2] = rawVolts & 0xFF;  data[3] = rawVolts >> 8;
    data[4] = rawAmps & 0xFF;   data[5] = rawAmps >> 8;
    data[6] = rawSpeed & 0xFF;  data[7] = rawSpeed >> 8;
}
