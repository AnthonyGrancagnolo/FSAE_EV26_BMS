#include "CANMessages.h"
#include <string.h> // for memset


// ===================== ENCODE FUNCTIONS =====================
// BMS Messages

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

// Inverter Messages

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
/* ===================== DECODE FUNCTIONS ===================== */

void Decode_BMS_STATUS(const uint8_t *data, BMS_STATUS_t *msg)
{
    uint16_t rawVolts = data[0] | (data[1] << 8);
    int16_t  rawAmps  = data[2] | (data[3] << 8);
    uint8_t  rawSOC   = data[4];

    msg->PACKVOLTS     = rawVolts * 0.01f;
    msg->PACKAMPS      = rawAmps * 0.01f;
    msg->STATEOFCHARGE = rawSOC * 0.392f;

    uint8_t status = data[5];
    msg->LOWVOLT      = (status >> 7) & 0x01;
    msg->HIGHVOLT     = (status >> 6) & 0x01;
    msg->LOWTEMP      = (status >> 5) & 0x01;
    msg->HIGHTEMP     = (status >> 4) & 0x01;
    msg->IMBALANCE    = (status >> 3) & 0x01;
    msg->DISCHARGE_OK = (status >> 2) & 0x01;
    msg->CHARGE_OK    = (status >> 1) & 0x01;
    msg->FAULTED      = (status >> 0) & 0x01;
}

void Decode_BMS_QUADVOLTS(const uint8_t *data, BMS_QUADVOLTS_t *msg)
{
    uint16_t r1 = data[0] | (data[1] << 8);
    uint16_t r2 = data[2] | (data[3] << 8);
    uint16_t r3 = data[4] | (data[5] << 8);
    uint16_t r4 = data[6] | (data[7] << 8);

    msg->QUAD1 = r1 * 0.01f;
    msg->QUAD2 = r2 * 0.01f;
    msg->QUAD3 = r3 * 0.01f;
    msg->QUAD4 = r4 * 0.01f;
}

void Decode_BMS_CELLVOLTS(const uint8_t *data, BMS_CELLVOLTS_t *msg)
{
    uint16_t r1 = data[0] | (data[1] << 8);
    uint16_t r2 = data[2] | (data[3] << 8);
    uint16_t r3 = data[4] | (data[5] << 8);
    uint16_t r4 = data[6] | (data[7] << 8);

    msg->AVGVOLT1 = r1 * 0.001f;
    msg->AVGVOLT2 = r2 * 0.001f;
    msg->AVGVOLT3 = r3 * 0.001f;
    msg->AVGVOLT4 = r4 * 0.001f;
}

void Decode_BMS_TEMPS(const uint8_t *data, BMS_TEMPS_t *msg)
{
    uint16_t r1 = data[0] | (data[1] << 8);
    uint16_t r2 = data[2] | (data[3] << 8);
    uint16_t r3 = data[4] | (data[5] << 8);
    uint16_t r4 = data[6] | (data[7] << 8);

    msg->TEMP1 = r1 * 0.1f;
    msg->TEMP2 = r2 * 0.1f;
    msg->TEMP3 = r3 * 0.1f;
    msg->TEMP4 = r4 * 0.1f;
}

void Decode_INVERTER_CTRL(const uint8_t *data, INVERTER_CTRL_t *msg)
{
    msg->ENABLED   = (data[1] >> 6) & 0x03;
    msg->DIRECTION = (data[1] >> 4) & 0x03;

    int16_t rawTorque = data[2] | (data[3] << 8);
    msg->TORQUECMD = rawTorque * 0.1f - 3212.8f;
}

void Decode_SYS_STATUS(const uint8_t *data, SYS_STATUS_t *msg)
{
    int16_t rawTorque = data[0] | (data[1] << 8);
    int16_t rawVolts  = data[2] | (data[3] << 8);
    int16_t rawAmps   = data[4] | (data[5] << 8);
    int16_t rawSpeed  = data[6] | (data[7] << 8);

    msg->ACTUAL_TORQUE = rawTorque * 0.1f - 3212.8f;
    msg->DCVOLTS       = rawVolts  * 0.1f - 3212.8f;
    msg->DCAMPS        = rawAmps   * 0.1f - 3212.8f;
    msg->SHAFT_SPEED   = rawSpeed  * 0.5f - 16064.0f;
}
