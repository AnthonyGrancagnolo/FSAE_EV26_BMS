#include "can_decode.h"
#include <string.h>

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
