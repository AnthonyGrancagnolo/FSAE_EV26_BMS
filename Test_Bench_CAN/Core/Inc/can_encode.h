#ifndef CAN_ENCODE_H
#define CAN_ENCODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Encode_BMS_STATUS(
    float packVolts,
    float packAmps,
    float soc,
    uint8_t lowVolt,
    uint8_t highVolt,
    uint8_t lowTemp,
    uint8_t highTemp,
    uint8_t imbalance,
    uint8_t dischargeOK,
    uint8_t chargeOK,
    uint8_t faulted,
    uint8_t *data);

void Encode_BMS_QUADVOLTS(
    float q1, float q2, float q3, float q4,
    uint8_t *data);

void Encode_BMS_CELLVOLTS(
    float v1, float v2, float v3, float v4,
    uint8_t *data);

void Encode_BMS_TEMPS(
    float t1, float t2, float t3, float t4,
    uint8_t *data);

void Encode_INVERTER_CTRL(
    uint8_t enabled,
    uint8_t direction,
    float torqueCmd,
    uint8_t *data);

void Encode_SYS_STATUS(
    float actualTorque,
    float dcVolts,
    float dcAmps,
    float shaftSpeed,
    uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* CAN_ENCODE_H */
