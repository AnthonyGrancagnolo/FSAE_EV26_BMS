#ifndef CANMessages_H
#define CANMessages_H

#include <stdint.h>
#include "main.h"
#include <string.h> // for memset
#include "can.h" // for CAN_HandleTypeDef


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

// ===================== DECODE FUNCTIONS =====================
typedef struct {
    float PACKVOLTS;
    float PACKAMPS;
    float STATEOFCHARGE;
    uint8_t LOWVOLT;
    uint8_t HIGHVOLT;
    uint8_t LOWTEMP;
    uint8_t HIGHTEMP;
    uint8_t IMBALANCE;
    uint8_t DISCHARGE_OK;
    uint8_t CHARGE_OK;
    uint8_t FAULTED;
} BMS_STATUS_t;

typedef struct {
    float QUAD1;
    float QUAD2;
    float QUAD3;
    float QUAD4;
} BMS_QUADVOLTS_t;

typedef struct {
    float AVGVOLT1;
    float AVGVOLT2;
    float AVGVOLT3;
    float AVGVOLT4;
} BMS_CELLVOLTS_t;

typedef struct {
    float TEMP1;
    float TEMP2;
    float TEMP3;
    float TEMP4;
} BMS_TEMPS_t;

typedef struct {
    uint8_t ENABLED;
    uint8_t DIRECTION;
    float TORQUECMD;
} INVERTER_CTRL_t;

typedef struct {
    float ACTUAL_TORQUE;
    float DCVOLTS;
    float DCAMPS;
    float SHAFT_SPEED;
} SYS_STATUS_t;

/* Decode functions */
void Decode_BMS_STATUS(const uint8_t *data, BMS_STATUS_t *msg);
void Decode_BMS_QUADVOLTS(const uint8_t *data, BMS_QUADVOLTS_t *msg);
void Decode_BMS_CELLVOLTS(const uint8_t *data, BMS_CELLVOLTS_t *msg);
void Decode_BMS_TEMPS(const uint8_t *data, BMS_TEMPS_t *msg);
void Decode_INVERTER_CTRL(const uint8_t *data, INVERTER_CTRL_t *msg);
void Decode_SYS_STATUS(const uint8_t *data, SYS_STATUS_t *msg);

#ifdef __cplusplus
}
#endif

#endif
