#ifndef CAN_DECODE_H
#define CAN_DECODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
