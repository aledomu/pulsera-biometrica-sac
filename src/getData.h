#include "MAX30105.h"

void getSpo2AndHeartRateInit(MAX30105* sensor, int32_t* spo2, int8_t* validSPO2, int32_t* heartRate, int8_t* validHeartRate);
void getSpo2AndHeartRateContinuous(MAX30105* sensor, int32_t* spo2, int8_t* validSPO2, int32_t* heartRate, int8_t* validHeartRate);