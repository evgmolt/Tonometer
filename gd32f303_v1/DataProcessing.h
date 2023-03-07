#include "main.h"

int16_t GetDerivative(int16_t *dataArr, int32_t Ind);
void GetArrayOfWaveIndexes(int16_t *valuesArray, int16_t *indexesArray, int16_t *indexes);
int GetMaxIndexInRegion(int16_t *sourceArray, int index);
int GetMinIndexInRegion(int16_t *sourceArray_MIN,int index);
void f_sorting_MAX(void);
void CountEnvelopeArray(int16_t *arrayOfIndexes, int16_t *arrayOfValues);
void GetSysDia(void);
int16_t SmoothAndRemoveDC(uint16_t *mass_in, int16_t DC, int16_t AC);
uint16_t CountPulse(void);


