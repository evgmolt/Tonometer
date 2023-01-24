#include "main.h"
#include "DataProcessing.h"

int16_t GetAver(int16_t nextValue) {
	ArrayForAver[ArrayForAverIndex] = nextValue;
	ArrayForAverIndex++;
	if (ArrayForAverIndex > AVER_SIZE - 1) ArrayForAverIndex = 0;
	int16_t sum = 0;
	for (int i = 0; i < AVER_SIZE; i++) sum += ArrayForAver[i];
	return sum / AVER_SIZE;
}