#include "main.h"
#include "stdbool.h"

void TFT_print(void);
void PrintNum(int16_t num, uint16_t X0, uint16_t Y0, uint8_t color);
void ClearScreen(void);
void PrintTime(uint32_t timevar);
void PrintError(uint8_t K);
void print_battery(void);
void PrintBattCharge(void);
void PrintSYS(int16_t IN);
void PrintDIA(int16_t IN);

void PrintHeart(bool show);
void PrintHeartX3(bool show);
void PrintBluetooth(bool show);
void PrintGsm(bool show);
void PrintSYS_label(bool show);
void PrintDIA_label(bool show);
