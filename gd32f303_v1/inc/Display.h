#include "main.h"
#include "stdbool.h"

void TFT_print(void);
void print_num_H(int16_t num, uint16_t X0, uint16_t Y0, uint8_t color);
void clear_monitor(void);
void print_time(uint32_t timevar);
void print_error(uint8_t K);
void print_batt_charge(void);
void print_SYS(int16_t IN);
void print_DIA(int16_t IN);

void print_heart(bool show);
void print_heartX3(bool show);
void print_bluetooth(bool show);
void print_gsm(bool show);
void print_sys_label(void);
void print_dia_label(void);
