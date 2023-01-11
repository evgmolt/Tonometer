
#ifndef __MAIN_H
#define __MAIN_H

#include "usbd_conf.h"

#define GREEN 	0x1
#define RED 		0x2
#define YELLOW 	0x3
#define BLACK 	0x4

#define comp_ON 		gpio_bit_set	(GPIOC, GPIO_PIN_11)
#define comp_OFF 		gpio_bit_reset(GPIOC, GPIO_PIN_11)
#define valve_1_ON 	gpio_bit_set	(GPIOC, GPIO_PIN_12)
#define valve_1_OFF gpio_bit_reset(GPIOC, GPIO_PIN_12)
#define valve_2_ON 	gpio_bit_set	(GPIOC, GPIO_PIN_13)
#define valve_2_OFF gpio_bit_reset(GPIOC, GPIO_PIN_13)

#define INIT_START 0
#define START_SCREEN 1
#define KEY_OFF 2
#define PUMPING_MANAGEMENT 3
#define USB_CHARGING 4
#define PRESSURE_TEST 5
#define MEASUREMENT 6
#define SEND_SAVE_BUFF_MSG 7

/* function declarations */

void i2c_config(void);
void my_delay(int time);	

void ADC_rcu_config(void);
void ADC_gpio_config(void);
void adc_config(void);
void dma_config(void);
void GPIO_config(void);
	
void spi_rcu_config(void);
void spi_gpio_config(void);	

void spi_config(void);

void nvic_configuration(void);
void rtc_configuration(void);
void time_display(uint32_t timevar);


void usb_send_i2c_convers(void);
void i2c_convers(void);	
void TFT_print(void);

void usart_config_0(void);
void my_send_string_UART_0(char *buf, uint8_t num);

void usart_config_1(void);
void my_send_string_UART_1(char *buf, uint8_t num);
void SIM_recieve_OK(void);
void str_clear(char *buff, uint16_t len);
int16_t GetAver(int16_t nextValue);
void print_num_H(uint16_t num, uint16_t X0, uint16_t Y0, uint8_t color);
void time_set(uint32_t tmp_hh,uint32_t tmp_mm,uint32_t tmp_ss);
void time_init(void);
void i2c_init(void);
void i2c_print(void);
void button_interrupt_config(void);

void reset_FLAG(void);
void set_FLAG(void);
void device_OFF(void);
void i2c_calibration(void);

void timer_config_1(void);
void nvic_config_1(void);

void timer_config_2(void);
void nvic_config_2(void);

void timer_2_start(void);
void timer_2_stop(void);
void timer_1_start(void);
void timer_1_stop(void);

short int convert_save_16(void);

uint8_t usb_send_save(int16_t *mass1, int16_t *mass2);
void f_dis(void);

int16_t GetDerivative(int16_t *dataArr, int32_t Ind);

int GetMaxIndexInRegion(int16_t *sourceArray, int index);
void GetArrayOfWaveIndexes(int16_t *valuesArray, int16_t *indexesArray, int16_t *indexes);

uint8_t usb_send_slim_AMP(void);

int GetMaxIndexInRegion(int16_t *sourceArray, int index);
int GetMinIndexInRegion(int16_t *sourceArray_MIN,int index);
	
void f_sorting_MAX(void);
void f_PSys_Dia(void);

uint16_t puls_convert(void);
void clear_monitor(void);
void usb_send_16(short int T1, short int T2);
short int convert_NO_save(void);
int16_t slim_mas(uint16_t *mass_in, int16_t DC, int16_t AC);

void CountEnvelopeArray(int16_t *arrayOfIndexes, int16_t *arrayOfValues);

void ADS1115_config(uint8_t pointer, uint8_t byte1, uint8_t byte2);
uint8_t ADS1115_read_IT(void);
void print_error(uint8_t K);
uint8_t boot_mode(void);
void print_bat_charg(void);
void bluetooth_check(void);
uint8_t finder(uint8_t *buff, uint8_t *_string, uint8_t _char, uint16_t *num);
uint8_t finder_msg(uint8_t *buff);

/* erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR */
void fmc_erase_pages(void);
/* program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR */
void fmc_program(void);
/* check fmc erase result */
void fmc_erase_pages_check(void);
/* check fmc program result */
void fmc_program_check(void);

void write_backup_register(uint16_t day, uint16_t month, uint16_t year);
void check_backup_register(uint16_t *_day, uint16_t *_month, uint16_t *_year);
void send_result_measurement(uint8_t c_day, uint8_t c_month, uint8_t c_year, uint8_t c_ss, uint8_t c_mm, uint8_t c_hh, int16_t sis, int16_t dia, int16_t pressure, int16_t bonus);

void print_SIS(int16_t IN);
void print_DIA(int16_t IN);

#endif /* __MAIN_H */
