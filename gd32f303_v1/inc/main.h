
#ifndef __MAIN_H
#define __MAIN_H

#include "usbd_conf.h"
#include "stdbool.h"

#define GREEN 0x1
#define RED 0x2
#define YELLOW 0x3
#define BLACK 0x4

#define YELLOW_LEVEL_SYS 140
#define RED_LEVEL_SYS 160
#define YELLOW_LEVEL_DIA 90
#define RED_LEVEL_DIA 100

#define PUMP_ON             gpio_bit_set(GPIOC, GPIO_PIN_11)
#define PUMP_OFF            gpio_bit_reset(GPIOC, GPIO_PIN_11)
#define VALVE_FAST_CLOSE    gpio_bit_set(GPIOC, GPIO_PIN_12)
#define VALVE_FAST_OPEN     gpio_bit_reset(GPIOC, GPIO_PIN_12)
#define VALVE_SLOW_CLOSE    gpio_bit_set(GPIOC, GPIO_PIN_13)
#define VALVE_SLOW_OPEN     gpio_bit_reset(GPIOC, GPIO_PIN_13)

#define INIT_START 0
#define START_SCREEN 1
#define KEY_OFF 2
#define PUMPING_MANAGEMENT 3
#define USB_CHARGING 4
#define PRESSURE_TEST 5
#define MEASUREMENT 6
#define SEND_SAVE_BUFF_MSG 7

#define STOP_MEAS_LEVEL 60
#define MIN_PRESSURE 120    
#define SEC_AFTER_MAX 8
#define DELAY_AFTER_PUMPING 50
#define DELAY_AFTER_START 400
#define DELAY_FOR_ERROR 1000
#define PRESSURE_FOR_ERROR 12
#define MAX_ALLOWED_PRESSURE 176
#define AVER_SIZE 10

#define DEBONCE_INTERVAL 4
#define GO_TO_TEST_INTERVAL 500
#define SWITCH_OFF_INTERVAL 120
#define SHUTDOWN_INTERVAL 10000 //100 seconds
#define SEVEN_SECONDS 700
#define MIN_PUMPING_INTERVAL 1000
#define SHOW_PRESSURE_INTERVAL 40

#define CONNECTED 1
#define DISCONNECTED 0

#define MAX_SYS 250
#define MIN_SYS 60
#define MAX_DIA 150
#define MIN_DIA 30
#define MIN_PULSE 25
#define MAX_PULSE 250

#define LOCK_INTERVAL 20

#define USB_COMMAND_SET_RATE 11

#define ERROR_CUFF 2
#define ERROR_TIME 3
#define ERROR_MEAS 4

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
void print_time(uint32_t timevar);

void usb_send_i2c_convers(void);
void i2c_convers(void);    
void TFT_print(void);

void usart_config_0(void);
void my_send_string_UART_0(char *buf, uint8_t num);

void usart_config_1(void);
void my_send_string_UART_1(char *buf, uint8_t num);
void SIM_recieve_OK(void);
int16_t GetAver(int16_t nextValue);
void print_num_H(int16_t num, uint16_t X0, uint16_t Y0, uint8_t color);
void time_set(uint32_t tmp_hh,uint32_t tmp_mm,uint32_t tmp_ss);
void time_init(void);
void i2c_init(void);
void i2c_print(void);
void button_interrupt_config(void);

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

int GetMaxIndexInRegion(int16_t *sourceArray, int index);
int GetMinIndexInRegion(int16_t *sourceArray_MIN,int index);
    
void f_sorting_MAX(void);
void GetSysDia(void);

uint16_t CountPulse(void);
void clear_monitor(void);
void usb_send_16(short int T1, short int T2);
short int convert_NO_save(void);
int16_t SmoothAndRemoveDC(uint16_t *mass_in, int16_t DC, int16_t AC);

void CountEnvelopeArray(int16_t *arrayOfIndexes, int16_t *arrayOfValues);

void ADS1115_config(uint8_t pointer, uint8_t byte1, uint8_t byte2);
uint8_t ADS1115_read_IT(void);
void print_error(uint8_t K);
void boot_mode(void);
void print_batt_charge(void);
void bluetooth_check(void);
uint8_t finder(uint8_t *buff, uint8_t *_string, uint8_t _char, uint16_t *num);
uint8_t finder_msg(uint8_t *buff);

/* erase fmc page from FMC_WRITE_START_ADDR */
void FmcErasePage(uint32_t page_address);
/* program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR */
void FmcProgramSerial(void);
/* check fmc erase result */
void FmcErasePage_check(void);
/* check fmc program result */
void FmcSerialCheck(void);

void write_backup_register(uint16_t day, uint16_t month, uint16_t year);
void check_backup_register(uint16_t *_day, uint16_t *_month, uint16_t *_year);
void send_result_measurement(uint8_t c_day, uint8_t c_month, uint8_t c_year, uint8_t c_ss, uint8_t c_mm, uint8_t c_hh, int16_t sis, int16_t dia, int16_t pressure, int16_t bonus);

void print_SYS(int16_t IN);
void print_DIA(int16_t IN);

void abort_meas(void);

double ReadRateFromFmc();

extern bool arrhythmia;
extern bool stop_meas;

extern uint16_t detect_FLAG;
extern uint16_t finish_time;

extern const int lo_limit; //ms - 200 
extern const int hi_limit; //ms - 30

extern uint8_t en_butt_flag;
extern uint8_t en_butt_count;

extern uint8_t UART1_buff[200];
extern uint8_t UART1_count;

extern uint8_t UART0_buff[200];
extern uint8_t UART0_count;

extern uint8_t FLAG_CONT_CH;

extern int slim_sr;
extern int slim_sr_final;
extern int slim_sr_OLD;
extern uint8_t slim_count;
extern float slim_K;

extern int16_t detect_level_start;
extern double detect_level;
extern int16_t _lockInterval;
extern double detect_levelCoeff;
extern double stop_meas_coeff;
extern int16_t current_value;
extern double current_max;
extern double global_max;
extern uint8_t wave_detect_flag;
extern int16_t Wave_detect_time;
extern int16_t Wave_detect_time_OLD;
extern int16_t T_Wave;
extern uint8_t wave_ind_flag;
extern int16_t silence_time_start;
extern int16_t MAX_dir_wave;
extern int16_t puls_buff[50];
extern uint8_t puls_counter;
extern int16_t sector_scan;
extern int16_t sector_start_scan;
extern uint8_t finish_6_flag;

extern double puls_out;
extern uint8_t puls_cur_counter;

extern uint32_t *ptrd;
extern uint32_t address;
extern uint32_t SERIAL[9];
/* calculate the number of page to be programmed/erased */
extern uint32_t PageNum;

extern int16_t puls_buff_NEW[50];
extern int16_t puls_buff_NEW_MIN[50];
extern int16_t puls_buff_AMP[50];
extern int16_t puls_buff_AMP_MIN[50];

extern uint16_t frequency;

extern uint16_t Lo_thresh_default;
extern uint16_t Hi_thresh_default;

extern uint8_t Hi_ADS1115_config;
extern uint8_t Lo_ADS1115_config;

extern uint8_t ADS1115_FLAG;

extern int16_t PSys;
extern int16_t PDia;
extern uint16_t m_ss;
extern uint16_t m_mm;
extern uint16_t m_hh;

extern int indexPSys;
extern int indexPDia;
extern int16_t XMax;

extern int DerivativeShift;
extern int DerivativeAverageWidth;

extern int16_t current_pressure;
extern int16_t ArrayForAver[AVER_SIZE];
extern int8_t ArrayForAverIndex;

extern int16_t i2c_out;
extern int i2c_out_K;
extern uint8_t indicate_charge_toggle;
extern uint8_t indicate_charge_counter;

extern uint16_t cur_day, cur_month, cur_year;
extern uint32_t cur_thh,cur_tmm,cur_tss;
extern uint32_t cur_time;

extern uint8_t bluetooth_status;
extern uint8_t bonus_byte;


extern uint32_t MAX_counter;
extern uint16_t Time_measurement; 

extern int16_t dc_array_window;
extern int16_t ac_array_window;
extern uint8_t UART0_flag;

extern uint8_t Hi_ADS1115_config;
extern uint8_t Lo_ADS1115_config;

extern uint8_t mode;

extern uint8_t sim800_FLAG;
extern uint8_t rang_batt_old;
extern uint8_t i2c_transmitter[16];
extern uint8_t i2c_receiver[16];

extern uint8_t send_buff[100];
extern uint8_t buff097[10];
extern uint16_t adc_value[8];


extern uint16_t num_string;

extern uint16_t count_send_bluetooth;
extern uint8_t size_pack;

extern short int save_clear[10000];
extern uint32_t main_index;
extern short int pressure_array[10000];
extern short int dc_array[10000];

extern uint32_t send_counter;

extern int lock_counter;

extern double rate;
extern double rate_whole;
extern double rate_fract;

extern int shutdown_counter;
extern int process_counter;
extern int show_pressure_counter;

extern int button_touched;
extern int button_pressed;
extern int button_released;
extern int button_touched_counter;
extern int button_pressed_counter;

extern uint8_t usb_command;


#endif /* __MAIN_H */
