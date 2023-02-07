/*!
    \file    main.c
    \brief   USB CDC ACM device

    \version 2020-08-01, V3.0.0, firmware for GD32F30x
    \version 2022-06-10, V3.1.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include "cdc_acm_core.h"
#include "usbd_hw.h"
#include "gd32f30x_it.h"
#include "string.h"

#include "main.h"
#include "systick.h"
//#include "math.h"

#include "ili9341.h"
#include "fonts.h"
#include "ili9341_touch.h"

#include "DataProcessing.h"
#include "Display.h"
#include "Timers.h"

#define BKP_DATA_REG_NUM        42

#define FMC_SERIAL_PAGE_SIZE    ((uint16_t)0x30U)
#define FMC_SERIAL_START_ADDR   ((uint32_t)0x0807E000U)
#define FMC_SERIAL_END_ADDR     FMC_SERIAL_START_ADDR + FMC_SERIAL_PAGE_SIZE

#define FMC_RATE_PAGE_SIZE      8
#define FMC_RATE_START_ADDR     ((uint32_t)0x0807F800U)
#define FMC_RATE_END_ADDR       FMC_RATE_START_ADDR + FMC_RATE_PAGE_SIZE


/* enter the second interruption,set the second interrupt flag to 1 */
__IO uint32_t timedisplay;

#define I2C0_OWN_ADDRESS7      0x72 
#define I2C0_SLAVE_ADDRESS7    0x91 

extern uint8_t en_butt_flag;
extern uint8_t en_butt_count;
extern uint8_t wave_ind_flag;
extern uint8_t wave_detect_flag;
extern int16_t T_Wave;
extern double detect_level;
extern int16_t silence_time_start;
extern int16_t puls_buff[50];
extern uint8_t puls_counter;
extern double current_max;
extern int16_t detect_level_start;
extern uint8_t UART0_buff[200];
extern uint8_t UART0_count;
extern uint8_t finish_6_flag;

double puls_out=0;
uint8_t puls_cur_counter=0;

uint32_t *ptrd;
uint32_t address = 0x00000000U;
uint32_t SERIAL[9]   = {'0','2','0','2','2','2','0','0','1'};
/* calculate the number of page to be programmed/erased */
uint32_t PageNum = (FMC_SERIAL_END_ADDR - FMC_SERIAL_START_ADDR) / FMC_SERIAL_PAGE_SIZE;

extern uint8_t UART0_flag;

int16_t puls_buff_NEW[50]={0};
int16_t puls_buff_NEW_MIN[50]={0};
int16_t puls_buff_AMP[50]={0};
int16_t puls_buff_AMP_MIN[50]={0};

uint16_t frequency=128;

uint16_t Lo_thresh_default = 0x8000;
uint16_t Hi_thresh_default = 0x0000;

uint8_t Hi_ADS1115_config = 0b10001111;
uint8_t Lo_ADS1115_config = 0b11100100;

uint8_t ADS1115_FLAG=0;

extern const int LoLimit;  
extern const int HiLimit; 

int16_t PSys = 0;
int16_t PDia = 0;
uint16_t m_ss;
uint16_t m_mm;
uint16_t m_hh;
int indexPSys = 0;
int indexPDia = 0;
int16_t XMax = 0;
int DerivativeShift = 13;
int DerivativeAverageWidth = 4;
int16_t current_pressure=0;
int16_t ArrayForAver[AVER_SIZE] = {0};
int8_t ArrayForAverIndex = 0;
int16_t i2c_out=0;
int i2c_out_K=0;
uint8_t indicate_charge_toggle=1;
uint8_t indicate_charge_counter=1;
uint16_t cur_day=13, cur_month=12, cur_year=2022;
uint32_t cur_thh=0,cur_tmm=0,cur_tss=0;
uint32_t cur_tim=0;
uint8_t bluetooth_status=0;
uint8_t bonus_byte=0;
uint8_t mode = INIT_START;
uint8_t sim800_FLAG=0;
uint8_t rang_batt_old=99;
uint8_t i2c_transmitter[16];
uint8_t i2c_receiver[16];
uint8_t send_buff[100]={0};
uint8_t buff097[10]={0};
usb_dev usbd_cdc;
uint16_t adc_value[8];
uint16_t num_string=0;
uint16_t count_send_bluetooth=0;
uint8_t size_pack=20;
short int save_clear[10000]={0};
uint32_t main_index=0;
short int PressurePulsationArray[10000]={0};
short int EnvelopeArray[10000]={0};
uint32_t send_counter=0;

int lock_counter = 0;

uint8_t usb_command;

double rate=18.69;
double rate_whole;
double rate_fract;

bool arrhythmia = false;
bool stop_meas = false;

int main(void)
{
    nvic_configuration();      // RTC
    time_init();                        // RTC

    GPIO_config();    
    systick_config();
    
    boot_mode();        

    rcu_config();                                   // USB
    gpio_config();                                  // USB    
    usbd_init(&usbd_cdc, &cdc_desc, &cdc_class);    // USB 
    nvic_config();                                  // USB     
    usbd_connect(&usbd_cdc);                        // USB 

    nvic_config_1();         // timer 1
    timer_config_1();        // timer 1

    nvic_config_2();         // timer 2
    timer_config_2();        // timer 2        
    
    i2c_config();                                                      //I2C ADS1115
    ADS1115_config(0b00000010, 0x00, 0x00);                            //Lo
    ADS1115_config(0b00000011, 0xFF, 0xFF);                            //Hi
    ADS1115_config(0b00000001, Hi_ADS1115_config, Lo_ADS1115_config);  //preconfig
    //ADS1115_read_IT();
    //i2c_init();

    ADC_rcu_config();   // ADC0
    ADC_gpio_config();  // ADC0
    dma_config();       // ADC0
    adc_config();       // ADC0
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL); // ADC0
    
    nvic_irq_enable(USART1_IRQn, 0, 0);             // UART1
    usart_config_1();                               // UART1
    usart_interrupt_enable(USART1, USART_INT_RBNE); // UART1
    
    nvic_irq_enable(USART0_IRQn, 0, 0);             // UART0
    usart_config_0();                               // UART0
    usart_interrupt_enable(USART0, USART_INT_RBNE); // UART0
        
    //while (USBD_CONFIGURED != usbd_cdc.cur_status) {/* wait for standard USB enumeration is finished */}    
        
    ILI9341_Init();
    ILI9341_Touch_init();
    
    if (mode == USB_CHARGING)
    {
        ILI9341_FillScreen(ILI9341_BLACK);
        print_battery();
    }
    else ILI9341_FillScreen(ILI9341_WHITE);
    
    button_interrupt_config();
    
    en_butt_flag=1;
    
    if (mode == INIT_START){        
        print_heart(true);
        print_bluetooth(true);
        print_gsm(true);
        print_heartX3(true);
        print_sys_label();
        print_dia_label();
        print_num_H(888,235,10,YELLOW);
        print_num_H(888,235,120,RED);
        print_num_H(888,235,250,BLACK);        
    }
    else if (mode == PRESSURE_TEST)
    {
        ILI9341_FillRectangle(70, 150, 100, 50, ILI9341_WHITE);
    }    
    
    // waiting for button release, or sitting mode?
    while (gpio_input_bit_get(GPIOC, GPIO_PIN_8)==1){}    
    delay_1ms(300);
    while (gpio_input_bit_get(GPIOC, GPIO_PIN_8)==1){}
    
    en_butt_flag=0;    
                   
    if (sim800_FLAG) {}      //GSM module ...        
    delay_1ms(1000);
    if (mode != USB_CHARGING) {            
            clear_monitor();
    }        
        
    i2c_calibration();    
    
    FmcSerialCheck();
    delay_1ms(200);
    memset(UART0_buff, 0, 200);
    /* PMU lock enable */
    rcu_periph_clock_enable(RCU_PMU); 
    /* BKP clock enable */
    rcu_periph_clock_enable(RCU_BKPI);
    /* enable write access to the registers in backup domain */
    pmu_backup_write_enable();
    /* clear the bit flag of tamper event */
    bkp_flag_clear(BKP_FLAG_TAMPER); 

    rate = ReadRateFromFmc();
        
    timer_1_start();
    
    while (1) 
    {    
        switch (mode)
        {
            case INIT_START:
                if (!button_pressed) 
                {
                    mode = START_SCREEN;
                    button_released = 0;
                    button_pressed_counter = 0;
                }
                break;
            case START_SCREEN:
                bluetooth_check();
                TFT_print();
                if (button_released) 
                {
                    ILI9341_FillRectangle(0, 0, 240, 280, ILI9341_WHITE);                            
                    ILI9341_FillRectangle(100, 270, 140, 50, ILI9341_WHITE);                        
                
                    count_send_bluetooth=0;
                
                    current_pressure=0;
                    i2c_calibration();
                    PUMP_ON;
                    VALVE_FAST_CLOSE;
                    VALVE_SLOW_CLOSE;
                    _lockInterval=50;
                    sector_start_scan=0;
                    reset_detector();
                    puls_counter=0;            
                    detect_FLAG=0;
                    timer_2_start();
                    finish_6_flag=0;
                    stop_meas = false;
                    mode = PUMPING_MANAGEMENT;
                    button_released = 0;
                    button_pressed_counter = 0;
                }
                break;
            case KEY_OFF:
                device_OFF();
                break;
            case PUMPING_MANAGEMENT:
                bluetooth_check();
                shutdown_counter = 0;
                ILI9341_FillRectangle(65, 245, 45, 27, ILI9341_WHITE);
                if (button_released) abort_meas();
//                if (current_pressure>=0 & current_pressure<400) print_num_H(GetAver(current_pressure),235,120,GREEN);
                if (show_pressure_counter == 0)
                {
                    show_pressure_counter = SHOW_PRESSURE_INTERVAL;
                    if (current_pressure>=0 & current_pressure<400) print_num_H(current_pressure,235,120,GREEN);
                }
            
                if (current_pressure >= MAX_ALLOWED_PRESSURE && process_counter > MIN_PUMPING_INTERVAL) 
                {
                    reset_detector();
                    puls_counter=0;      
                    stop_meas = false;
                    PUMP_OFF;
                    VALVE_SLOW_OPEN;
                    mode = MEASUREMENT;        
                }                        
                break;
            case USB_CHARGING:
                shutdown_counter = 0;
                if (gpio_input_bit_get(GPIOB, GPIO_PIN_8)==0) indicate_charge_toggle=1;
                print_batt_charge();                
                delay_1ms(1500);                
                if (gpio_input_bit_get(GPIOC, GPIO_PIN_10)==0) device_OFF();                        
                break;
            case PRESSURE_TEST:
                shutdown_counter = 0;
                convert_NO_save();
                print_num_H(current_pressure,235,120,GREEN);
                usb_send_16(i2c_out,0);
                delay_1ms(200);
                print_time(rtc_counter_get());
                if (usb_command == USB_COMMAND_SET_RATE)
                {   
                    rate = rate_whole + rate_fract / 100;
                }
                break;
            case MEASUREMENT:
                shutdown_counter = 0;
                if (button_released) abort_meas();
                if (show_pressure_counter == 0)
                {
                    show_pressure_counter = SHOW_PRESSURE_INTERVAL;
                    if (current_pressure>=0 & current_pressure<400) print_num_H(current_pressure,235,120,GREEN);
                }
                if (main_index>1+size_pack*(count_send_bluetooth+1))
                {                        
                    uint8_t c_summ=0;                            
                    uint8_t cur_buff_ble[400]={'0', '2', 0x05, count_send_bluetooth & 0xFF, (count_send_bluetooth>>8) & 0xFF, size_pack};
                    
                    for (int f=0;f<size_pack;f++){
                            int16_t cur_press=(((save_clear[count_send_bluetooth*size_pack+f]-i2c_out_K)*100)/rate);
                            cur_buff_ble[6+f*2]=cur_press&0xFF;
                            cur_buff_ble[6+f*2+1]=(cur_press>>8)&0xFF;
                    }                            
                    for (int f=0;f<size_pack*2+6;f++){
                            c_summ+=cur_buff_ble[f];
                    }
                    cur_buff_ble[size_pack*2+6]=c_summ;
                    my_send_string_UART_0(cur_buff_ble,size_pack*2+6+1);
                    count_send_bluetooth++;
                }
                if (current_pressure <= STOP_MEAS_LEVEL || stop_meas)
                {
                    for (int i = 0; i < AVER_SIZE; i++) 
                    {
                        ArrayForAver[i] = 0;
                    }
                
                    ILI9341_FillRectangle(55, 10, 180, 106, ILI9341_WHITE);
                    ILI9341_FillRectangle(55, 120, 180, 106, ILI9341_WHITE);
                    ILI9341_FillRectangle(112, 250, 123, 64, ILI9341_WHITE);    
                
                    memset(EnvelopeArray, 0, 10000);
                    GetArrayOfWaveIndexes(PressurePulsationArray, puls_buff, puls_buff_NEW);
                    f_sorting_MAX();
                    CountEnvelopeArray(puls_buff_NEW,puls_buff_AMP);
                    GetSysDia();
                    CountPulse();    
                    bonus_byte=0;
                    if (main_index>1000 & 
                        PSys > MIN_SYS & 
                        PSys < MAX_SYS & 
                        PDia > MIN_DIA & 
                        PDia < MAX_DIA & 
                        puls_out > MIN_PULSE & 
                        puls_out < MAX_PULSE) 
                    {
                        print_sys_label();
                        print_dia_label();    
                        print_SYS(PSys);
                        print_DIA(PDia);                                    
                        print_num_H((int16_t)puls_out,235,250,BLACK);

                        if (arrhythmia) print_heartX3(true);
                    
                        cur_tim = rtc_counter_get();
                        m_hh = cur_tim / 3600;
                        m_mm = (cur_tim % 3600) / 60;
                        m_ss = (cur_tim % 3600) % 60;
                        check_backup_register(&cur_day, &cur_month, &cur_year);
                        if     (cur_year>=255)    cur_year-=2000;
                    }
                    else 
                    {
                        bonus_byte|=0x80;
                        print_error(4);                            
                    }
                    send_result_measurement((uint8_t)cur_day, (uint8_t)cur_month, (uint8_t)cur_year, (uint8_t)m_ss, (uint8_t)m_mm, (uint8_t)m_hh, (uint8_t)PSys, (uint8_t)PDia, (uint8_t)puls_out,bonus_byte);
                    
                    VALVE_FAST_OPEN;
                    VALVE_SLOW_OPEN;
                    mode = SEND_SAVE_BUFF_MSG;   
//                        mode = INIT_START;
                }                    
                break;
            case SEND_SAVE_BUFF_MSG:
                shutdown_counter = 0;
                break;
        }
        
        if (UART0_flag==1){                    
            for (int w=0;w<200;w++)
            {
                if (finder_msg(UART0_buff))
                {                                                                    
                        ILI9341_FillRectangle(1, 1, 100, 100, ILI9341_RED);                                                            
                }
                if (finder(UART0_buff, "OFF",0,&num_string)) 
                {
                        ILI9341_FillRectangle(1, 1, 100, 100, ILI9341_WHITE);
                }
            }
            UART0_flag=0;
        }
        
        if (wave_ind_flag)
        {                
            print_heart(true);
            delay_1ms(200);
            print_heart(false);
            wave_ind_flag=0;
        }
                    
        if (0U == cdc_acm_check_ready(&usbd_cdc)) 
        {
            cdc_acm_data_receive(&usbd_cdc);
        } 
        else 
        {
            cdc_acm_data_send(&usbd_cdc);
        }            
    }
}

void abort_meas(void) 
{
    mode = START_SCREEN;
    PUMP_OFF;
    VALVE_FAST_OPEN;
    VALVE_SLOW_OPEN;
    button_released = 0;
    button_pressed_counter = 0;
}

void i2c_config(void){
        /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
        /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);
    
      /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
//  gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
        gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    
    /* cofigure I2C clock */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    /* cofigure I2C address */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_OWN_ADDRESS7);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

void SIM_recieve_OK(void){
    usbd_ep_send (&usbd_cdc, CDC_IN_EP, "OK", 3);
}

void ADS1115_config(uint8_t pointer, uint8_t byte1, uint8_t byte2){
        /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
        
        /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));        

        i2c_data_transmit(I2C0,pointer); //0x01);
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}
        i2c_data_transmit(I2C0, byte1); //0x8F);
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}                
        i2c_data_transmit(I2C0, byte2); //0xA3);
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}        
        
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0) & 0x0200);                
}

uint8_t ADS1115_read_IT(void){
        if (gpio_input_bit_get(GPIOB, GPIO_PIN_10)==0)
        {
            while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
            /* send a start condition to I2C bus */
            i2c_start_on_bus(I2C0);
            /* wait until SBSEND bit is set */
            while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));        
            /* send slave address to I2C bus */
            i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);
            /* wait until ADDSEND bit is set */
            while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
            /* clear ADDSEND bit */
            i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
            /* wait until the transmit data buffer is empty */
            while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));        

            i2c_data_transmit(I2C0, 0x00);
            while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}    
            
            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(I2C0);
            /* wait until stop condition generate */
            while(I2C_CTL0(I2C0) & 0x0200);    
        /* wait until I2C bus is idle */    
            while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));        ///////////////////////////////////////////////////////////////////////////
            i2c_start_on_bus(I2C0);                                         
            /* wait until SBSEND bit is set */
            while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));            
            /* send slave address to I2C bus */
            i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_RECEIVER);
            /* wait until ADDSEND bit is set */
            while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
            /* clear ADDSEND bit */
            i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);        
            
            /* wait until the RBNE bit is set */
            while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
            /* read a data from I2C_DATA */
            i2c_receiver[0] = i2c_data_receive(I2C0);
            
            /* wait until the RBNE bit is set */
            while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
            /* read a data from I2C_DATA */
            i2c_receiver[1] = i2c_data_receive(I2C0);
            
            while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));
            /* disable acknowledge */
            i2c_ack_config(I2C0, I2C_ACK_DISABLE);
                            
            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(I2C0);
            /* wait until stop condition generate */
            while(I2C_CTL0(I2C0) & 0x0200);
            /* enable acknowledge */
            i2c_ack_config(I2C0, I2C_ACK_ENABLE);        
            
            ADS1115_config(0b00000001, Hi_ADS1115_config, Lo_ADS1115_config);
            
            return 1;
        }
        return 0;
}

void i2c_convers(void){    
        while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));        
        /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));        

        i2c_data_transmit(I2C0, 0x00);
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}    
        
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0) & 0x0200);    
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
        i2c_start_on_bus(I2C0);                                         ///////////////////////////////////////////////////////////////////////////
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));            
        /* send slave address to I2C bus */
        i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_RECEIVER);
        /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);        
        
        /* wait until the RBNE bit is set */
        while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
    /* read a data from I2C_DATA */
    i2c_receiver[0] = i2c_data_receive(I2C0);
        
        /* wait until the RBNE bit is set */
        while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
    /* read a data from I2C_DATA */
    i2c_receiver[1] = i2c_data_receive(I2C0);
        
        while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));
    /* disable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
                        
        /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0) & 0x0200);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
//        i2c_transmitter[1]=0x01;
            
        /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
        
        /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));        

        i2c_data_transmit(I2C0, 0x01);
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}
        i2c_data_transmit(I2C0, 0x8F);
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}                
        i2c_data_transmit(I2C0,0xC3);//0x83);//0xC3);// 0xA3);
        while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}        
        
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0) & 0x0200);    
}

void GPIO_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);        
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);  //POWER_ON_DETECT
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_10);         //ADS_RDY  --EXTI
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);          // STDBY
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_9);          // CHRG

    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);  //SIM_EXT
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);       //SIM_PWKEY
    gpio_init(GPIOC, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);          //KEY IN
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);       //POWER_ON
    gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10); //USB_ON    
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);      //COMP_ON/OFF
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);      //VALVE1_OP/CL
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);      //VALVE2_OP/CL            
}

void ADC_rcu_config(void)
{
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
}

void ADC_gpio_config(void)
{
    /* config the GPIO as analog mode */
    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
}

void adc_config(void)
{    
        /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);

    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
 
    /* ADC regular channel config */

    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_9, ADC_SAMPLETIME_7POINT5);
    //adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_3, ADC_SAMPLETIME_7POINT5);


    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    
    /* ADC discontinuous mode */
    adc_discontinuous_mode_config(ADC0, ADC_REGULAR_CHANNEL, 3);

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
        //my_delay(50000);

    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
}

void dma_config(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);
    
    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(adc_value);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;  
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = 8;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}

void usart_config_0(void)
{
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);   
        /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);        
        /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
        /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    
    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 9600);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

void my_send_string_UART_0(char *buf, uint8_t num)
{
    for(int j1=0;j1<num;j1++)
    {
        usart_data_transmit(USART0, (uint8_t)buf[j1]);
        while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));            
    }
}

void usart_config_1(void){
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);   
        /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);        
        /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
        /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    
    /* USART configure */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 9600);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);
}

void my_send_string_UART_1(char *buf, uint8_t num)
{
    for(int j1=0;j1<num;j1++)
    {
        usart_data_transmit(USART1, (uint8_t)buf[j1]);
        while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));            
    }
}

void nvic_configuration(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(RTC_IRQn,1,0);
}

void rtc_configuration(void)
{
    /* enable PMU and BKPI clocks */
    rcu_periph_clock_enable(RCU_BKPI);
    rcu_periph_clock_enable(RCU_PMU);
    /* allow access to BKP domain */
    pmu_backup_write_enable();

    /* reset backup domain */
    bkp_deinit();

    /* enable LXTAL */
    rcu_osci_on(RCU_LXTAL);
    /* wait till LXTAL is ready */
    rcu_osci_stab_wait(RCU_LXTAL);
    
    /* select RCU_LXTAL as RTC clock source */
    rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);

    /* enable RTC Clock */
    rcu_periph_clock_enable(RCU_RTC);

    /* wait for RTC registers synchronization */
    rtc_register_sync_wait();

    /* wait until last write operation on RTC registers has finished */
    rtc_lwoff_wait();

    /* enable the RTC second interrupt*/
    rtc_interrupt_enable(RTC_INT_SECOND);

    /* wait until last write operation on RTC registers has finished */
    rtc_lwoff_wait();

    /* set RTC prescaler: set RTC period to 1s */
    rtc_prescaler_set(32767);

    /* wait until last write operation on RTC registers has finished */
    rtc_lwoff_wait();
}

void time_set(uint32_t tmp_hh,uint32_t tmp_mm,uint32_t tmp_ss)
{
    rtc_configuration(); 
    rtc_lwoff_wait();
    rtc_counter_set((tmp_hh*3600 + tmp_mm*60 + tmp_ss));
    rtc_lwoff_wait();
    bkp_write_data(BKP_DATA_0, 0xA5A5);    
}

void time_init(void)
{
    if (bkp_read_data(BKP_DATA_0) != 0xA5A5)
    {
        time_set(0, 0, 0);
    }
    else
    {
        /* check if the power on reset flag is set */
        if (rcu_flag_get(RCU_FLAG_PORRST) != RESET){}
                else if (rcu_flag_get(RCU_FLAG_SWRST) != RESET){}
        /* allow access to BKP domain */
        rcu_periph_clock_enable(RCU_PMU);
        pmu_backup_write_enable();       
        /* wait for RTC registers synchronization */
        rtc_register_sync_wait();
        /* enable the RTC second */
        rtc_interrupt_enable(RTC_INT_SECOND);
        /* wait until last write operation on RTC registers has finished */
        rtc_lwoff_wait();
    }    
    rcu_all_reset_flag_clear();
}

void i2c_init(void)
{
        /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));        
        /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, I2C0_SLAVE_ADDRESS7, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));        

    i2c_data_transmit(I2C0, 0x01);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}
    i2c_data_transmit(I2C0, 0x8F);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}                
    i2c_data_transmit(I2C0,0xC3);//0x83);//0xC3); //0xA3);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE)){}        
        
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0) & 0x0200);            
}

void i2c_print(void)
{
    uint8_t buff1[10]={0};
    i2c_convers();
    i2c_out=(((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K);
    current_pressure=((((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K)/rate);
    sprintf(buff1,"%6d",i2c_out);
    sprintf(buff1,"%6d",current_pressure);
}

void button_interrupt_config(void)
{
        //nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    
    /* enable the key user clock */
    rcu_periph_clock_enable(RCU_GPIOC);   
    
    /* configure button pin as input */
    //gpio_init(GPIOC, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    
    /* enable the AF clock */
    rcu_periph_clock_enable(RCU_AF);    
    
    /* enable and set key user EXTI interrupt to the lowest priority */
        nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(EXTI5_9_IRQn, 2U, 2U);

    /* connect key user EXTI line to key GPIO pin */
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOC, GPIO_PIN_SOURCE_8);

    /* configure key user EXTI line */
    exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_BOTH );
    exti_interrupt_flag_clear(EXTI_8);
}

void device_OFF(void)
{
    PUMP_OFF;
    VALVE_FAST_OPEN;
    VALVE_SLOW_OPEN;
    ILI9341_FillScreen(ILI9341_BLACK);
    gpio_bit_reset(GPIOC, GPIO_PIN_9);
}

void i2c_calibration(void)
{
    i2c_out_K=0;
    for (int g=0;g<20;g++)
    {
        while (ADS1115_read_IT()==0){}
        i2c_out_K+=((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF);                
    }
    i2c_out_K = i2c_out_K/20;
}

void usb_send_i2c_convers(void)
{
    i2c_convers();    
    uint8_t send_buff[3]={25,i2c_receiver[1],i2c_receiver[0]};
    i2c_out=(((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K);
    current_pressure=i2c_out/rate;
    usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 3);
}

uint8_t usb_send_save(int16_t *mass1, int16_t *mass2)
{
    //Add markers of SYS, MAX and DIA points into array
    for (int h=0;h<puls_counter;h++)
    {
            if (send_counter==XMax) PressurePulsationArray[send_counter]=100;                    
    }        
    for (int h=0;h<puls_counter;h++)
    {
            if (send_counter==indexPSys | send_counter==indexPDia)PressurePulsationArray[send_counter]=-100;                    
    }        
    
    uint8_t send_H1=(mass1[send_counter]>>8)&0xFF;
    uint8_t send_L1=mass1[send_counter]&0xFF;
    uint8_t send_H2=(mass2[send_counter]>>8)&0xFF;
    uint8_t send_L2=mass2[send_counter]&0xFF;
    
    uint8_t send_buff[5]={25,send_L1,send_H1,send_L2,send_H2};
    usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 5);
    send_counter++;
    if (send_counter>=main_index) return 1;
    else return 0;
}

short int convert_save_16(void)
{            
    if (ADS1115_read_IT()==0) return 0;
    save_clear[main_index]=(((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K);
    if (save_clear[main_index-1]<0) save_clear[main_index-1]=0;
    main_index++;            
    return 1;
}

short int convert_NO_save(void)
{
    if (ADS1115_read_IT()==0) return 0;
    i2c_out=(((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K);
    current_pressure=i2c_out/rate;    
    return i2c_out;
}

void usb_send_16(short int T1, short int T2)
{
    uint8_t send_H1=(T1>>8)&0xFF;
    uint8_t send_L1=T1&0xFF;
    uint8_t send_H2=(T2>>8)&0xFF;
    uint8_t send_L2=T2&0xFF;
    uint8_t send_buff[5]={25,send_L1,send_H1,send_L2,send_H2};
    usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 5);
}


void boot_mode(void)
{
    if (gpio_input_bit_get(GPIOB, GPIO_PIN_0))
    {
        delay_1ms(10);
        if (gpio_input_bit_get(GPIOC, GPIO_PIN_8))
        {
            gpio_bit_set(GPIOC, GPIO_PIN_9);    
            mode = INIT_START;
        }
        else if (gpio_input_bit_get(GPIOC, GPIO_PIN_10))
        {
            //gpio_bit_set(GPIOC, GPIO_PIN_9);    
            mode = USB_CHARGING;
        }
    }
}

void bluetooth_check(void)
{ 
    if (finder(UART0_buff,"OK",0,0)) 
    {
        if (bluetooth_status == CONNECTED) 
        {
            bluetooth_status = DISCONNECTED;
            print_bluetooth(true);
        }
    }
    else 
    {
        if (bluetooth_status == DISCONNECTED) 
        {
            print_bluetooth(false);
            bluetooth_status = CONNECTED;    
        }
    }
    my_send_string_UART_0("AT\0\n",strlen("AT\0\n"));            
}

uint8_t finder(uint8_t *buff, uint8_t *_string, uint8_t _char, uint16_t *num)
{    
    uint8_t _flag=0;
    for (int j=0;j<200;j++)
    {
        if (buff[j]==_string[0])
        {
            _flag=1;
            for (int k=0;k<strlen(_string);k++)
            {
                if (buff[j+k]!=_string[k]) \
                {
                    _flag=0;
                    k=strlen(_string);
                }                                    
            }                        
        }
        if (_flag) {
                *num=j;
                buff[j]=_char;
                return 1;
        }
    }
    return 0;
}

uint8_t finder_msg(uint8_t *buff)
{    
    uint8_t _flag=0;
    uint8_t _string[20]={'0','2'};

    //ILI9341_WriteString(1, 30, _string, Font_11x18, ILI9341_RED, ILI9341_WHITE);
    for (int j=0;j<200;j++)
    {
        if (buff[j]==_string[0] & buff[j+1]==_string[1])
        {
            _flag=1;                                                
        }
        if (_flag) 
        {
            if (buff[j+2]==0x04)
            {
                uint8_t check_sum=0;
                for (int a=0;a<10;a++)
                {
                    check_sum+=buff[j+a];
                }                                
                if (buff[j+10]==check_sum)
                {
                    SERIAL[2] = buff[j+3];
                    SERIAL[3] = buff[j+4];
                    SERIAL[4] = buff[j+5];
                    SERIAL[5] = buff[j+6];
                    SERIAL[6] = buff[j+7];
                    SERIAL[7] = buff[j+8];
                    SERIAL[8] = buff[j+9];
                    
                    FmcProgramSerial();                        
                
                    buff[j]=0xFF;
                
                    delay_1ms(200);                                    
                    device_OFF();
                
                    return 1;
                }
            }
            else if (buff[j+2]==0x03)
            {
                uint8_t check_sum=0;
                for (int a=0;a<10;a++)
                {
                    check_sum+=buff[j+a];
                }                                
                if (buff[j+10]==check_sum){
                    cur_day=cur_month=cur_month=cur_year=cur_tss=cur_tmm=cur_thh=0;
                    cur_day=(uint16_t)buff[j+3];
                    cur_month=(uint16_t)buff[j+4];
                    cur_year=(uint16_t)(2000+buff[j+5]);
                    cur_tss=(uint32_t)buff[j+6];
                    cur_tmm=(uint32_t)buff[j+7];
                    cur_thh=(uint32_t)buff[j+8];    
                
                    time_set((uint32_t)cur_thh,(uint32_t)cur_tmm,(uint32_t)cur_tss);
                    write_backup_register((uint16_t)cur_day, (uint16_t)cur_month, (uint16_t)cur_year);
                
                    buff[j]=0xFF;
                    return 2;                    }
            }
        }
    }
    return 0;
}

void FmcFlagsClear(void)
{
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
}

void FmcErasePage(uint32_t page_address)
{
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    FmcFlagsClear();
    /* erase the flash page */
    fmc_page_erase(page_address);
    /* lock the main FMC after the erase operation */
    fmc_lock();
}

void FmcProgramSerial(void)
{
    FmcErasePage(FMC_SERIAL_START_ADDR);  

    uint8_t cur_count=0;
    /* unlock the flash program/erase controller */
    fmc_unlock();

    address = FMC_SERIAL_START_ADDR;

    /* program flash */
    while(address < FMC_SERIAL_END_ADDR){
        fmc_word_program(address, SERIAL[cur_count++]);
        address += 4;
        FmcFlagsClear();
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

void FmcProgramRate(uint32_t whole_part, uint32_t fract_part)
{
    FmcErasePage(FMC_RATE_START_ADDR);
    
    /* unlock the flash program/erase controller */
    fmc_unlock();

    address = FMC_RATE_START_ADDR;

    /* program flash */
    fmc_word_program(address, whole_part);
    address += 4;
    FmcFlagsClear();
    fmc_word_program(address, fract_part);
    FmcFlagsClear();

    /* lock the main FMC after the program operation */
    fmc_lock();
}

double ReadRateFromFmc()
{
    uint32_t *ptr;
    ptr = (uint32_t *)FMC_RATE_START_ADDR;
    uint32_t whole = *(ptr);
    uint16_t fract = *(ptr + 1);
    rate_whole = whole;
    rate_fract = fract;
    return (double)(rate_whole + rate_fract / 100);
}

void FmcSerialCheck(void)
{
    uint8_t cur_SERIAL[7]={0};
    uint8_t cur_buff[30]={'A','T','+','N','A','M','E','=','T','O','N','0','2'};

    ptrd = (uint32_t *)FMC_SERIAL_START_ADDR;
        
    if((*ptrd) == '0' & (*(ptrd+1)) == '2')
    {
        SERIAL[2]=cur_SERIAL[0]=*(ptrd+2);    
        SERIAL[3]=cur_SERIAL[1]=*(ptrd+3);
        SERIAL[4]=cur_SERIAL[2]=*(ptrd+4);
        SERIAL[5]=cur_SERIAL[3]=*(ptrd+5);
        SERIAL[6]=cur_SERIAL[4]=*(ptrd+6);
        SERIAL[7]=cur_SERIAL[5]=*(ptrd+7);
        SERIAL[8]=cur_SERIAL[6]=*(ptrd+8);                
    }        
    else
    {
        SERIAL[0]='0';
        SERIAL[1]='2'; 
        SERIAL[2]='0';    cur_SERIAL[0]='0';
        SERIAL[3]='0';    cur_SERIAL[1]='0';
        SERIAL[4]='0';    cur_SERIAL[2]='0';
        SERIAL[5]='0';    cur_SERIAL[3]='0';
        SERIAL[6]='0';    cur_SERIAL[4]='0';
        SERIAL[7]='0';    cur_SERIAL[5]='0';
        SERIAL[8]='0';    cur_SERIAL[6]='0';
        FmcErasePage(FMC_SERIAL_START_ADDR);
        FmcProgramSerial();                
    }        
    strncat(cur_buff,cur_SERIAL,7);
    strncat(cur_buff,"\0\n",2);
    my_send_string_UART_0(cur_buff,strlen(cur_buff));
}

void write_backup_register(uint16_t day, uint16_t month, uint16_t year)
{
    BKP_DATA10_41(10) = day;
    BKP_DATA10_41(11) = month;
    BKP_DATA10_41(12) = year;
}

void check_backup_register(uint16_t *_day, uint16_t *_month, uint16_t *_year)
{
    *_day = BKP_DATA_GET(BKP_DATA10_41(10));
    *_month = BKP_DATA_GET(BKP_DATA10_41(11));
    *_year = BKP_DATA_GET(BKP_DATA10_41(12));
}

void send_result_measurement(uint8_t c_day, uint8_t c_month, uint8_t c_year, uint8_t c_ss, uint8_t c_mm, uint8_t c_hh, int16_t sis, int16_t dia, int16_t pressure, int16_t bonus)
{
    uint8_t cur_buff[13]={'0','2', 0x01, c_day, c_month, c_year, c_ss, c_mm, c_hh, sis, dia, pressure, bonus};        
    uint8_t c_summ=0;        
    for (int q=0;q<13;q++)
    {
       c_summ+=cur_buff[q];
    }            
    cur_buff[13]=c_summ;
    my_send_string_UART_0(cur_buff,14);
}

