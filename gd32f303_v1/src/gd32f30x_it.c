/*!
    \file    gd32f30x_it.c
    \brief   main interrupt service routines

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

#include "gd32f30x_it.h"
#include "usbd_lld_int.h"
#include "systick.h"
#include "main.h"
#include "cdc_acm_core.h"
#include "ili9341.h"

extern __IO uint32_t timedisplay;
extern uint8_t start_send_ble_flag;
extern uint8_t start_finish_ble_flag;

const int lo_limit = 30;  // 256 bpm
const int hi_limit = 250; // 30 bpm

uint8_t UART1_buff[200]={0};
uint8_t UART1_count=0;

uint8_t UART0_buff[200]={0};
uint8_t UART0_count=0;

int16_t detect_level_start = 4;
double detect_level = 4;
int16_t lock_interval = 50;
double detect_levelCoeff = 0.7;
double detect_levelCoeffDia = 0.55;
double stop_meas_coeff = 0.58;
int16_t current_interval = 0;
double current_max=0;
double global_max=0;
uint8_t wave_detect_flag=0;
int16_t Wave_detect_time=0;
int16_t Wave_detect_time_OLD=0;
uint8_t wave_ind_flag=0;
bool show_heart = false;
bool erase_heart = false;
int16_t silence_time_start=0;
int16_t puls_buff[50]={0};
uint8_t puls_counter=0;

int16_t dc_array_window = 30;
int16_t ac_array_window = 4;

uint8_t UART0_flag=0;

int shutdown_counter = 0;
int process_counter = 0;
int heart_counter = 0;
int show_pressure_counter = 0;

int button_touched = 0;
int button_pressed = 0;
int button_released = 0;
int button_touched_counter = 0;
int button_pressed_counter = 0;

extern usb_dev usbd_cdc;
/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles USBD interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBD_LP_CAN0_RX0_IRQHandler (void)
{
    usbd_isr();
}


#ifdef USBD_LOWPWR_MODE_ENABLE

/*!
    \brief      this function handles USBD wakeup interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBD_WKUP_IRQHandler (void)
{
    exti_interrupt_flag_clear(EXTI_18);
}

#endif /* USBD_LOWPWR_MODE_ENABLE */

void TIMER1_IRQHandler(void)
{    
    const int pwrkey_down_time = 100;
    const int ext_up_time = 120;
    const int pwrkey_up_time = 250;
    if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP))
    {        
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);        
        timer_interrupt_enable(TIMER1, TIMER_INT_UP);
        timer_enable(TIMER1);            

        if (lock_counter > 0) lock_counter--;
        if (show_pressure_counter > 0) show_pressure_counter--;

        process_counter++;
/*        if (mode == INIT_START || mode == START_SCREEN || mode == PUMPING_MANAGEMENT) 
        {
            if (process_counter == pwrkey_down_time) 
            SIM800_PWRKEY_DOWN;
        if (process_counter == ext_up_time)      
            SIM800_EXT_UP;
        if (process_counter == pwrkey_up_time)   
            SIM800_PWRKEY_UP;
            if (process_counter == 300) 
                send_buf_UART_1("AT",strlen("AT"));            
        }*/
        
        if (heart_counter > 0)
        {
            heart_counter--;
            erase_heart = (heart_counter == 0);
        }
            
        shutdown_counter++;
        if (shutdown_counter > SHUTDOWN_INTERVAL) 
        {
            shutdown_counter = 0;
//            mode = KEY_OFF;                    
        }
    
        button_touched = gpio_input_bit_get(GPIOC, GPIO_PIN_8);
        if (button_touched) 
        {
            button_touched_counter++;
            if (button_touched_counter > DEBONCE_INTERVAL) button_pressed = 1;
        }
        else 
        {
            button_touched = 0;
            button_pressed = 0;
            button_released = 0;
            button_touched_counter = 0;
        }
        
        if (button_pressed) 
        {
            button_pressed_counter++;
            shutdown_counter = 0;
        }
        else 
        {
            if (button_pressed_counter > 0) button_released = 1;
        }

        if (mode == INIT_START) 
        {
            if (button_pressed_counter > GO_TO_TEST_INTERVAL) 
            {
                ILI9341_FillScreen(ILI9341_WHITE);
                Timer2Stop();
                i2cCalibration();
                VALVE_FAST_CLOSE;
                VALVE_SLOW_CLOSE;
                button_pressed_counter = 0;
                mode = PRESSURE_TEST;
            }
        }
        else 
        {
            if (button_pressed_counter > SWITCH_OFF_INTERVAL && mode != USB_CHARGING) 
            {
                mode = KEY_OFF;
            }
        }
    }
}

void Lock(void)
{
    lock_counter = LOCK_INTERVAL;
}


void TIMER2_IRQHandler(void)
{
    static int32_t max_index;
    
    int16_t current_value;

    if(SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP))
    {
        /* clear update interrupt bit */
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);        
        timer_interrupt_enable(TIMER2, TIMER_INT_UP);
        timer_enable(TIMER2);            

        if (mode == PUMPING_MANAGEMENT)
        {                                                                                                                                                                //////////////////////////////////
            if (convert_save_16())
            {
                if (main_index<300) 
                {
                    convert_NO_save();
                }
                else if (main_index>300)
                {
                    pressure_pulsation_array[main_index-1] = SmoothAndRemoveDC(pressure_array, dc_array_window, ac_array_window);											
                }	
            
                if (main_index >= DELAY_AFTER_START)
                {                                        
                    current_value=GetDerivative(pressure_pulsation_array, main_index-1);
                    usb_send_16(current_value,(short)current_max);
                    if (current_value>current_max)
                    {
                        current_max=current_value;
                        max_index=main_index;
                    }
                    if (current_pressure > MIN_PRESSURE)
                    {                                                
                        if (main_index > max_index + SEC_AFTER_MAX * frequency)
                        {    
                            main_index=0;        
                            ble_buffer_counter = 0;
                            wave_detect_flag=0;                                                    
                            current_max = 0;    
                            global_max = 0;
                            Lock();
                            PUMP_OFF;
                            VALVE_SLOW_OPEN;
                            stop_meas = false;
                            mode = MEASUREMENT;
                        }
                    }                                        
                }
                
                if (main_index > DELAY_FOR_ERROR & current_pressure < PRESSURE_FOR_ERROR)
                { 
                    ResetDetector();
                    Timer2Stop();
                    PrintError(ERROR_CUFF);
                    PUMP_OFF;
                    VALVE_FAST_OPEN;
                    VALVE_SLOW_OPEN;
                    mode = START_SCREEN;
                }
                
                if (main_index>9990)
                {
                    ResetDetector();
                    Timer2Stop();
                    PrintError(ERROR_TIME);
                    PUMP_OFF;
                    VALVE_FAST_OPEN;
                    VALVE_SLOW_OPEN;
                    mode = START_SCREEN;
                }
            }
        }
        else 
        if (mode == MEASUREMENT) 
        {
            if (convert_save_16()) 
            {
                //ѕодготовка данных дл€ передачи по bluetooth
                ble_buffer_counter++;
                if (ble_buffer_counter > BLE_PACKET_SIZE - 1)
                {
                    ble_buffer_counter = 0;
                    for (int i = 0; i < BLE_PACKET_SIZE; i++)
                    {
                        ble_buffer[i] = (pressure_array[main_index - 1 - BLE_PACKET_SIZE + i] * 100) / rate;                        
                    }
                    ble_data_ready = true;
                }
                
                if (main_index>100)
                {
                    if (lock_counter > 0)
                    {
                        pressure_pulsation_array[main_index-1] = 0; 
                    }
                    else
                    {
                        pressure_pulsation_array[main_index-1] = SmoothAndRemoveDC(pressure_array, dc_array_window, ac_array_window);  
                    }
                }    
                else         
                {
                    pressure_pulsation_array[main_index-1]=0;
                }
                                    
                if (main_index >= DELAY_AFTER_PUMPING)
                {                
                    current_interval++;
                    if (current_interval > NO_WAVE_INTERVAL)
                    {
                        detect_level = detect_level_start;
                    }
                    current_value = GetDerivative(pressure_pulsation_array, main_index-1);
                    usb_send_16(current_value, current_max); 
                    if (current_value > detect_level & (main_index - 1) > (silence_time_start + lock_interval)) wave_detect_flag = 1;
                    if (wave_detect_flag == 1 & (main_index-1)>(silence_time_start + lock_interval))
                    {
                        if (current_value > current_max) 
                        {
                            current_max = current_value;
                            max_index = main_index - 1;
                        }
                        else if (current_value < detect_level)
                        {
                            global_max = fmax(global_max, current_max);
                            if (process_counter > SEVEN_SECONDS && current_max < global_max * stop_meas_coeff)
                            {
                                stop_meas = true;
                            }
                            first_max = max_index;
                            Wave_detect_time_OLD = Wave_detect_time;
                            Wave_detect_time = max_index - 1;                                                                                                                        
                            puls_buff[puls_counter++]= max_index - 1;
                            heart_counter = HEART_INTERVAL;
                            wave_ind_flag = 1; 
                            show_heart = true;    
                            lock_interval = (Wave_detect_time - Wave_detect_time_OLD) / 2;
//                            if (lock_interval > hi_limit | lock_interval < lo_limit) lock_interval = 50;
                            silence_time_start = max_index - 1;
                            detect_level = current_max * detect_levelCoeff;
                            if (detect_level < detect_level_start) detect_level = detect_level_start;
                            current_max = 0;
                            current_interval = 0;
                            wave_detect_flag = 0;
                        }
                    }                        
                }                        
            }
        }
        else if (mode == SEND_SAVE_BUFF_MSG) 
        {
            if (button_released) 
            {
                mode = START_SCREEN;
                button_released = 0;
                button_pressed_counter = 0;
                VALVE_FAST_OPEN;
                VALVE_SLOW_OPEN;
                PUMP_OFF;
            }
            else
            if (usb_send_save(pressure_pulsation_array,envelope_array))
            {            
                    mode = INIT_START;
                    Timer2Stop();
                    main_index=0;
                    ble_buffer_counter = 0;
                    send_counter=0;
            }
        } 
        else if (mode == PRESSURE_TEST)
        {
            VALVE_FAST_CLOSE;
            VALVE_SLOW_CLOSE;
        }
    }
}

uint8_t usb_send_save(int16_t *mass1, int16_t *mass2)
{
    //Add markers of SYS, MAX and DIA points into array
    for (int h=0;h<puls_counter;h++)
    {
            if (send_counter==XMax) pressure_pulsation_array[send_counter]=100;                    
    }        
    for (int h=0;h<puls_counter;h++)
    {
            if (send_counter==indexPSys | send_counter==indexPDia) pressure_pulsation_array[send_counter]=-100;                    
    }        
    
    uint8_t send_H1=(mass1[send_counter]>>8)&0xFF;
    uint8_t send_L1=mass1[send_counter]&0xFF;
    uint8_t send_H2=(mass2[send_counter]>>8)&0xFF;
    uint8_t send_L2=mass2[send_counter]&0xFF;
    
    uint8_t send_buff[5]={25,send_L1,send_H1,send_L2,send_H2};
    usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 5);
    send_counter++;
    if (send_counter >= total_size) return 1;
    else return 0;
}

//‘ункци€ используетс€ при втором проходе по массиву пульсаций давлени€, когда уже известно положение максимума.
//ƒо максимума используетс€ detect_levelCoeff, а после detect_levelCoeffDia, так как скорость спада амплитуд пульсаций
//после максимума выше
void Detect(int32_t x_max, uint16_t index)
{                
    static int32_t index_of_max;
    int16_t current_value;
    double coeff;
        
    current_interval++;
    if (current_interval > NO_WAVE_INTERVAL)
    {
        detect_level = detect_level_start;
    }
    current_value = GetDerivative(pressure_pulsation_array, index - 1);
    if (current_value > detect_level & (index - 1) > (silence_time_start + lock_interval)) wave_detect_flag = 1;
    if (wave_detect_flag == 1 & (index - 1) > (silence_time_start + lock_interval))
    {
        if (current_value > current_max) 
        {
            current_max = current_value;
            index_of_max = index-1;
        }
        else if (current_value < detect_level)
        {
            global_max = fmax(global_max, current_max);
            Wave_detect_time_OLD = Wave_detect_time;
            Wave_detect_time = index_of_max - 1;                                                                                                                        
            puls_buff[puls_counter++] = index_of_max-1;
            lock_interval=(Wave_detect_time - Wave_detect_time_OLD) / 2;
            silence_time_start = index_of_max - 1;
            if (index < x_max) coeff = detect_levelCoeff; else coeff = detect_levelCoeffDia;
            detect_level = current_max * coeff;
            if (detect_level < detect_level_start) detect_level = detect_level_start;
            current_max=0;
            current_interval = 0;
            wave_detect_flag=0;
        }
    }                        
}                        

void ResetDetector(void)
{
    process_counter = 0;
    main_index=0;        
    wave_detect_flag=0;    
    current_max=0;        
    global_max=0;        
    detect_level=detect_level_start;
    silence_time_start=0;
    ble_buffer_counter = 0;
}

void my_i2c_send(uint8_t data){
    uint8_t bit;
//    gpio_init(GPIOB, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    for (int i=0;i<8;i++) {
        gpio_bit_reset(GPIOB, GPIO_PIN_6);
        gpio_bit_reset(GPIOB, GPIO_PIN_7);
        my_delay(5);
        bit=(data>>(7-i))&1;
        if (bit==1) gpio_bit_set(GPIOB, GPIO_PIN_7);
        else gpio_bit_reset(GPIOB, GPIO_PIN_7);        
        gpio_bit_set(GPIOB, GPIO_PIN_6);
        my_delay(10);
    }    
    gpio_bit_reset(GPIOB, GPIO_PIN_7);
    gpio_bit_reset(GPIOB, GPIO_PIN_6);
//    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    my_delay(10);
    gpio_bit_set(GPIOB, GPIO_PIN_6);
    my_delay(10);
    gpio_bit_reset(GPIOB, GPIO_PIN_6);
    my_delay(1000);    
    gpio_bit_set(GPIOB, GPIO_PIN_6);        
}

uint8_t my_i2c_read(void){
    uint8_t out=0;
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    for (int i=0;i<8;i++) {
        out=out<<1;
        gpio_bit_reset(GPIOB, GPIO_PIN_6);
        my_delay(10);
        gpio_bit_set(GPIOB, GPIO_PIN_6);        
        if (gpio_input_bit_get(GPIOB, GPIO_PIN_7)) out++;        
        my_delay(5);
    }
    gpio_bit_reset(GPIOB, GPIO_PIN_7);
    gpio_bit_reset(GPIOB, GPIO_PIN_6);
    gpio_bit_set(GPIOB, GPIO_PIN_6);
    gpio_bit_reset(GPIOB, GPIO_PIN_6);    
    return 0;
}

uint16_t ADS115_read_conversion(void){
    uint16_t out =0;
    master_START();
    my_i2c_send(0x90);
//    ACK();
    my_i2c_send(0x00);    
    my_delay(30);
    master_START();
//    ACK();
    my_i2c_send(0x90);
    my_i2c_read();
    my_i2c_read();
    master_STOP();
    return out;
}
void SysTick_Handler(void)
{
    delay_decrement();
}

void my_delay(int time){
    int count = 0;
    for(int j=0;j<time;j++) count++;
}

void master_START(void){
    gpio_bit_set(GPIOB, GPIO_PIN_6);
    gpio_bit_set(GPIOB, GPIO_PIN_7);
    my_delay(10);
    gpio_bit_reset(GPIOB, GPIO_PIN_7);
    my_delay(10);
    gpio_bit_reset(GPIOB, GPIO_PIN_6);
    my_delay(10);    
}

void master_STOP(void){
    gpio_bit_reset(GPIOB, GPIO_PIN_7);
    my_delay(10);
    gpio_bit_set(GPIOB, GPIO_PIN_6);
    my_delay(10);
    gpio_bit_set(GPIOB, GPIO_PIN_7);
    my_delay(10);    
}
void ACK(void)
{
    gpio_bit_reset(GPIOB, GPIO_PIN_7);
        my_delay(1);
    gpio_bit_set(GPIOB, GPIO_PIN_6);
    my_delay(10);
    gpio_bit_reset(GPIOB, GPIO_PIN_6);
        my_delay(10);
        gpio_bit_set(GPIOB, GPIO_PIN_7);
    my_delay(10);
}
void ADS115_config(void){
    master_START();
    my_i2c_send(0x91);
//    my_i2c_send(0x01);
//    my_i2c_send(0x8F);
//    my_i2c_send(0xA3);
}

void USART0_IRQHandler(void)            //CH-08
{
        uint8_t cur_buff[5]={0};
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){            
            UART0_buff[UART0_count]=usart_data_receive(USART0);
            cur_buff[0]=usart_data_receive(USART0);
            usbd_ep_send (&usbd_cdc, CDC_IN_EP, cur_buff, 1);
            UART0_count++;        
            UART0_flag=1;
            if (UART0_count>=200) UART0_count=0;    
            usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);                        
    }
}

void USART1_IRQHandler(void)        //SIM800C
{
    uint8_t cur_buff[5]={0};
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
    {            
        UART1_buff[UART1_count]=usart_data_receive(USART1);
        cur_buff[0]=usart_data_receive(USART1);
        usbd_ep_send (&usbd_cdc, CDC_IN_EP, cur_buff, 1);
        UART1_count++;            
        if (UART1_count>=200) UART1_count=0;        
        usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);                        
    }
}
uint8_t check_FLAG_CONT_CH(void){
        for(uint8_t g=1;g<200;g++){
                if (UART0_buff[g]=='K' & UART0_buff[g-1]=='O') {
                        UART0_buff[g]=0xFF;
                        UART0_buff[g-1]=0xFF;
                        return 1;
                }
        }
        return 0;
}

void RTC_IRQHandler(void)
{
    if (rtc_flag_get(RTC_FLAG_SECOND) != RESET){
        /* clear the RTC second interrupt flag*/
        rtc_flag_clear(RTC_FLAG_SECOND);

        /* enable time update */
        timedisplay = 1;

        /* wait until last write operation on RTC registers has finished */
        rtc_lwoff_wait();
        /* reset RTC counter when time is 23:59:59 */
        if (rtc_counter_get() == 0x00015180){
            rtc_counter_set(0x0);
            /* wait until last write operation on RTC registers has finished */
            rtc_lwoff_wait();
        }
    }
}

void EXTI5_9_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_8)){
        int statusBtn = gpio_input_bit_get(GPIOC, GPIO_PIN_10);
            if (statusBtn)
            {
//                DeviceOff();
//                    en_butt_flag=1;
//                    en_butt_count=0;
            }
    }
    exti_interrupt_flag_clear(EXTI_8);
}

