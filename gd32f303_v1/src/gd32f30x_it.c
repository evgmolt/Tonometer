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
extern uint8_t mode;
extern int16_t CurrentPressure;
extern short int  save_clear[10000];
extern uint32_t main_index;
extern uint32_t send_counter;
extern int i2c_out_K;
extern short int save_dir[10000];
extern uint32_t save_dir_counter;
extern short int EnvelopeArray[10000];
extern uint16_t count_send_bluetooth;
extern uint8_t start_send_ble_flag;
extern uint8_t start_finish_ble_flag;
extern uint16_t frequency;

const int LoLimit = 50;  //ms - 200 
const int HiLimit = 250; //ms - 30

uint8_t EN_BUTT_FLAG=0;
uint8_t EN_BUTT_count=0;

uint8_t UART1_buff[200]={0};
uint8_t UART1_count=0;

uint8_t UART0_buff[200]={0};
uint8_t UART0_count=0;

uint8_t FLAG_CONT_CH=0;

int slim_sr=0;
int slim_sr_final=0;
int slim_sr_OLD=0;
uint8_t slim_count=0;
float slim_K=1;

int16_t _detectLevel_start = 4;
int16_t _detectLevel = 4;
int16_t _detectLevel_comp_UP = 15;
int16_t _detectLevel_comp_DOWN = 8;
int16_t _minDetectLevel = 5;
int16_t _lockInterval = 50;
double _detectLevelCoeff=0.6;
int16_t cur_dir_save=0;
int16_t _maxD=0;
uint8_t Wave_detect_FLAG=0;
int16_t Wave_detect_time=0;
int16_t Wave_detect_time_OLD=0;
int16_t T_Wave=0;
uint8_t Wave_ind_FLAG=0;
int16_t silence_time_start=0;
int16_t MAX_dir_wave=0;
int16_t puls_buff[50]={0};
uint8_t puls_counter=0;
int16_t sector_scan = 20;
int16_t sector_start_scan=0;
uint8_t finish_6_flag=0;

uint16_t detect_FLAG=0;
uint16_t finish_time=500;


uint32_t MAX_counter=0;
uint16_t Time_measurement=50; 

int DCArrayWindow = 60;
int ACArrayWindow = 6;

uint8_t UART0_flag=0;

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
    if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP)){        
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);		
				timer_interrupt_enable(TIMER1, TIMER_INT_UP);
				timer_enable(TIMER1);			
			
				button_touched = gpio_input_bit_get(GPIOC, GPIO_PIN_8);
				if (button_touched) {
					button_touched_counter++;
					if (button_touched_counter > DEBONCE_INTERVAL) {
						button_pressed = 1;
					}
				}
				else {
					button_touched = 0;
					button_pressed = 0;
					button_touched_counter = 0;
				}
				
				if (button_pressed) {
					button_pressed_counter++;
				}
				else {
					if (button_pressed_counter > 0) {
						button_released = 1;
					}
				}
				
				if (mode == INIT_START) {
						if (button_pressed_counter > GO_TO_TEST_INTERVAL) {
							ILI9341_FillScreen(ILI9341_WHITE);
							timer_2_stop();
							mode = PRESSURE_TEST;
						}
				}
				else {
					if (button_pressed_counter > SWITCH_OFF_INTERVAL) {
						device_OFF();
					}
				}
			
				//bluetooth_check();
			
/*				if (EN_BUTT_FLAG == 1) {
						EN_BUTT_count++;
				}				 
				if (EN_BUTT_count >= 10) {	
						ILI9341_FillScreen(ILI9341_WHITE);
						timer_2_stop();
						mode = PRESSURE_TEST;
				}
				else if (EN_BUTT_count>1 & mode!=4 & mode!=0) {
					mode = KEY_OFF;
					//device_OFF();					
				}					
				if (EN_BUTT_FLAG==0 & mode == START_SCREEN) TFT_print();
				if (mode == PRESSURE_TEST) time_display(rtc_counter_get());	*/
    }
}

void TIMER2_IRQHandler(void)
{
		int16_t  CUR_adc=0;	
    if(SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP)){
        /* clear update interrupt bit */
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);		
				timer_interrupt_enable(TIMER2, TIMER_INT_UP);
				timer_enable(TIMER2);			

				if (mode == PUMPING_MANAGEMENT){																																								//////////////////////////////////
						if (convert_save_16()){
								if (main_index<300) convert_NO_save();
								else if (main_index>300){
										save_dir[main_index-1]=slim_mas(save_clear, 30, 4);											
								}	
								else 		save_dir[main_index-1]=0;				
								
								if (main_index >= DELAY_AFTER_START){										
										cur_dir_save=GetDerivative(save_dir, main_index-1);
										usb_send_16(cur_dir_save,_maxD);
										if (cur_dir_save>_maxD){
												_maxD=cur_dir_save;
												MAX_counter=main_index;
										}
										if (CurrentPressure > MIN_PRESSURE){												
												if (main_index > MAX_counter + SEC_AFTER_MAX * frequency){	
														main_index=0;		
														save_dir_counter=0;		
														Wave_detect_FLAG=0;													
														_maxD=0;	
														//MAX_counter=0;
														mode = MEASUREMENT;
												}
										}										
								}
								
								
								if (main_index>400 & CurrentPressure<10){
										main_index=0;		
										save_dir_counter=0;		
										Wave_detect_FLAG=0;	
										_maxD=0;		
										_detectLevel_comp_UP=15;
										_detectLevel=_detectLevel_start;
										silence_time_start=0;
										timer_2_stop();
										print_error(2);
										timer_1_start();									
										mode = START_SCREEN;
								}
								
								if (main_index>9990){
										main_index=0;		
										save_dir_counter=0;		
										Wave_detect_FLAG=0;	
										_maxD=0;		
										_detectLevel_comp_UP=15;
										_detectLevel=_detectLevel_start;
										silence_time_start=0;
										timer_2_stop();
										print_error(3);
										timer_1_start();									
										mode = START_SCREEN;
								}
								
								
						}
				}
				else if (mode == MEASUREMENT) { //convers_save(); //usb_send_i2c_convers();	                          ///////////////////////////////////////////////
						if (convert_save_16()) {
								if (main_index>300){
										save_dir[main_index-1]=slim_mas(save_clear, DCArrayWindow, ACArrayWindow);											
								}	
								else 		save_dir[main_index-1]=0;				
													
								if (main_index >= DELAY_AFTER_PUMPING){										
										cur_dir_save=GetDerivative(save_dir, main_index-1);
										usb_send_16(cur_dir_save, _maxD); //save_clear[main_index-1]);  //_maxD);	 cur_dir_save
									
										//if (Wave_detect_FLAG==1 & main_index > (silence_time_start+_lockInterval*4)) finish_6_flag=1;
											
										if (cur_dir_save>_detectLevel & (main_index-1)>(silence_time_start+_lockInterval)) Wave_detect_FLAG=1;
										if (Wave_detect_FLAG==1 & (main_index-1)>(silence_time_start+_lockInterval)){
												if (cur_dir_save>_maxD) {
														_maxD=cur_dir_save;
														MAX_counter=main_index-1;
												}
												else if (cur_dir_save<_detectLevel){
														Wave_detect_time_OLD=Wave_detect_time;
														Wave_detect_time=MAX_counter-1;																														
														puls_buff[puls_counter++]=MAX_counter-1;
														Wave_ind_FLAG=1;												
														_lockInterval=(Wave_detect_time-Wave_detect_time_OLD)/2;
														if (_lockInterval>HiLimit | _lockInterval<LoLimit) _lockInterval=50;
														silence_time_start=MAX_counter-1;
														_detectLevel=_maxD*0.7;
														if (_detectLevel<4) _detectLevel=4;
														_maxD=0;
														Wave_detect_FLAG=0;
												}
										}						
								}						
						}
				}
				else if (mode == SEND_SAVE_BUFF_MSG) {
						if (usb_send_save(save_dir,EnvelopeArray)){			
								mode = INIT_START;
								timer_2_stop();
								set_FLAG();
								main_index=0;
								send_counter=0;
						}
				}				
    }
}

void my_i2c_send(uint8_t data){
	uint8_t bit;
//	gpio_init(GPIOB, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
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
//	gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
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
//	ACK();
	my_i2c_send(0x00);	
	my_delay(30);
	master_START();
//	ACK();
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
	int count;
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
//	my_i2c_send(0x01);
//	my_i2c_send(0x8F);
//	my_i2c_send(0xA3);
}

void USART0_IRQHandler(void)			//CH-08
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

void USART1_IRQHandler(void)		//SIM800C
{
		uint8_t cur_buff[5]={0};
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)){			
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

/*void EXTI5_9_IRQHandler(void)
{
	if (RESET != exti_interrupt_flag_get(EXTI_8)){
		int statusBtn = gpio_input_bit_get(GPIOC, GPIO_PIN_8);
			if (statusBtn){
					EN_BUTT_FLAG=1;
					EN_BUTT_count=0;
			}
			if (statusBtn == 0 & mode == KEY_OFF){
					device_OFF();
			}			
			if (statusBtn == 0){				
					if (mode == PUMPING_MANAGEMENT & EN_BUTT_count<2) {
							timer_1_start();
							mode = START_SCREEN;
					}						
					else if (mode == START_SCREEN & EN_BUTT_count<2) {
							timer_1_stop();							
							ILI9341_FillRectangle(0, 0, 240, 280, ILI9341_WHITE);							
							ILI9341_FillRectangle(100, 270, 140, 50, ILI9341_WHITE);						
												
							count_send_bluetooth=0;
						
							CurrentPressure=0;
							i2c_calibration();
							comp_ON;
							valve_1_ON;
							valve_2_ON;
							//timer_2_stop();	
							_lockInterval=50;
							sector_start_scan=0;
							main_index=0;		
							save_dir_counter=0;		
							Wave_detect_FLAG=0;	
							_maxD=0;		
							_detectLevel_comp_UP=15;
							_detectLevel=_detectLevel_start;
							silence_time_start=0;
							puls_counter=0;			
							detect_FLAG=0;
							timer_2_start();
							CurrentPressure=0;
							finish_6_flag=0;
							mode = PUMPING_MANAGEMENT; 							
					}	
					else if (mode == MEASUREMENT & EN_BUTT_count<2) {
							timer_2_stop();
							mode = START_SCREEN; 							
					}
					else if (mode == PRESSURE_TEST & EN_BUTT_count<2) {
							ILI9341_FillRectangle(0, 0, 240, 280, ILI9341_WHITE);							
							ILI9341_FillRectangle(100, 270, 140, 50, ILI9341_WHITE);	
							//clear_monitor();	
							timer_1_start();
							mode = START_SCREEN;
							//timer_1_start();
					}
					else if (EN_BUTT_count>10){
							//ILI9341_FillScreen(ILI9341_WHITE);
							mode = PRESSURE_TEST;
							timer_1_start();
							//EN_BUTT_FLAG=0;							
					}
					
					EN_BUTT_count=0;
					EN_BUTT_FLAG=0;
			}	
	}
	exti_interrupt_flag_clear(EXTI_8);
}*/

void EXTI5_9_IRQHandler(void)
{
	if (RESET != exti_interrupt_flag_get(EXTI_8)){
		int statusBtn = gpio_input_bit_get(GPIOC, GPIO_PIN_8);
			if (statusBtn){
//					EN_BUTT_FLAG=1;
//					EN_BUTT_count=0;
			}
	}
	exti_interrupt_flag_clear(EXTI_8);
}

