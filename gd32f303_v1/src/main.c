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

//#include "img_all.h"
#include "bat_clr_44_24.h"
#include "bat_dif_7_22.h"
#include "heart_31_30.h"
#include "bluetooth_15_24.h"
#include "gsm_29_18.h"
#include "heartX3_45_27.h"
#include "SYS_46_36.h"
#include "DIA_45_35.h"
#include "text_H_GREEN.h"
#include "text_H_RED.h"
#include "text_H_YELLOW.h"
#include "text_L_BLACK.h"

//#include ""

#define  BKP_DATA_REG_NUM              42
#define FMC_PAGE_SIZE           ((uint16_t)0x30U)
#define FMC_WRITE_START_ADDR    ((uint32_t)0x0807E000U)
#define FMC_WRITE_END_ADDR      ((uint32_t)0x0807E030U)

#define NCoef 2 

/* enter the second interruption,set the second interrupt flag to 1 */
__IO uint32_t timedisplay;


#define I2C0_OWN_ADDRESS7      0x72 
#define I2C0_SLAVE_ADDRESS7    0x91 

extern uint8_t EN_BUTT_FLAG;
extern uint8_t EN_BUTT_count;
extern uint8_t Wave_ind_FLAG;
extern uint8_t Wave_detect_FLAG;
extern int16_t T_Wave;
extern int16_t _detectLevel;
extern int16_t silence_time_start;
extern int16_t puls_buff[50];
extern uint8_t puls_counter;
extern int16_t _maxD;
extern int16_t _detectLevel_start;
extern uint8_t UART0_buff[200];
extern uint8_t UART0_count;
extern uint8_t finish_6_flag;

double puls_out=0;
uint8_t puls_cur_counter=0;

uint32_t *ptrd;
uint32_t address = 0x00000000U;
uint32_t SERIAL[9]   = {'0','2','0','2','2','2','0','0','1'};
/* calculate the number of page to be programmed/erased */
uint32_t PageNum = (FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) / FMC_PAGE_SIZE;
/* calculate the number of page to be programmed/erased */
uint32_t WordNum = ((FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) >> 2);

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

int16_t i2c_out_norm=0;
int16_t i2c_out=0;
int i2c_out_K=0;
uint8_t indicate_charge_toggle=1;

uint16_t cur_day=13, cur_month=12, cur_year=2022;
uint32_t cur_thh=0,cur_tmm=0,cur_tss=0;
uint32_t cur_tim=0;

uint8_t bluetooth_status=0;
uint8_t bonus_byte=0;
/*
mode device:
0 - init start;
1 - start screen
2 - key off
3 - pumping measurement
4 - usb charging
5 - pressure test
6 - measurement
7 - send save buff msg
8 -
*/

#define modeInitStart 					0
#define modeStartScreen 				1
#define modeKeyOff		  				2
#define modePumpingMeasurement 	3
#define modeUsbCharging					4
#define modePressureTest				5
#define modeMeasurement					6
#define modeSendSaveBuffMsg			7

uint8_t mode = modeInitStart;

uint8_t sim800_FLAG=0;
uint8_t rang_bat_old=99;

uint8_t i2c_transmitter[16];
uint8_t i2c_receiver[16];

uint8_t send_buff[100]={0};

uint8_t buff097[10]={0};

usb_dev usbd_cdc;

uint16_t adc_value[8];
uint16_t num_string=0;

uint16_t count_send_bluetooth=0;
uint16_t size_pack=20;

short int save_clear[10000]={0};
uint32_t save_clear_counter=0;
short int save_dir[10000]={0};
uint32_t save_dir_counter=0;
short int EnvelopeArray[10000]={0};

uint32_t send_counter=0;
float rate=18.69;

void timer_config_1(void)
{
    /* ----------------------------------------------------------------------------
    TIMER1 Configuration: 
    TIMER1CLK = SystemCoreClock/(12000*10000) = 1s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 11999;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    timer_enable(TIMER1);
}

void nvic_config_1(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 1, 1);
}

void timer_config_2(void)
{
    /* ----------------------------------------------------------------------------
    TIMER2 Configuration: 
    TIMER1CLK = SystemCoreClock/(1000*f) = ... s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 999;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = (120000/frequency)-1;//= 599; 479
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);

    //timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    //timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    //timer_enable(TIMER2);
}

void nvic_config_2(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER2_IRQn, 1, 1);
}

int main(void)
{
		uint16_t i;	
		
		nvic_configuration();	  // RTC
		time_init();						// RTC
	
		GPIO_config();	
		systick_config();

		
		boot_mode();		

    rcu_config();																	// USB
    gpio_config();																// USB    
    usbd_init(&usbd_cdc, &cdc_desc, &cdc_class);	// USB 
    nvic_config();																// USB     
    usbd_connect(&usbd_cdc);											// USB 
	
		nvic_config_1();		// timer 1
		timer_config_1();		// timer 1
	
		nvic_config_2();		// timer 2
		timer_config_2();		// timer 2
		
		
		i2c_config();																												//I2C ADS1115
		ADS1115_config(0b00000010, 0x00, 0x00);															//Lo
		ADS1115_config(0b00000011, 0xFF, 0xFF);															//Hi
		ADS1115_config(0b00000001, Hi_ADS1115_config, Lo_ADS1115_config);		//preconfig
		//ADS1115_read_IT();
		//i2c_init();

		ADC_rcu_config();																				// ADC0
		ADC_gpio_config();																			// ADC0
		dma_config();																						// ADC0
		adc_config();																						// ADC0
		adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);	// ADC0
		
		nvic_irq_enable(USART1_IRQn, 0, 0);							// UART1
		usart_config_1();																// UART1
		usart_interrupt_enable(USART1, USART_INT_RBNE);	// UART1
		
		nvic_irq_enable(USART0_IRQn, 0, 0);							// UART0
		usart_config_0();																// UART0
		usart_interrupt_enable(USART0, USART_INT_RBNE);	// UART0
		
    //while (USBD_CONFIGURED != usbd_cdc.cur_status) {/* wait for standard USB enumeration is finished */}	
		
		ILI9341_Init();
		ILI9341_Touch_init();
		
		if (mode == modeUsbCharging){
				ILI9341_FillScreen(ILI9341_BLACK);
				ILI9341_FillRectangle(50, 150, 150, 70, ILI9341_BLACK);
				ILI9341_FillRectangle(52, 152, 146, 66, ILI9341_WHITE);
				ILI9341_FillRectangle(40, 150+35-20, 10, 40, ILI9341_BLACK);
				ILI9341_FillRectangle(40+2, 150+35-20+2, 10-2, 40-4, ILI9341_WHITE);
		}
		else ILI9341_FillScreen(ILI9341_WHITE);
		
		button_interrupt_config();
		
		uint8_t buff2[10]={0};
		EN_BUTT_FLAG=1;			

		if (mode == modeInitStart){		
		ILI9341_DrawImage(72, 279, 31, 30, (const uint16_t*)heart);
		ILI9341_DrawImage(5, 255, 15, 24, (const uint16_t*)bluetooth);
		ILI9341_DrawImage(22, 258, 29, 18, (const uint16_t*)gsm);
		ILI9341_DrawImage(65, 245, 45, 27, (const uint16_t*)heartX3);
		ILI9341_DrawImage(5, 10, 46, 36, (const uint16_t*)SYS);
		ILI9341_DrawImage(5, 133, 45, 35, (const uint16_t*)DIA);
		print_num_H(888,235,10,YELLOW);
		print_num_H(888,235,120,RED);
		print_num_H(888,235,250,BLACK);		
		}
		else if (mode == modePressureTest){
				ILI9341_FillRectangle(70, 150, 100, 50, ILI9341_WHITE);
		}	
		
		// waiting for button release, or sitting mode?
		while (gpio_input_bit_get(GPIOC, GPIO_PIN_8)==1){}	
		delay_1ms(300);
		while (gpio_input_bit_get(GPIOC, GPIO_PIN_8)==1){}
		
		EN_BUTT_FLAG=0;	
		
			
		comp_OFF;
		valve_1_OFF;
		valve_2_OFF;	
		
		/*
		my_send_string_UART_0("AT+NAME=TONOMETER\0\n",strlen("AT+NAME=TONOMETER\0\n"));
		delay_1ms(1000);
		my_send_string_UART_0("AT+RX\0\n",strlen("AT+RX\0\n"));
		delay_1ms(1000);
		my_send_string_UART_0("AT+MODE=?\0\n",strlen("AT+MODE=?\0\n"));		
		delay_1ms(1000);
		//my_send_string_UART_0("AT+ADDR=1234567890AB\0\n",strlen("AT+ADDR=1234567890AB\0\n"));		
		//delay_1ms(1000);
		my_send_string_UART_0("AT+ROLE=?\0\n",strlen("AT+ROLE=?\0\n"));		
		delay_1ms(1000);
		uint8_t UP[10]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
				
		for (int i=0;i<10;i++){
				usart_data_transmit(USART0, (uint8_t)UP[i]);
				while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));	
		}
		*/	
		//my_send_string_UART_0("AT\0\n",strlen("AT\0\n"));	
		if (sim800_FLAG) {}	  //GSM module ...		
		delay_1ms(1000);
		if (mode!=4) {			
				ILI9341_FillRectangle(72, 279, 31, 30, ILI9341_WHITE);
				ILI9341_FillRectangle(5, 255, 15, 24, ILI9341_WHITE);
				ILI9341_FillRectangle(22, 258, 29, 18, ILI9341_WHITE);
				ILI9341_FillRectangle(65, 245, 45, 27, ILI9341_WHITE);
				ILI9341_FillRectangle(5, 10, 46, 36, ILI9341_WHITE);
				ILI9341_FillRectangle(5, 133, 45, 35, ILI9341_WHITE);				

				ILI9341_FillRectangle(55, 10, 180, 106, ILI9341_WHITE);
				ILI9341_FillRectangle(55, 120, 180, 106, ILI9341_WHITE);
				ILI9341_FillRectangle(112, 250, 123, 64, ILI9341_WHITE);	
		}		
			
		uint16_t count_0=0; 		

					
		i2c_calibration();	
		
		//uint16_t x, y;
		
		fmc_program_check();
		delay_1ms(200);
		str_clear(UART0_buff,200);		
		
		/* PMU lock enable */
    rcu_periph_clock_enable(RCU_PMU);																// 
    /* BKP clock enable */																					//
    rcu_periph_clock_enable(RCU_BKPI);															// 
    /* enable write access to the registers in backup domain */			//	backap data (save data)
    pmu_backup_write_enable();																			// 
    /* clear the bit flag of tamper event */												//
    bkp_flag_clear(BKP_FLAG_TAMPER);																//
			
		//time_set(23,59,55);
		//write_backup_register(cur_day, cur_month, cur_year);		
		
		if (mode == modeInitStart) mode = modeStartScreen;
	
    while (1) {	
			
			//boot_mode();
			
			if (mode == modeKeyOff){				
					device_OFF();
			}
			
			if (UART0_flag==1){					
					for (int w=0;w<200;w++){
							if (finder_msg(UART0_buff)){																	
										ILI9341_FillRectangle(1, 1, 100, 100, ILI9341_RED);															
							}
							if (finder(UART0_buff, "OFF",0,&num_string)) {
									ILI9341_FillRectangle(1, 1, 100, 100, ILI9341_WHITE);
							}
					}
					UART0_flag=0;
			}
			
		//if (bluetooth_status) ILI9341_DrawImage(5, 255, 15, 24, (const uint16_t*)bluetooth);			
		//else ILI9341_FillRectangle(5, 255, 15, 24, ILI9341_WHITE);		
			
			if (Wave_ind_FLAG){				
					ILI9341_DrawImage(72, 279, 31, 30, (const uint16_t*)heart);
					delay_1ms(200);
					ILI9341_FillRectangle(72, 279, 31, 30, ILI9341_WHITE);
					Wave_ind_FLAG=0;
			}
			
			/*
       if(ILI9341_TouchGetCoordinates(&x, &y)) {
						//ILI9341_DrawPixel(240-x, 320-y, ILI9341_WHITE); 
						sprintf(buff097,"%4d",x);
						ILI9341_WriteString(1, 70, buff097, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
						sprintf(buff097,"%4d",y);
						ILI9341_WriteString(1, 90, buff097, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
       }
			*/ 		
			
		
			/*	
			sprintf(buff2,"%2d",mode);
			ILI9341_WriteString(5, 200, buff2, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
			sprintf(buff2,"%2d",EN_BUTT_count);
			ILI9341_WriteString(5, 220, buff2, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
			//sprintf(buff2,"%3d",i2c_out_norm);
			//ILI9341_WriteString(5, 240, buff2, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
			*/
			
			
			if (mode == modePumpingMeasurement){
					ILI9341_FillRectangle(65, 245, 45, 27, ILI9341_WHITE);
					comp_ON;
					valve_1_ON;
					valve_2_ON;						
					
					if (i2c_out_norm>=0 & i2c_out_norm<400) print_num_H(i2c_out_norm,235,120,RED);
				
					if (i2c_out_norm>=190) {
							_detectLevel = _detectLevel_start;						
							save_clear_counter=0;		
							save_dir_counter=0;		
							Wave_detect_FLAG=0;	
							silence_time_start=0;
							_maxD=0;							
							puls_counter=0;						
							mode = modeMeasurement;		
					}						
			}	
			else if (mode == modeStartScreen){	
					//clear_monitor();
					comp_OFF;
					valve_1_OFF;
					valve_2_OFF;						
			}
			else if (mode == modeMeasurement){				
					if (i2c_out_norm>=0 & i2c_out_norm<400) print_num_H(i2c_out_norm,235,120,RED);
					comp_OFF;
					valve_2_OFF;
					if (save_clear_counter>1+size_pack*(count_send_bluetooth+1)){						
							uint8_t c_summ=0;							
							uint8_t cur_buff_ble[400]={'0','2',0x05,count_send_bluetooth&0xFF,(count_send_bluetooth>>8)&0xFF,size_pack};
							
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
					if (i2c_out_norm<=50){
							//timer_2_stop();						                           ///////////////////////////////////////////////
						
							ILI9341_FillRectangle(55, 10, 180, 106, ILI9341_WHITE);
							ILI9341_FillRectangle(55, 120, 180, 106, ILI9341_WHITE);
							ILI9341_FillRectangle(112, 250, 123, 64, ILI9341_WHITE);	
						
							str_clear(EnvelopeArray,10000);
							GetArrayOfWaveIndexes(save_dir, puls_buff, puls_buff_NEW);
							f_sorting_MAX();
							CountEnvelopeArray(puls_buff_NEW,puls_buff_AMP);
							f_PSys_Dia();
							puls_convert();	
							bonus_byte=0;
							if (save_clear_counter>1000 & PSys>10 & PSys<300 & PDia>10 & PDia<300 & puls_out>10 & puls_out<300) {
									ILI9341_DrawImage(5, 10, 46, 36, (const uint16_t*)SYS);
									ILI9341_DrawImage(5, 133, 45, 35, (const uint16_t*)DIA);	
									print_SIS(PSys);
									print_DIA(PDia);									
									print_num_H((uint16_t)puls_out,235,250,BLACK);
								
									cur_tim = rtc_counter_get();
									m_hh = cur_tim / 3600;
									m_mm = (cur_tim % 3600) / 60;
									m_ss = (cur_tim % 3600) % 60;
									check_backup_register(&cur_day, &cur_month, &cur_year);
									if 	(cur_year>=255)	cur_year-=2000;
									//send_result_measurement(14, 15, 16, 17, 18, 19, 20, 21, 22, 23);																							
							}
							else {
									bonus_byte|=0x80;
									print_error(4);							
							}
							send_result_measurement((uint8_t)cur_day, (uint8_t)cur_month, (uint8_t)cur_year, (uint8_t)m_ss, (uint8_t)m_mm, (uint8_t)m_hh, (uint8_t)PSys, (uint8_t)PDia, (uint8_t)puls_out,bonus_byte);
							
							valve_1_OFF;
							valve_2_OFF;
							mode = modeSendSaveBuffMsg;								
							timer_1_start();
					}					
			}
			else if (mode == modePressureTest){	
					convert_NO_save();
					print_num_H(i2c_out_norm,235,10,YELLOW);
					//print_num_H(i2c_out_norm,235,120,RED);
					usb_send_16(i2c_out,0);
					delay_1ms(200);
			}
			else if (mode == modeUsbCharging){	
					if (gpio_input_bit_get(GPIOB, GPIO_PIN_8)==0) indicate_charge_toggle=1;
					print_bat_charg();				
					delay_1ms(2000);				
					if (gpio_input_bit_get(GPIOC, GPIO_PIN_10)==0) device_OFF();						
			}	
			
			//usbd_ep_send (&usbd_cdc, CDC_IN_EP, "(HC)-->\n", strlen("(HC)-->\n"));		
			
	/*		
			ILI9341_DrawImage(0, 0, 240, 320, (const uint16_t*)img_t_c_3);
			while(ILI9341_TouchGetCoordinates(&x, &y)==0) {my_delay(9999);}
			ILI9341_DrawImage(0, 0, 240, 320, (const uint16_t*)img_t_c_4);
			while(ILI9341_TouchGetCoordinates(&x, &y)==0) {my_delay(9999);}
	*/	
			
			
        if (0U == cdc_acm_check_ready(&usbd_cdc)) {
            cdc_acm_data_receive(&usbd_cdc);
        } else {
            cdc_acm_data_send(&usbd_cdc);
        }			
    }
}


void print_num_H(uint16_t num, uint16_t X0, uint16_t Y0, uint8_t color){
	double now=0;
	uint16_t now1=0;
	uint8_t max;
	if (num>=100)max=3;
	else if (num>=10) {
		max=2;
		if (color==BLACK) ILI9341_FillRectangle(X0-41*3, Y0, 41, 64, ILI9341_WHITE);
		else 							ILI9341_FillRectangle(X0-60*3, Y0, 60, 106, ILI9341_WHITE);
	}
	else {
		max=1;
		if (color==BLACK) ILI9341_FillRectangle(X0-41*3, Y0, 82, 64, ILI9341_WHITE);
		else 							ILI9341_FillRectangle(X0-60*3, Y0, 120, 106, ILI9341_WHITE);
	}
	
	
	
	for (int g=0;g<max;g++){
			now=pow(10,g);
			now1=now;			
			switch ((num/now1)%10)	{	
				case 0:					
					if (color==GREEN)				ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_0);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_0);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_0);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_0);
				break;
				case 1:
				 	if (color==GREEN)				ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_1);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_1);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_1);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_1);
				break;
				case 2:
				 	if (color==GREEN)				ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_2);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_2);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_2);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_2);
				break;
				case 3:
					if (color==GREEN)				ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_3);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_3);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_3);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_3);
				break;
				case 4:
					if (color==GREEN)				ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_4);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_4);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_4);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_4);
				break;
				case 5:
					if (color==GREEN)				ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_5);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_5);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_5);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_5);
				break;
				case 6:
					if (color==GREEN) 			ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_6);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_6);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_6);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_6);
				break;
				case 7:
					if (color==GREEN) 			ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_7);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_7);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_7);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_7);
				break;
				case 8:
					if (color==GREEN) 			ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_8);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_8);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_8);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_8);
				break;
				case 9:
					if (color==GREEN) 			ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_9);
					else if (color==RED) 		ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_9);
					else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_9);
					else if (color==BLACK) 	ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_9);
				break;
			}
	}
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

void TFT_print(void){
		uint8_t buff[100]={0};
		uint16_t adc_1=0;
		uint8_t rang_bat=0;
	
		adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
		delay_1ms(100);		
		adc_1=adc_value[0]*3300/(0xFFF);
		
		if (adc_1<1800) return;

		if (adc_1>2430) 									rang_bat=5;
		else if (adc_1<2430 & adc_1>2364) rang_bat=5;
		else if (adc_1<2364 & adc_1>2298) rang_bat=4;
		else if (adc_1<2298 & adc_1>2232) rang_bat=3;
		else if (adc_1<2232 & adc_1>2166) rang_bat=2;
		else if (adc_1<2166 & adc_1>2100) rang_bat=1;
		else if (adc_1<2100) 							rang_bat=0;
		
		
		if (rang_bat_old!=rang_bat){	
				ILI9341_DrawImage(6, 285, 44, 24, (const uint16_t*)bat_clr);			
				for (int i1=0;i1<rang_bat;i1++){
						ILI9341_DrawImage(42-i1*8, 286, 7, 22, (const uint16_t*)bat_dif);
				}
				rang_bat_old=rang_bat;
		}		
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
		if (gpio_input_bit_get(GPIOB, GPIO_PIN_10)==0){
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
				while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));		///////////////////////////////////////////////////////////////////////////
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
		i2c_start_on_bus(I2C0); 										///////////////////////////////////////////////////////////////////////////
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
//		i2c_transmitter[1]=0x01;
	
	
		
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

void GPIO_config(void){
		rcu_periph_clock_enable(RCU_GPIOB);		
		gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);	  //POWER_ON_DETECT
		gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_10);	//ADS_RDY  --EXTI
		gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);	// STDBY
		gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_9);	// CHRG
	
		rcu_periph_clock_enable(RCU_GPIOC);
		gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);	  //SIM_EXT
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);	  		//SIM_PWKEY
		gpio_init(GPIOC, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);					//KEY IN
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);	  		//POWER_ON
		gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);	  //USB_ON	
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);	  		//COMP_ON/OFF
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);	  		//VALVE1_OP/CL
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);	  		//VALVE2_OP/CL			
}

void ADC_rcu_config(void){
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable ADC0 clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
}

void ADC_gpio_config(void){
    /* config the GPIO as analog mode */
    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
}

void adc_config(void){	
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

void dma_config(void){
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

void usart_config_0(void){
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

void my_send_string_UART_0(char *buf, uint8_t num){
		for(int j1=0;j1<num;j1++){
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

void my_send_string_UART_1(char *buf, uint8_t num){
		for(int j1=0;j1<num;j1++){
				usart_data_transmit(USART1, (uint8_t)buf[j1]);
				while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));			
		}
}
void str_clear(char *buff, uint16_t len){
		for (int i=0;i<len;i++){
				buff[i]=0;
		}
		//memset(buff,0,strlen(buff));
		buff[0] = '\0';
}

void nvic_configuration(void){
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(RTC_IRQn,1,0);
}

void rtc_configuration(void){
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

void time_display(uint32_t timevar){
		uint8_t buff[100]={0};
    uint32_t thh = 0, tmm = 0, tss = 0;
		
    /* compute  hours */
    thh = timevar / 3600;
    /* compute minutes */
    tmm = (timevar % 3600) / 60;
    /* compute seconds */
    tss = (timevar % 3600) % 60;
		
		check_backup_register(&cur_day, &cur_month, &cur_year);
		if (thh==0 & cur_day==0 & cur_month==0){

				cur_day=1;
				cur_month=1;
				write_backup_register(cur_day, cur_month, cur_year);
		}
		
		if (thh>23) {
				cur_day++; 
				thh=0;
				time_set(thh,tmm,tss);
				write_backup_register(cur_day, cur_month, cur_year);				
		}
		if (cur_day>=29) { //add calendar....
				cur_month++; 
				cur_day=1;	
				write_backup_register(cur_day, cur_month, cur_year);
		}
		if (cur_month>12) {
				cur_year++; 
				cur_month=1;
				write_backup_register(cur_day, cur_month, cur_year);
		}
		sprintf(buff,"%2d:",thh);
		ILI9341_WriteString(130, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		sprintf(buff,"%2d:",tmm);
		ILI9341_WriteString(160, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		sprintf(buff,"%2d",tss);
		ILI9341_WriteString(190, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);	
		
		sprintf(buff,"%2d.",cur_day);
		ILI9341_WriteString(130, 260, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		sprintf(buff,"%2d.",cur_month);
		ILI9341_WriteString(160, 260, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		sprintf(buff,"%4d",cur_year);
		ILI9341_WriteString(190, 260, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
}

void time_set(uint32_t tmp_hh,uint32_t tmp_mm,uint32_t tmp_ss){
    rtc_configuration(); 

		rtc_lwoff_wait();
				
		rtc_counter_set((tmp_hh*3600 + tmp_mm*60 + tmp_ss));
		
		rtc_lwoff_wait();

    bkp_write_data(BKP_DATA_0, 0xA5A5);	
}

void time_init(void){
	if (bkp_read_data(BKP_DATA_0) != 0xA5A5){
				time_set(0, 0, 0);
  }
	else{
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

void i2c_init(void){
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

void i2c_print(void){
	uint8_t buff1[10]={0};
	i2c_convers();
	i2c_out=(((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K);
	i2c_out_norm=((((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K)/rate);
	sprintf(buff1,"%6d",i2c_out);
	sprintf(buff1,"%6d",i2c_out_norm);
}

void button_interrupt_config(void){
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

void set_FLAG(void){
		//ILI9341_FillRectangle(5, 70, 30, 30, ILI9341_BLACK);
		//uint8_t buff1[10]={0};
		//sprintf(buff1,"%6d",save_clear_counter);
		//ILI9341_WriteString(5, 90, buff1, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
		
}
void reset_FLAG(void){
		ILI9341_FillRectangle(5, 70, 30, 30, ILI9341_WHITE);
}
void device_OFF(void){
		comp_OFF;
		valve_1_OFF;
		valve_2_OFF;
		ILI9341_FillScreen(ILI9341_BLACK);
		gpio_bit_reset(GPIOC, GPIO_PIN_9);
}
void i2c_calibration(void){
		uint8_t buff1[10]={0};
		i2c_out_K=0;
		for (int g=0;g<20;g++){			
				while (ADS1115_read_IT()==0){}
				i2c_out_K+=((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF);				
		}
		i2c_out_K=i2c_out_K/20;
}

void usb_send_i2c_convers(void){
		i2c_convers();	
		uint8_t send_buff[3]={25,i2c_receiver[1],i2c_receiver[0]};
		i2c_out=(((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K);
		i2c_out_norm=i2c_out/rate;
		usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 3);
}

void timer_2_start(void){
		timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    timer_enable(TIMER2);
}
void timer_2_stop(void){
	timer_disable(TIMER2);			
}
void timer_1_start(void){
		timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    timer_enable(TIMER1);
}
void timer_1_stop(void){
	timer_disable(TIMER1);			
}

uint8_t usb_send_save(int16_t *mass1, int16_t *mass2){
	for (int h=0;h<puls_counter;h++){
			if (send_counter==indexPSys | send_counter==indexPDia | send_counter==XMax)save_dir[send_counter]=1000;					
	}		
	
	uint8_t send_H1=(mass1[send_counter]>>8)&0xFF;
	uint8_t send_L1=mass1[send_counter]&0xFF;
	uint8_t send_H2=(mass2[send_counter]>>8)&0xFF;
	uint8_t send_L2=mass2[send_counter]&0xFF;
	
	uint8_t send_buff[5]={25,send_L1,send_H1,send_L2,send_H2};
	usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 5);
	send_counter++;
	if (send_counter>=save_clear_counter) return 1;
	else return 0;
}
short int convert_save_16(void){			
		if (ADS1115_read_IT()==0) return 0;

		save_clear[save_clear_counter]=(((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K);
		if (save_clear[save_clear_counter-1]<0) save_clear[save_clear_counter-1]=0;
		save_clear_counter++;			
		return 1;
}
short int convert_NO_save(void){			
		if (ADS1115_read_IT()==0) return 0;
		i2c_out=(((i2c_receiver[0]<<8)&0xFF00)+(i2c_receiver[1]&0xFF)-i2c_out_K);
		i2c_out_norm=i2c_out/rate;	
		return i2c_out;
}

void usb_send_16(short int T1, short int T2){
		uint8_t send_H1=(T1>>8)&0xFF;
		uint8_t send_L1=T1&0xFF;
		uint8_t send_H2=(T2>>8)&0xFF;
		uint8_t send_L2=T2&0xFF;
		uint8_t send_buff[5]={25,send_L1,send_H1,send_L2,send_H2};
		usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 5);
}

int16_t GetDerivative(int16_t *dataArr, int32_t Ind){
   if (Ind < (DerivativeAverageWidth+DerivativeShift)){
       return 0;
   }
   int32_t val1 = 0;
   int32_t val2 = 0;
   for (int i = 0; i < DerivativeAverageWidth; i++){
       val1 += dataArr[Ind - DerivativeAverageWidth + i];
       val2 += dataArr[Ind - DerivativeAverageWidth - DerivativeShift + i];
   }
   val1 /= DerivativeAverageWidth;
   val2 /= DerivativeAverageWidth;
   return (int16_t)(val1 - val2);
}

void GetArrayOfWaveIndexes(int16_t *valuesArray, int16_t *indexesArray, int16_t *indexes){    
    for (int i=0; i<puls_counter; i++)
    {
        puls_buff_NEW_MIN[i] = GetMinIndexInRegion(valuesArray, indexesArray[i]);
				puls_buff_AMP_MIN[i] = valuesArray[puls_buff_NEW_MIN[i]];
				indexes[i] = GetMaxIndexInRegion(valuesArray, indexesArray[i]);
				puls_buff_AMP[i]=valuesArray[indexes[i]];				
    }    
}

int GetMaxIndexInRegion(int16_t *sourceArray, int index){ 
    int range = 50;
    int16_t max = -200;
    int maxIndex = 0;
    for (int i1 = 0; i1 < range; i1++){
        //if (i - range / 2 < 0) continue;
        //if (i - range / 2 > strlen(sourceArray)) continue;
        if (sourceArray[index + i1 - range / 2] > max){
            max = sourceArray[index + i1 - range / 2];
            maxIndex = i1 - range / 2;
        }
    }		
    return index + maxIndex;
}

int GetMinIndexInRegion(int16_t *sourceArray_MIN,int index){		
		int range_MIN=100;
		int16_t min = 1000;
		int minIndex = 0;
		for (int i1 = 0; i1 < range_MIN; i1++){
				if (sourceArray_MIN[index+i1] < min){
						min=sourceArray_MIN[index+i1];
						minIndex=i1;
				}
		}			
    return index + minIndex;
}

uint8_t usb_send_slim_AMP(void){
	uint8_t send_H=(save_dir[send_counter]>>8)&0xFF;
	uint8_t send_L=save_dir[send_counter]&0xFF;
	uint8_t send_buff[3]={25,send_L,send_H};
	usbd_ep_send (&usbd_cdc, CDC_IN_EP, send_buff, 3);
	send_counter++;
	if (send_counter>=puls_counter) return 1;
	else return 0;
}
void CountEnvelopeArray(int16_t *arrayOfIndexes, int16_t *arrayOfValues){
    for (int i = 1; i < puls_counter; i++){
        int x1 = arrayOfIndexes[i - 1];
        int x2 = arrayOfIndexes[i];
        double y1 = arrayOfValues[i - 1];
        double y2 = arrayOfValues[i];
        double coeff = (y2 - y1) / (x2 - x1);
        for (int j = x1 - 1; j < x2; j++) {
            int ind = i + j;            
            EnvelopeArray[j] = y1 + coeff * (j - x1);
        }				
    }
}

void f_sorting_MAX(void){
		int16_t MaximumAmplitude=-100;
		uint8_t FLAG=1;	
		uint16_t mini_XMAX=0;
		int16_t z=0;
		uint8_t buff1[10]={0};		
		
		int level = 8;
    for (int i = 1; i < puls_counter - 1; i++){
				if (abs(puls_buff_AMP[i] - puls_buff_AMP[i - 1]) > level)
				{
						puls_buff_AMP[i] = (puls_buff_AMP[i - 1] + puls_buff_AMP[i + 1]) / 2;
				}
    }
		
		
		for (int i=0; i<puls_counter; i++){
				puls_buff_AMP[i]=puls_buff_AMP[i]-puls_buff_AMP_MIN[i];
		}
		
		for (int i=0; i<puls_counter; i++){
				if (puls_buff_AMP[i]>MaximumAmplitude){
						MaximumAmplitude=puls_buff_AMP[i];							
						mini_XMAX=i;
				}		
		}			
		
		//sprintf(buff1,"%4d",puls_counter);
		//ILI9341_WriteString(1, 50, buff1, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
		//sprintf(buff1,"%4d",mini_XMAX);
		//ILI9341_WriteString(1, 70, buff1, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
		
		while (FLAG==1){
				FLAG=0;
				for (int i=1; i<mini_XMAX; i++){
						if (puls_buff_AMP[i-1]>puls_buff_AMP[i]){
								z=puls_buff_AMP[i-1];
								puls_buff_AMP[i-1]=puls_buff_AMP[i];
								puls_buff_AMP[i]=z;
								//swap(puls_buff_AMP[i-1],puls_buff_AMP[i]);
								FLAG=1;
						}
				}
		}
		FLAG=1;
		while (FLAG==1){
				FLAG=0;
				for (int i=mini_XMAX+2; i<puls_counter; i++){
						if (puls_buff_AMP[i-1]<puls_buff_AMP[i]){
								z=puls_buff_AMP[i-1];
								puls_buff_AMP[i-1]=puls_buff_AMP[i];
								puls_buff_AMP[i]=z;
								//swap(puls_buff_AMP[i-1],puls_buff_AMP[i]);
								FLAG=1;
						}
				}
		}		
}



void f_PSys_Dia(void){
	double MaximumAmplitude=-100;
	
	for (int i=0; i<puls_counter; i++){
			if (puls_buff_AMP[i]>MaximumAmplitude){
					MaximumAmplitude=puls_buff_AMP[i];
					XMax=puls_buff_NEW[i];					
			}		
	}		
	
	int16_t ValueSys = 0.46 * MaximumAmplitude;
	int16_t ValueDia = 0.82 * MaximumAmplitude;	
	
	for (int i = XMax; i >= 200; i--){
			if (EnvelopeArray[i] < ValueSys){
					PSys = save_clear[i]/rate;
					indexPSys = i;
					break;
			}
	}
	for (int i = XMax; i < save_clear_counter; i++)
	{
    if (EnvelopeArray[i] < ValueDia)
    {
        PDia = save_clear[i]/rate;
        indexPDia = i;
        break;
    }
	}
}
uint16_t puls_convert(void){
		double level = 0.06;
		uint16_t intervals[50]={0};
		uint8_t buff1[10]={0};
		double first_puls=0;
		int16_t cur_puls=0;
		puls_out=0;
		puls_cur_counter=0;
		if (puls_counter<10) return 0;
		for (int m=3;m<puls_counter-3;m++){
				cur_puls=puls_buff[m]-puls_buff[m-1];
				if (cur_puls>LoLimit & cur_puls<HiLimit){
						first_puls+=cur_puls;
						puls_cur_counter++;
				}
		}		
		first_puls=first_puls/puls_cur_counter;
		//sprintf(buff1,"w-%3d",puls_cur_counter);
		//ILI9341_WriteString(1, 70, buff1, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
		
		puls_cur_counter=0;
		for (int m=1;m<puls_counter;m++){
				cur_puls=puls_buff[m]-puls_buff[m-1];
				if (cur_puls>LoLimit & cur_puls<HiLimit & cur_puls*1.5>first_puls & cur_puls/1.5<first_puls){
						puls_out+=cur_puls;
						intervals[puls_cur_counter]=cur_puls;
						puls_cur_counter++;	
				}
		}
		
		//sprintf(buff1,"c-%3d",puls_cur_counter);
		//ILI9341_WriteString(1, 90, buff1, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
		
		
		double Aver=puls_out/puls_cur_counter;
		double TwentyFivePercent = Aver / 4;
		int Counter = 0;
    double SumSqr = 0;
		for (int i = 0; i < puls_cur_counter; i++){
				double Diff = intervals[i] - Aver;
				if (abs(Diff) < TwentyFivePercent){
						SumSqr += Diff * Diff;
						Counter++;
				}
    }
		//sprintf(buff1,"c-%3d",Counter);
		//ILI9341_WriteString(1, 110, buff1, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
		
		double SKO = sqrt(SumSqr/Counter);
		if ((SKO/Aver)>level) {
				ILI9341_DrawImage(65, 245, 45, 27, (const uint16_t*)heartX3);
		}		
		
		puls_out=60/(puls_out/(puls_cur_counter*frequency));
		return puls_out;
}


void clear_monitor(void){
		ILI9341_FillRectangle(72, 279, 31, 30, ILI9341_WHITE);
		ILI9341_FillRectangle(5, 255, 15, 24, ILI9341_WHITE);
		ILI9341_FillRectangle(22, 258, 29, 18, ILI9341_WHITE);
		ILI9341_FillRectangle(65, 245, 45, 27, ILI9341_WHITE);
		ILI9341_FillRectangle(5, 10, 46, 36, ILI9341_WHITE);
		ILI9341_FillRectangle(5, 133, 45, 35, ILI9341_WHITE);				

		ILI9341_FillRectangle(55, 10, 180, 106, ILI9341_WHITE);
		ILI9341_FillRectangle(55, 120, 180, 106, ILI9341_WHITE);
		ILI9341_FillRectangle(112, 250, 123, 64, ILI9341_WHITE);	
}

int16_t slim_mas(uint16_t *mass_in, int16_t DC, int16_t AC){
		int32_t DCLevel = 0;
		int32_t ACLevel = 0;					
		for(int r=0;r<DC;r++){
				DCLevel+=mass_in[save_clear_counter-1-r];
		}
		DCLevel/=DC;	
		for (int j=0;j<AC;j++){
       ACLevel+=mass_in[save_clear_counter-1-j];
    }
		ACLevel/=AC;
		i2c_out=ACLevel;
		i2c_out_norm=i2c_out/rate;
		if (i2c_out_norm<0 & save_clear_counter<500) i2c_out_norm=0;
		mass_in[save_clear_counter-1]=ACLevel;	
		
		
		float ACoef[NCoef+1] = { 
        0.97913295295553560000, 
        -1.95826590591107120000, 
        0.97913295295553560000 
    }; 
 
    float BCoef[NCoef+1] = { 
        1.00000000000000000000, 
        -1.95778812550116580000, 
        0.95837795232608958000 
    }; 
 
    static float y[NCoef+1]; //output samples 
    static float x[NCoef+1]; //input samples 
    int n; 
 
    //shift the old samples 
    for(n=NCoef; n>0; n--) { 
       x[n] = x[n-1]; 
       y[n] = y[n-1]; 
    } 
 
    //Calculate the new output 
    x[0] = ACLevel; 
    y[0] = ACoef[0] * x[0]; 
    for(n=1; n<=NCoef; n++) 
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n]; 
		
		
		
		return (int16_t)y[0];
		//return ACLevel-DCLevel;
}

void print_error(uint8_t K){
		ILI9341_FillRectangle(55, 10, 180, 106, ILI9341_WHITE);
		ILI9341_FillRectangle(55, 120, 180, 106, ILI9341_WHITE);
		ILI9341_FillRectangle(112, 250, 123, 64, ILI9341_WHITE);	
		uint8_t _buff[15]={0};
		if (K==2){
				sprintf(_buff,"majetta error");		
				ILI9341_WriteString(1, 30, _buff, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);			
		}
		if (K==3){
				sprintf(_buff,"time fail");		
				ILI9341_WriteString(1, 30, _buff, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);			
		}
		if (K==4){
				sprintf(_buff,"measurement error");		
				ILI9341_WriteString(1, 30, _buff, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);			
		}
		
}

void cur_slim(uint16_t *mass, int16_t AC){
		
}

uint8_t boot_mode(void){
		if (gpio_input_bit_get(GPIOB, GPIO_PIN_0)){     //
				delay_1ms(10);
				if (gpio_input_bit_get(GPIOC, GPIO_PIN_8)){
						gpio_bit_set(GPIOC, GPIO_PIN_9);	
						mode = modeInitStart;
				}
				else if (gpio_input_bit_get(GPIOC, GPIO_PIN_10)){
						//gpio_bit_set(GPIOC, GPIO_PIN_9);	
						mode = modeUsbCharging;
				}
		}
}
void print_bat_charg(void){
		if(indicate_charge_toggle) {
						for (int i=1;i<6;i++){
						ILI9341_FillRectangle(200-i*29, 154, 25, 62, ILI9341_GREEN);						
				}					
				//ILI9341_FillRectangle(200-(rang_bat)*29, 154, 25, 62, ILI9341_GREEN);
				indicate_charge_toggle=0;
		}
		else {
				ILI9341_FillRectangle(52, 152, 146, 66, ILI9341_WHITE);
				indicate_charge_toggle=1;
		}
}
void bluetooth_init(void){
		//my_send_string_UART_0("AT\0\n",strlen("AT\0\n"));	
		//delay_1ms(1000);		
}
void bluetooth_check(void){
		if (finder(UART0_buff,"OK",0,0)) bluetooth_status=0;
		else bluetooth_status=1;	
		my_send_string_UART_0("AT\0\n",strlen("AT\0\n"));			
}

uint8_t finder(uint8_t *buff, uint8_t *_string, uint8_t _char, uint16_t *num){	
		uint8_t _flag=0;
		for (int j=0;j<200;j++){
				if (buff[j]==_string[0]){
						_flag=1;
						for (int k=0;k<strlen(_string);k++){
								if (buff[j+k]!=_string[k]) {
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

uint8_t finder_msg(uint8_t *buff){	
		uint8_t cur_SERIAL[7]={0};
		uint8_t cur_buff[30]={'A','T','+','N','A','M','E','=','T','O','N','O','M'};
		uint8_t _flag=0;
		uint8_t _string[20]={'0','2'};

		//ILI9341_WriteString(1, 30, _string, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		for (int j=0;j<200;j++){
				if (buff[j]==_string[0] & buff[j+1]==_string[1]){
						_flag=1;												
				}
				if (_flag) {
						if (buff[j+2]==0x04){
								uint8_t check_sum=0;
								for (int a=0;a<10;a++){
										check_sum+=buff[j+a];
								}								
								if (buff[j+10]==check_sum){
										SERIAL[2]=cur_SERIAL[0]=buff[j+3];
										SERIAL[3]=cur_SERIAL[1]=buff[j+4];
										SERIAL[4]=cur_SERIAL[2]=buff[j+5];
										SERIAL[5]=cur_SERIAL[3]=buff[j+6];
										SERIAL[6]=cur_SERIAL[4]=buff[j+7];
										SERIAL[7]=cur_SERIAL[5]=buff[j+8];
										SERIAL[8]=cur_SERIAL[6]=buff[j+9];
										
										fmc_erase_pages();  
										fmc_program();						
									
										buff[j]=0xFF;
									
										delay_1ms(200);									
										device_OFF();
									
										return 1;
								}
						}
						else if (buff[j+2]==0x03){
								uint8_t check_sum=0;
								for (int a=0;a<10;a++){
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
		/*									
		uint8_t buff[100]={0};
		sprintf(buff,"%2d:",cur_thh);
		ILI9341_WriteString(130, 50, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		sprintf(buff,"%2d:",cur_tmm);
		ILI9341_WriteString(160, 50, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		sprintf(buff,"%2d",cur_tss);
		ILI9341_WriteString(190, 50, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);	
		
		sprintf(buff,"%2d.",cur_day);
		ILI9341_WriteString(130, 80, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		sprintf(buff,"%2d.",cur_month);
		ILI9341_WriteString(160, 80, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
		sprintf(buff,"%4d",cur_year);
		ILI9341_WriteString(190, 80, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
*/
									
										buff[j]=0xFF;
										return 2;
								}
						}
				}
		}
		return 0;
}

void fmc_erase_pages(void){
    uint32_t EraseCounter;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < PageNum; EraseCounter++){
        fmc_page_erase(FMC_WRITE_START_ADDR + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

void fmc_program(void){
		uint8_t cur_count=0;
    /* unlock the flash program/erase controller */
    fmc_unlock();

    address = FMC_WRITE_START_ADDR;

    /* program flash */
    while(address < FMC_WRITE_END_ADDR){
        fmc_word_program(address, SERIAL[cur_count++]);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END);
        fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
        fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

void fmc_program_check(void){	
		uint8_t cur_SERIAL[7]={0};
		uint8_t cur_buff[30]={'A','T','+','N','A','M','E','=','T','O','N','0','2'};
		uint8_t cur_count=0;
    uint32_t i;

    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;
		
		if((*ptrd) == '0' & (*(ptrd+1)) == '2'){
				SERIAL[2]=cur_SERIAL[0]=*(ptrd+2);	
				SERIAL[3]=cur_SERIAL[1]=*(ptrd+3);
				SERIAL[4]=cur_SERIAL[2]=*(ptrd+4);
				SERIAL[5]=cur_SERIAL[3]=*(ptrd+5);
				SERIAL[6]=cur_SERIAL[4]=*(ptrd+6);
				SERIAL[7]=cur_SERIAL[5]=*(ptrd+7);
				SERIAL[8]=cur_SERIAL[6]=*(ptrd+8);				
		}
		
		else{
				SERIAL[0]='0';
				SERIAL[1]='2'; 
				SERIAL[2]='0';	cur_SERIAL[0]='0';
				SERIAL[3]='0';	cur_SERIAL[1]='0';
				SERIAL[4]='0';	cur_SERIAL[2]='0';
				SERIAL[5]='0';	cur_SERIAL[3]='0';
				SERIAL[6]='0';	cur_SERIAL[4]='0';
				SERIAL[7]='0';	cur_SERIAL[5]='0';
				SERIAL[8]='0';	cur_SERIAL[6]='0';
				fmc_erase_pages();
				fmc_program();				
		}		
		strncat(cur_buff,cur_SERIAL,7);
		strncat(cur_buff,"\0\n",2);
		my_send_string_UART_0(cur_buff,strlen(cur_buff));
}

void write_backup_register(uint16_t day, uint16_t month, uint16_t year){
    uint32_t temp = 0;	
		BKP_DATA10_41(10) = day;
		BKP_DATA10_41(11) = month;
		BKP_DATA10_41(12) = year;
}

void check_backup_register(uint16_t *_day, uint16_t *_month, uint16_t *_year){
    uint32_t temp = 0;    
		*_day = BKP_DATA_GET(BKP_DATA10_41(10));
		*_month = BKP_DATA_GET(BKP_DATA10_41(11));
		*_year = BKP_DATA_GET(BKP_DATA10_41(12));
}

void send_result_measurement(uint8_t c_day, uint8_t c_month, uint8_t c_year, uint8_t c_ss, uint8_t c_mm, uint8_t c_hh, int16_t sis, int16_t dia, int16_t pressure, int16_t bonus){		
		uint8_t cur_buff[13]={'0','2',0x01, c_day, c_month, c_year, c_ss, c_mm, c_hh, sis, dia, pressure, bonus};		
		uint8_t c_summ=0;		
		for (int q=0;q<13;q++){
				c_summ+=cur_buff[q];
		}			
		cur_buff[13]=c_summ;
		my_send_string_UART_0(cur_buff,14);
}
void print_SIS(int16_t IN){		
		if (IN<140) print_num_H(IN,235,10,GREEN);
		else if (IN<160) print_num_H(IN,235,10,YELLOW);
		else print_num_H(IN,235,10,RED);

}
void print_DIA(int16_t IN){
		if (IN<90) print_num_H(IN,235,120,GREEN);
		else if (IN<99) print_num_H(IN,235,120,YELLOW);
		else print_num_H(IN,235,120,RED);
}

