#include "main.h"
#include "Display.h"
#include "bat_dif_7_22.h"
#include "bat_clr_44_24.h"
#include "systick.h"
#include "ili9341.h"
#include "text_H_GREEN.h"
#include "text_H_RED.h"
#include "text_H_YELLOW.h"
#include "text_L_BLACK.h"
#include "heart_31_30.h"
#include "heartX3_45_27.h"
#include "bluetooth_15_24.h"
#include "gsm_29_18.h"
#include "SYS_46_36.h"
#include "DIA_45_35.h"

#define BATT_RANG_MAX 2400
#define BATT_RANG_5 2364
#define BATT_RANG_4 2298
#define BATT_RANG_3 2232
#define BATT_RANG_2 2166
#define BATT_RANG_1 2100

//Max 2290

extern uint16_t adc_value[8];

void print_heart(bool show){
    if (show) {
            ILI9341_DrawImage(72, 279, 31, 30, (const uint16_t*)heart);
        }
        else {
            ILI9341_FillRectangle(72, 279, 31, 30, ILI9341_WHITE);    
        }
}

void print_heartX3(bool show){
    if (show) {
            ILI9341_DrawImage(65, 245, 45, 27, (const uint16_t*)heartX3);
        }
        else {
            ILI9341_FillRectangle(65, 245, 45, 27, ILI9341_WHITE);            
        }
}

void print_bluetooth(bool show){    
    if (show) {
            ILI9341_DrawImage(5, 255, 15, 24, (const uint16_t*)bluetooth);
        }
        else {
            ILI9341_FillRectangle(5, 255, 15, 24, ILI9341_WHITE);            
        }
}

void print_gsm(bool show){
    if (show) {
            ILI9341_DrawImage(22, 258, 29, 18, (const uint16_t*)gsm);        }
        else {
            ILI9341_FillRectangle(22, 258, 29, 18, ILI9341_WHITE);            
        }
}

void print_sys_label(void) {
        ILI9341_DrawImage(5, 10, 46, 36, (const uint16_t*)SYS);
}

void print_dia_label(void) {
        ILI9341_DrawImage(5, 133, 45, 35, (const uint16_t*)DIA);
}

void print_SYS(int16_t IN){        
        uint8_t color = GREEN;
        if (IN > YELLOW_LEVEL_SYS) color = YELLOW;
        if (IN > RED_LEVEL_SYS) color = RED;
        print_num_H(IN, 235, 10, color);
}

void print_DIA(int16_t IN){
        uint8_t color = GREEN;
        if (IN > YELLOW_LEVEL_DIA) color = YELLOW;
        if (IN > RED_LEVEL_DIA) color = RED;
        print_num_H(IN, 235, 120, color);
}

void print_battery(void){
        ILI9341_FillRectangle(50, 150, 150, 70, ILI9341_BLACK);
        ILI9341_FillRectangle(52, 152, 146, 66, ILI9341_WHITE);
        ILI9341_FillRectangle(40, 150+35-20, 10, 40, ILI9341_BLACK);
        ILI9341_FillRectangle(40+2, 150+35-20+2, 10-2, 40-4, ILI9341_WHITE);
}

void print_batt_charge(void){
        uint8_t num_of_segments = 6;
        uint16_t adc_1;

        adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
        delay_1ms(100);        
        adc_1 = adc_value[0]*3300/(0xFFF);

        uint8_t buff[5]={0};
        sprintf(buff,"%04d:",adc_1);
//        ILI9341_WriteString(130, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
    
        if (adc_1 > BATT_RANG_MAX) {
            for (int i=1; i < num_of_segments; i++)
            {
               ILI9341_FillRectangle(200-i*29, 154, 25, 62, ILI9341_GREEN);                                    
            }
            return;
        }
        
        if(indicate_charge_toggle) {
                indicate_charge_counter++;
                if (indicate_charge_counter > num_of_segments) {
                    indicate_charge_counter = 1;
                    ILI9341_FillRectangle(52, 152, 146, 66, ILI9341_WHITE);
                }
                for (int i=1; i<indicate_charge_counter; i++){
                        ILI9341_FillRectangle(200-i*29, 154, 25, 62, ILI9341_GREEN);                        
                }                    
                indicate_charge_toggle=0;
        }
        else {
//                ILI9341_FillRectangle(52, 152, 146, 66, ILI9341_WHITE);
                indicate_charge_toggle=1;
        }
}


void print_error(uint8_t K){
        ILI9341_FillRectangle(55, 10, 180, 106, ILI9341_WHITE);
        ILI9341_FillRectangle(55, 120, 180, 106, ILI9341_WHITE);
        ILI9341_FillRectangle(112, 250, 123, 64, ILI9341_WHITE);    
        uint8_t _buff[15]={0};
        if (K==ERROR_CUFF){
                sprintf(_buff,"majetta error");        
                ILI9341_WriteString(1, 30, _buff, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);            
        }
        if (K==ERROR_TIME){
                sprintf(_buff,"time fail");        
                ILI9341_WriteString(1, 30, _buff, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);            
        }
        if (K==ERROR_MEAS){
                sprintf(_buff,"measurement error");        
                ILI9341_WriteString(1, 30, _buff, Font_11x18, ILI9341_BLACK, ILI9341_WHITE);            
        }        
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

void print_time(uint32_t timevar){
        uint8_t buff[100]={0};
    uint32_t thh = 0, tmm = 0, tss = 0;
        
    // compute  hours 
    thh = timevar / 3600;
    // compute minutes 
    tmm = (timevar % 3600) / 60;
    // compute seconds 
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
        sprintf(buff,"%02d:",thh);
        ILI9341_WriteString(130, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
        sprintf(buff,"%02d:",tmm);
        ILI9341_WriteString(160, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
        sprintf(buff,"%02d",tss);
        ILI9341_WriteString(190, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);    
        
        sprintf(buff,"%02d.",cur_day);
        ILI9341_WriteString(130, 260, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
        sprintf(buff,"%02d.",cur_month);
        ILI9341_WriteString(160, 260, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
        sprintf(buff,"%04d",cur_year);
        ILI9341_WriteString(190, 260, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
}


void TFT_print(void){
        uint16_t adc_1=0;
        uint8_t rang_batt=0;
    
        adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
        delay_1ms(100);        
        adc_1=adc_value[0]*3300/(0xFFF);

        uint8_t buff[5]={0};
        sprintf(buff,"%04d:",adc_1);
//        ILI9341_WriteString(130, 230, buff, Font_11x18, ILI9341_RED, ILI9341_WHITE);
    
        if (adc_1<1800) return;

        if (adc_1> BATT_RANG_MAX)                    rang_batt=5;
        else if (adc_1 < BATT_RANG_MAX & adc_1 > BATT_RANG_5) rang_batt=5;
        else if (adc_1 < BATT_RANG_5 & adc_1 > BATT_RANG_4) rang_batt=4;
        else if (adc_1 < BATT_RANG_4 & adc_1 > BATT_RANG_3) rang_batt=3;
        else if (adc_1 < BATT_RANG_3 & adc_1 > BATT_RANG_2) rang_batt=2;
        else if (adc_1 < BATT_RANG_2 & adc_1 > BATT_RANG_1) rang_batt=1;
        else if (adc_1 < BATT_RANG_1)    rang_batt=0;
                
        if (rang_batt_old!=rang_batt)
        {    
            ILI9341_DrawImage(6, 285, 44, 24, (const uint16_t*)bat_clr);            
            for (int i1=0;i1<rang_batt;i1++){
                    ILI9341_DrawImage(42-i1*8, 286, 7, 22, (const uint16_t*)bat_dif);
            }
            rang_batt_old=rang_batt;
        }        
}


void print_num_H(int16_t num, uint16_t X0, uint16_t Y0, uint8_t color){
    double now=0;
    uint16_t now1=0;
    uint8_t max;
    if (num>=100)max=3;
    else if (num>=10) {
        max=2;
        if (color==BLACK) ILI9341_FillRectangle(X0-41*3, Y0, 41, 64, ILI9341_WHITE);
        else                             ILI9341_FillRectangle(X0-60*3, Y0, 60, 106, ILI9341_WHITE);
    }
    else {
        max=1;
        if (color==BLACK) ILI9341_FillRectangle(X0-41*3, Y0, 82, 64, ILI9341_WHITE);
        else                             ILI9341_FillRectangle(X0-60*3, Y0, 120, 106, ILI9341_WHITE);
    }
            
    for (int g=0;g<max;g++){
            now=pow(10,g);
            now1=now;            
            switch ((num/now1)%10)    {    
                case 0:                    
                    if (color==GREEN)                ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_0);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_0);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_0);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_0);
                break;
                case 1:
                     if (color==GREEN)                ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_1);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_1);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_1);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_1);
                break;
                case 2:
                     if (color==GREEN)                ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_2);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_2);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_2);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_2);
                break;
                case 3:
                    if (color==GREEN)                ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_3);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_3);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_3);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_3);
                break;
                case 4:
                    if (color==GREEN)                ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_4);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_4);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_4);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_4);
                break;
                case 5:
                    if (color==GREEN)                ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_5);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_5);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_5);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_5);
                break;
                case 6:
                    if (color==GREEN)             ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_6);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_6);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_6);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_6);
                break;
                case 7:
                    if (color==GREEN)             ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_7);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_7);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_7);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_7);
                break;
                case 8:
                    if (color==GREEN)             ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_8);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_8);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_8);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_8);
                break;
                case 9:
                    if (color==GREEN)             ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)G_9);
                    else if (color==RED)         ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)R_9);
                    else if (color==YELLOW) ILI9341_DrawImage(X0-60-g*60, Y0, 60, 106, (const uint16_t*)Y_9);
                    else if (color==BLACK)     ILI9341_DrawImage(X0-41-g*41, Y0, 41, 64, (const uint16_t*)B_L_9);
                break;
            }
    }
}


