/* vim: set ai et ts=4 sw=4: */

#include "ili9341_touch.h"
#include "gd32f30x_it.h"
#include "ili9341.h"


#define set_t_CS              gpio_bit_set(ILI9341_TOUCH_CS_GPIO_Port,ILI9341_TOUCH_CS_Pin);
#define reset_t_CS          gpio_bit_reset(ILI9341_TOUCH_CS_GPIO_Port,ILI9341_TOUCH_CS_Pin);

#define READ_X 0xD0
#define READ_Y 0x90

int delay_time=200;

void SPI_Receive(uint32_t SPI_, uint16_t *iData, uint16_t Size){
        for (int j=0;j<Size;j++){
            iData[j]=spi_i2s_data_receive(SPI_);
            my_delay(100);
    }
}

void SPI_TransmitReceive(uint32_t SPI_, uint16_t *oData, uint16_t *iData, uint16_t Size, int Timeout){
//    spi_i2s_data_transmit(SPI_, oData[0]);
//    my_delay(Timeout);
//    spi_i2s_data_transmit(SPI_, oData[1]);
//    my_delay(Timeout);
    for (int j=0;j<Size;j++){
//            
//            my_delay(Timeout);
            spi_i2s_data_transmit(SPI_, oData[j]);
            my_delay(10);
            iData[j]=spi_i2s_data_receive(SPI_);
            //my_delay(Timeout);
//            spi_i2s_data_transmit(SPI_, oData[j]);
    }
}

void ILI9341_Touch_init(){
        t_spi_rcu_config();
        t_spi_gpio_config();    
    t_spi_config();
        spi_enable(ILI9341_TOUCH_SPI_PORT);
}

static void ILI9341_TouchSelect() {
    reset_t_CS
}

void ILI9341_TouchUnselect() {
    set_t_CS
}

bool ILI9341_TouchPressed() {
        return gpio_input_bit_get(ILI9341_TOUCH_IRQ_GPIO_Port, ILI9341_TOUCH_IRQ_Pin)==0;
 //   return HAL_GPIO_ReadPin(ILI9341_TOUCH_IRQ_GPIO_Port, ILI9341_TOUCH_IRQ_Pin) == GPIO_PIN_RESET;
}

bool ILI9341_TouchGetCoordinates(uint16_t *x, uint16_t *y) {
        char buffer[12];
    static const uint8_t cmd_read_x[1] = { READ_X };
    static const uint8_t cmd_read_y[1] = { READ_Y };
    static const uint8_t zeroes_tx[2] = { 0x00, 0x00 };

    ILI9341_TouchSelect();

    uint32_t avg_x = 0;
    uint32_t avg_y = 0;
    uint8_t nsamples = 0;
    for(uint8_t i = 0; i < 16; i++) {
        if(!ILI9341_TouchPressed())
            break;

        nsamples++;
                
                //gpio_bit_set(GPIOB, GPIO_PIN_0);                
                
                
            /*
                send_my_spi1(0x90);
                send_my_spi1(0x0);
                send_my_spi1(0x0);
                send_my_spi1(0xD0);
                send_my_spi1(0x0);
                send_my_spi1(0x0);
                */
                
        SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), delay_time);
        uint8_t y_raw[2]={0,0};        
                //my_delay(delay_time);
                
        
                //y_raw[0]=spi_i2s_data_receive(ILI9341_TOUCH_SPI_PORT);
                //SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, 1, delay_time);

                    
                
                y_raw[0]=spi_i2s_data_receive(ILI9341_TOUCH_SPI_PORT);
                SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, 1, delay_time);
                y_raw[1]=spi_i2s_data_receive(ILI9341_TOUCH_SPI_PORT);
                SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, 1, delay_time);
                    
                //SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, 1, delay_time);
                
                
                
                
                
        //SPI_TransmitReceive(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), 100);
                //avg_x=spi_i2s_data_receive(ILI9341_TOUCH_SPI_PORT);
                

                
       SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), delay_time);                
        uint8_t x_raw[2]={0,0};
                
                //my_delay(delay_time);
                //SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, 1, delay_time);
                //x_raw[0]=spi_i2s_data_receive(ILI9341_TOUCH_SPI_PORT);
                //SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, 1, delay_time);    

                
                
                x_raw[0]=spi_i2s_data_receive(ILI9341_TOUCH_SPI_PORT);
                SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, 1, delay_time);
                x_raw[1]=spi_i2s_data_receive(ILI9341_TOUCH_SPI_PORT);
                SPI_Transmit(ILI9341_TOUCH_SPI_PORT, (uint8_t*)zeroes_tx, 1, delay_time);

                
                
        avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
                 

    }

    ILI9341_TouchUnselect();

    if(nsamples < 16)
        return false;

    uint32_t raw_x = (avg_x / 16);
    if(raw_x < ILI9341_TOUCH_MIN_RAW_X) raw_x = ILI9341_TOUCH_MIN_RAW_X;
    if(raw_x > ILI9341_TOUCH_MAX_RAW_X) raw_x = ILI9341_TOUCH_MAX_RAW_X;

    uint32_t raw_y = (avg_y / 16);
    if(raw_y < ILI9341_TOUCH_MIN_RAW_X) raw_y = ILI9341_TOUCH_MIN_RAW_Y;
    if(raw_y > ILI9341_TOUCH_MAX_RAW_Y) raw_y = ILI9341_TOUCH_MAX_RAW_Y;

    // Uncomment this line to calibrate touchscreen:
    // UART_Printf("raw_x = %d, raw_y = %d\r\n", x, y);

    *x = (raw_x - ILI9341_TOUCH_MIN_RAW_X) * ILI9341_TOUCH_SCALE_X / (ILI9341_TOUCH_MAX_RAW_X - ILI9341_TOUCH_MIN_RAW_X);
    *y = (raw_y - ILI9341_TOUCH_MIN_RAW_Y) * ILI9341_TOUCH_SCALE_Y / (ILI9341_TOUCH_MAX_RAW_Y - ILI9341_TOUCH_MIN_RAW_Y);        

    return true;
}

void t_spi_rcu_config(void)
{
        rcu_periph_clock_enable(ILI9341_TOUCH_CLK_RCU);
        rcu_periph_clock_enable(ILI9341_TOUCH_MISO_RCU);
        rcu_periph_clock_enable(ILI9341_TOUCH_MOSI_RCU);
        rcu_periph_clock_enable(ILI9341_TOUCH_IRQ_RCU);
        rcu_periph_clock_enable(ILI9341_TOUCH_CS_RCU);
    rcu_periph_clock_enable(ILI9341_TOUCH_RCU_SPI);
    rcu_periph_clock_enable(RCU_AF);
}

void t_spi_gpio_config(void)
{
        gpio_init(ILI9341_TOUCH_CLK_GPIO_Port,     GPIO_MODE_AF_PP,             GPIO_OSPEED_50MHZ, ILI9341_TOUCH_CLK_Pin);//GPIO_MODE_AF_PP,                 GPIO_OSPEED_50MHZ, ILI9341_TOUCH_CLK_Pin);
        gpio_init(ILI9341_TOUCH_MOSI_GPIO_Port, GPIO_MODE_AF_PP,             GPIO_OSPEED_50MHZ, ILI9341_TOUCH_MOSI_Pin); //GPIO_MODE_AF_PP,                 GPIO_OSPEED_50MHZ, ILI9341_TOUCH_MOSI_Pin);    
    gpio_init(ILI9341_TOUCH_MISO_GPIO_Port, GPIO_MODE_IN_FLOATING,     GPIO_OSPEED_50MHZ, ILI9341_TOUCH_MISO_Pin);      
        gpio_init(ILI9341_TOUCH_IRQ_GPIO_Port,     GPIO_MODE_IN_FLOATING,     GPIO_OSPEED_50MHZ, ILI9341_TOUCH_IRQ_Pin);
        gpio_init(ILI9341_TOUCH_CS_GPIO_Port,     GPIO_MODE_OUT_PP,             GPIO_OSPEED_50MHZ, ILI9341_TOUCH_CS_Pin);
}

void t_spi_config(void)
{
    spi_parameter_struct spi_init_struct;

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;

    spi_init_struct.nss                  = SPI_NSS_SOFT;
        spi_init_struct.prescale             = SPI_PSC_128;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(ILI9341_TOUCH_SPI_PORT, &spi_init_struct);
}

void send_my_spi1(uint8_t data){
        for (int k=0;k<8;k++){
                gpio_bit_reset(ILI9341_TOUCH_CLK_GPIO_Port,ILI9341_TOUCH_CLK_Pin);            
                if (((data>>(7-k))&1)==1) gpio_bit_set(ILI9341_TOUCH_MOSI_GPIO_Port,ILI9341_TOUCH_MOSI_Pin);
                else gpio_bit_reset(ILI9341_TOUCH_MOSI_GPIO_Port,ILI9341_TOUCH_MOSI_Pin);                    
                my_delay(3);
                gpio_bit_set(ILI9341_TOUCH_CLK_GPIO_Port,ILI9341_TOUCH_CLK_Pin);
                my_delay(3);
        }
        //gpio_bit_reset(ILI9341_TOUCH_MOSI_GPIO_Port,ILI9341_TOUCH_MOSI_Pin);
        my_delay(10);
}
