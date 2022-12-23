/* vim: set ai et ts=4 sw=4: */

#include "ili9341.h"
#include "gd32f30x_it.h"

#define set_RES          	gpio_bit_set(ILI9341_RES_GPIO_Port,ILI9341_RES_Pin);
#define reset_RES         gpio_bit_reset(ILI9341_RES_GPIO_Port,ILI9341_RES_Pin);
#define set_CS          	gpio_bit_set(ILI9341_CS_GPIO_Port,ILI9341_CS_Pin);
#define reset_CS          gpio_bit_reset(ILI9341_CS_GPIO_Port,ILI9341_CS_Pin);
#define set_DC          	gpio_bit_set(ILI9341_DC_GPIO_Port,ILI9341_DC_Pin);
#define reset_DC          gpio_bit_reset(ILI9341_DC_GPIO_Port,ILI9341_DC_Pin);
//#define set_LED          	gpio_bit_set(ILI9341_LED_GPIO_Port,ILI9341_LED_Pin);
//#define reset_LED         gpio_bit_reset(ILI9341_LED_GPIO_Port,ILI9341_LED_Pin);

#define SPI_DELAY         1

void SPI_Transmit(uint32_t SPI_, uint8_t *pData, uint16_t Size, int Timeout){
		for (int j=0;j<Size;j++){
				spi_i2s_data_transmit(SPI_, pData[j]);
				my_delay(Timeout);
		}
}

void ILI9341_Select() {
		reset_CS
}

void ILI9341_Unselect() {
    set_CS 
}

static void ILI9341_Reset() {
    reset_RES
    my_delay(5);
    set_RES
}

void ILI9341_WriteCommand(uint8_t cmd) {
		uint8_t data[]={cmd,2,3};
    reset_DC		
    SPI_Transmit(ILI9341_SPI_PORT, &cmd, sizeof(cmd), SPI_DELAY);
}

void ILI9341_WriteData(uint8_t* buff, int buff_size) {
    set_DC

    // split data in small chunks because HAL can't send more then 64K at once
    while(buff_size > 0) {
        uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
        SPI_Transmit(ILI9341_SPI_PORT, buff, chunk_size, SPI_DELAY);
        buff += chunk_size;
        buff_size -= chunk_size;
    }
}

static void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // column address set
    ILI9341_WriteCommand(0x2A); // CASET
    {
        uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
        ILI9341_WriteData(data, sizeof(data));
    }

    // row address set
    ILI9341_WriteCommand(0x2B); // RASET
    {
        uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
        ILI9341_WriteData(data, sizeof(data));
    }

    // write to RAM
    ILI9341_WriteCommand(0x2C); // RAMWR
}

void ILI9341_Init() {	
		//set_LED
		spi_rcu_config();
		spi_gpio_config();	
    spi_config();
		spi_enable(ILI9341_SPI_PORT);

		ILI9341_Unselect();
    ILI9341_Select();
    ILI9341_Reset();

    // command list is based on https://github.com/martnak/STM32-ILI9341

    // SOFTWARE RESET
    ILI9341_WriteCommand(0x01);
    my_delay(300000);
        
    // POWER CONTROL A
    ILI9341_WriteCommand(0xCB);
    {
        uint8_t data[] = { 0x39, 0x2C, 0x00, 0x34, 0x02 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POWER CONTROL B
    ILI9341_WriteCommand(0xCF);
    {
        uint8_t data[] = { 0x00, 0xC1, 0x30 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // DRIVER TIMING CONTROL A
    ILI9341_WriteCommand(0xE8);
    {
        uint8_t data[] = { 0x85, 0x00, 0x78 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // DRIVER TIMING CONTROL B
    ILI9341_WriteCommand(0xEA);
    {
        uint8_t data[] = { 0x00, 0x00 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POWER ON SEQUENCE CONTROL
    ILI9341_WriteCommand(0xED);
    {
        uint8_t data[] = { 0x64, 0x03, 0x12, 0x81 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // PUMP RATIO CONTROL
    ILI9341_WriteCommand(0xF7);
    {
        uint8_t data[] = { 0x20 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POWER CONTROL,VRH[5:0]
    ILI9341_WriteCommand(0xC0);
    {
        uint8_t data[] = { 0x23 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POWER CONTROL,SAP[2:0];BT[3:0]
    ILI9341_WriteCommand(0xC1);
    {
        uint8_t data[] = { 0x10 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // VCM CONTROL
    ILI9341_WriteCommand(0xC5);
    {
        uint8_t data[] = { 0x3E, 0x28 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // VCM CONTROL 2
    ILI9341_WriteCommand(0xC7);
    {
        uint8_t data[] = { 0x86 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // MEMORY ACCESS CONTROL
    ILI9341_WriteCommand(0x36);
    {
        uint8_t data[] = { 0x48 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // PIXEL FORMAT
    ILI9341_WriteCommand(0x3A);
    {
        uint8_t data[] = { 0x55 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // FRAME RATIO CONTROL, STANDARD RGB COLOR
    ILI9341_WriteCommand(0xB1);
    {
        uint8_t data[] = { 0x00, 0x18 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // DISPLAY FUNCTION CONTROL
    ILI9341_WriteCommand(0xB6);
    {
        uint8_t data[] = { 0x08, 0x82, 0x27 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // 3GAMMA FUNCTION DISABLE
    ILI9341_WriteCommand(0xF2);
    {
        uint8_t data[] = { 0x00 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // GAMMA CURVE SELECTED
    ILI9341_WriteCommand(0x26);
    {
        uint8_t data[] = { 0x01 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // POSITIVE GAMMA CORRECTION
    ILI9341_WriteCommand(0xE0);
    {
        uint8_t data[] = { 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                           0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 };
        ILI9341_WriteData(data, sizeof(data));
    }

    // NEGATIVE GAMMA CORRECTION
    ILI9341_WriteCommand(0xE1);
    {
        uint8_t data[] = { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                           0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F };
        ILI9341_WriteData(data, sizeof(data));
    }

    // EXIT SLEEP
    ILI9341_WriteCommand(0x11);
    my_delay(1);

    // TURN ON DISPLAY
    ILI9341_WriteCommand(0x29);

    // MADCTL
    ILI9341_WriteCommand(0x36);
    {
        uint8_t data[] = { ILI9341_ROTATION };
        ILI9341_WriteData(data, sizeof(data));
    }

    ILI9341_Unselect();
}

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= ILI9341_WIDTH) || (y >= ILI9341_HEIGHT))
        return;

    ILI9341_Select();

    ILI9341_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ILI9341_WriteData(data, sizeof(data));

    ILI9341_Unselect();
}

static void ILI9341_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    ILI9341_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                ILI9341_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                ILI9341_WriteData(data, sizeof(data));
            }
        }
    }
}

void ILI9341_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    ILI9341_Select();

    while(*str) {
        if(x + font.width >= ILI9341_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= ILI9341_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        ILI9341_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }
    ILI9341_Unselect();
}

void ILI9341_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= ILI9341_WIDTH) || (y >= ILI9341_HEIGHT)) return;
    if((x + w - 1) >= ILI9341_WIDTH) w = ILI9341_WIDTH - x;
    if((y + h - 1) >= ILI9341_HEIGHT) h = ILI9341_HEIGHT - y;

    ILI9341_Select();
    ILI9341_SetAddressWindow(x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    set_DC
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            SPI_Transmit(ILI9341_SPI_PORT, data, sizeof(data), SPI_DELAY);
        }
    }
    ILI9341_Unselect();
}

void ILI9341_FillScreen(uint16_t color) {
    ILI9341_FillRectangle(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, color);
}

void ILI9341_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= ILI9341_WIDTH) || (y >= ILI9341_HEIGHT)) return;
    if((x + w - 1) >= ILI9341_WIDTH) return;
    if((y + h - 1) >= ILI9341_HEIGHT) return;

    ILI9341_Select();
    ILI9341_SetAddressWindow(x, y, x+w-1, y+h-1);
		//SPI_Transmit(ILI9341_SPI_PORT, (uint8_t*)data, 0xFA, 20);
    ILI9341_WriteData((uint8_t*)data, w*h*2);
    ILI9341_Unselect();
}

void ILI9341_InvertColors(bool invert) {
    ILI9341_Select();
    ILI9341_WriteCommand(invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
    ILI9341_Unselect();
}

void spi_rcu_config(void)
{
		rcu_periph_clock_enable(ILI9341_CLK_RCU);
		rcu_periph_clock_enable(ILI9341_MISO_RCU);
		rcu_periph_clock_enable(ILI9341_MOSI_RCU);
		rcu_periph_clock_enable(ILI9341_RES_RCU);
		rcu_periph_clock_enable(ILI9341_CS_RCU);
		rcu_periph_clock_enable(ILI9341_DC_RCU);
//		rcu_periph_clock_enable(ILI9341_LED_RCU);
    rcu_periph_clock_enable(ILI9341_RCU_SPI);
    rcu_periph_clock_enable(RCU_AF);
}

void spi_gpio_config(void)
{
		gpio_init(ILI9341_CLK_GPIO_Port, 	GPIO_MODE_AF_PP, 				GPIO_OSPEED_50MHZ, ILI9341_CLK_Pin);
		gpio_init(ILI9341_MOSI_GPIO_Port, GPIO_MODE_AF_PP, 				GPIO_OSPEED_50MHZ, ILI9341_MOSI_Pin);	
    gpio_init(ILI9341_MISO_GPIO_Port, GPIO_MODE_IN_FLOATING, 	GPIO_OSPEED_50MHZ, ILI9341_MISO_Pin);	  
		gpio_init(ILI9341_RES_GPIO_Port, 	GPIO_MODE_OUT_PP, 			GPIO_OSPEED_50MHZ, ILI9341_RES_Pin);
		gpio_init(ILI9341_CS_GPIO_Port, 	GPIO_MODE_OUT_PP, 			GPIO_OSPEED_50MHZ, ILI9341_CS_Pin);
		gpio_init(ILI9341_DC_GPIO_Port, 	GPIO_MODE_OUT_PP, 			GPIO_OSPEED_50MHZ, ILI9341_DC_Pin);
//		gpio_init(ILI9341_LED_GPIO_Port, 	GPIO_MODE_OUT_PP, 			GPIO_OSPEED_50MHZ, ILI9341_LED_Pin);	
}

void spi_config(void)
{
    spi_parameter_struct spi_init_struct;

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;

    spi_init_struct.nss                  = SPI_NSS_SOFT;
		spi_init_struct.prescale             = SPI_PSC_8;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(ILI9341_SPI_PORT, &spi_init_struct);
}

void ILI9341_my_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color){
		uint16_t arg_m1=y1;
		uint16_t arg=0;	

		ILI9341_DrawPixel(x1,y1, color);
		for (uint16_t j=x1+1;j<=x2;j++){				
					arg=(((j-x1)*(y2-y1)/(x2-x1))+y1);
					ILI9341_DrawPixel(j,arg, color);
					for(int k=arg_m1;k<=arg;k++){
							ILI9341_DrawPixel(j,k, color);
					}
		}	
}

