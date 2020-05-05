#include "stm32f0xx.h" 
#include "stdbool.h"

#define 	LCD_Addr		0x78
#define   COMM    		0x80			//0b10000000
#define   DATE    		0xC0			//0b11000000
#define   BURST   		0x40			//0b01000000

const unsigned char init_lcd[]= 																										{
	0xA8,0x3F,										//0xA8 Set Multiplex Ratio
	0xD3,0x00,										//0xD3 Set Display Offset
	0x40,													//0x40 Set Display Start Line
	0xA1,													//Set Segment Re-map 
	0xC8,													//Set COM Output Scan Direction
	0xDA,0x12,										//Set COM Pins Hardware Configuration
	0x81,0xF0,										//0x81  0xXX - Set contrast (default 0x7F)
	0xA4,													//Entire Display ON Ram read
	0xA6,													//0xA6 Set Normal/Inverse Display, 0xA6 - Normal /A7 - Inverse
	0xD5,0x80,										//0xD5 Set Display Clock Divide Ratio/Oscillator Frequency
	0x8D,
	0x14,													//Set Higher column
	0xAF													//Display ON
};
const unsigned char font[]= 																												{// Font 8*5 
 0x00, 0x00, 0x00, 0x00, 0x00 ,   // sp  32
 0x00, 0x00, 0x2f, 0x00, 0x00 ,   // !   33
 0x00, 0x07, 0x00, 0x07, 0x00 ,   // "   34
 0x14, 0x7f, 0x14, 0x7f, 0x14 ,   // #   35
 0x24, 0x2a, 0x7f, 0x2a, 0x12 ,   // $   36
 0xc4, 0xc8, 0x10, 0x26, 0x46 ,   // %   37
 0x36, 0x49, 0x55, 0x22, 0x50 ,   // &   38
 0x00, 0x05, 0x03, 0x00, 0x00 ,   // '   39
 0x00, 0x1c, 0x22, 0x41, 0x00 ,   // (   40
 0x00, 0x41, 0x22, 0x1c, 0x00 ,   // )   41
 0x14, 0x08, 0x3E, 0x08, 0x14 ,   // *   42
 0x08, 0x08, 0x3E, 0x08, 0x08 ,   // +   43
 0x00, 0x00, 0x50, 0x30, 0x00 ,   // ,   44
 0x10, 0x10, 0x10, 0x10, 0x10 ,   // -   45
 0x00, 0x60, 0x60, 0x00, 0x00 ,   // .   46
 0x20, 0x10, 0x08, 0x04, 0x02 ,   // /   47
 0x3E, 0x51, 0x49, 0x45, 0x3E ,   // 0   48
 0x00, 0x42, 0x7F, 0x40, 0x00 ,   // 1   49
 0x42, 0x61, 0x51, 0x49, 0x46 ,   // 2   50
 0x21, 0x41, 0x45, 0x4B, 0x31 ,   // 3   51
 0x18, 0x14, 0x12, 0x7F, 0x10 ,   // 4   52
 0x27, 0x45, 0x45, 0x45, 0x39 ,   // 5   53
 0x3C, 0x4A, 0x49, 0x49, 0x30 ,   // 6   54
 0x01, 0x71, 0x09, 0x05, 0x03 ,   // 7   55
 0x36, 0x49, 0x49, 0x49, 0x36 ,   // 8   56
 0x06, 0x49, 0x49, 0x29, 0x1E ,   // 9   57
 0x00, 0x36, 0x36, 0x00, 0x00 ,   // :   58
 0x00, 0x56, 0x36, 0x00, 0x00 ,   // ;   59
 0x08, 0x14, 0x22, 0x41, 0x00 ,   // <   60
 0x14, 0x14, 0x14, 0x14, 0x14 ,   // =   61
 0x00, 0x41, 0x22, 0x14, 0x08 ,   // >   62
 0x02, 0x01, 0x51, 0x09, 0x06 ,   // ?   63
 0x32, 0x49, 0x59, 0x51, 0x3E ,   // @   64
 0x7E, 0x11, 0x11, 0x11, 0x7E ,   // A   65
 0x7F, 0x49, 0x49, 0x49, 0x36 ,   // B   66
 0x3E, 0x41, 0x41, 0x41, 0x22 ,   // C   67
 0x7F, 0x41, 0x41, 0x22, 0x1C ,   // D   68
 0x7F, 0x49, 0x49, 0x49, 0x41 ,   // E   69
 0x7F, 0x09, 0x09, 0x09, 0x01 ,   // F   70
 0x3E, 0x41, 0x49, 0x49, 0x7A ,   // G   71 
 0x7F, 0x08, 0x08, 0x08, 0x7F ,   // H   72
 0x00, 0x41, 0x7F, 0x41, 0x00 ,   // I   73
 0x20, 0x40, 0x41, 0x3F, 0x01 ,   // J   74
 0x7F, 0x08, 0x14, 0x22, 0x41 ,   // K   75
 0x7F, 0x40, 0x40, 0x40, 0x40 ,   // L   76
 0x7F, 0x02, 0x0C, 0x02, 0x7F ,   // M   77
 0x7F, 0x04, 0x08, 0x10, 0x7F ,   // N   78
 0x3E, 0x41, 0x41, 0x41, 0x3E ,   // O   79
 0x7F, 0x09, 0x09, 0x09, 0x06 ,   // P   80
 0x3E, 0x41, 0x51, 0x21, 0x5E ,   // Q   81
 0x7F, 0x09, 0x19, 0x29, 0x46 ,   // R   82
 0x46, 0x49, 0x49, 0x49, 0x31 ,   // S   83
 0x01, 0x01, 0x7F, 0x01, 0x01 ,   // T   84
 0x3F, 0x40, 0x40, 0x40, 0x3F ,   // U   85
 0x1F, 0x20, 0x40, 0x20, 0x1F ,   // V   86
 0x3F, 0x40, 0x38, 0x40, 0x3F ,   // W   87
 0x63, 0x14, 0x08, 0x14, 0x63 ,   // X   88
 0x07, 0x08, 0x70, 0x08, 0x07 ,   // Y   89
 0x61, 0x51, 0x49, 0x45, 0x43 ,   // Z   90
 0x00, 0x7F, 0x41, 0x41, 0x00 ,   // [   91
 0x55, 0x2A, 0x55, 0x2A, 0x55 ,   // 55  92
 0x00, 0x41, 0x41, 0x7F, 0x00 ,   // ]   93
 0x04, 0x02, 0x01, 0x02, 0x04 ,   // ^   94
 0x40, 0x40, 0x40, 0x40, 0x40 ,   // _   95
 0x00, 0x01, 0x02, 0x04, 0x00 ,   // '   96
 0x20, 0x54, 0x54, 0x54, 0x78 ,   // a   97
 0x7F, 0x48, 0x44, 0x44, 0x38 ,   // b   98
 0x38, 0x44, 0x44, 0x44, 0x20 ,   // c   99
 0x38, 0x44, 0x44, 0x48, 0x7F ,   // d   100
 0x38, 0x54, 0x54, 0x54, 0x18 ,   // e   101
 0x08, 0x7E, 0x09, 0x01, 0x02 ,   // f   102
 0x0C, 0x52, 0x52, 0x52, 0x3E ,   // g   103
 0x7F, 0x08, 0x04, 0x04, 0x78 ,   // h   104
 0x00, 0x44, 0x7D, 0x40, 0x00 ,   // i   105
 0x20, 0x40, 0x44, 0x3D, 0x00 ,   // j   106
 0x7F, 0x10, 0x28, 0x44, 0x00 ,   // k   107
 0x00, 0x41, 0x7F, 0x40, 0x00 ,   // l   108
 0x7C, 0x04, 0x18, 0x04, 0x78 ,   // m   109
 0x7C, 0x08, 0x04, 0x04, 0x78 ,   // n   110
 0x38, 0x44, 0x44, 0x44, 0x38 ,   // o   111
 0x7C, 0x14, 0x14, 0x14, 0x08 ,   // p   112
 0x08, 0x14, 0x14, 0x18, 0x7C ,   // q   113
 0x7C, 0x08, 0x04, 0x04, 0x08 ,   // r   114
 0x48, 0x54, 0x54, 0x54, 0x20 ,   // s   115
 0x04, 0x3F, 0x44, 0x40, 0x20 ,   // t   116
 0x3C, 0x40, 0x40, 0x20, 0x7C ,   // u   117
 0x1C, 0x20, 0x40, 0x20, 0x1C ,   // v   118
 0x3C, 0x40, 0x30, 0x40, 0x3C ,   // w   119
 0x44, 0x28, 0x10, 0x28, 0x44 ,   // x   120
 0x0C, 0x50, 0x50, 0x50, 0x3C ,   // y   121
 0x44, 0x64, 0x54, 0x4C, 0x44 ,   // z   122
  //
 0x00, 0x08, 0x36, 0x41, 0x00 ,   //7B- {
 0x00, 0x00, 0x7f, 0x00, 0x00 ,   //7C- |
 0x00, 0x41, 0x36, 0x08, 0x00 ,   //7D- }
 0x04, 0x02, 0x04, 0x08, 0x04 ,   //7E- ~
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //7F- 
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //80- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //81- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //82- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //83- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //84- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //85- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //86- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //87- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //88- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //89- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //8A- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //8B- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //8C- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //8D- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //8E- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //8F- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //90- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //91- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //92- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   // 93- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //94- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //95- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //96- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //97- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //98- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //99- � 
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //9A- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //9B- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //9C- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //9D- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //9E- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //9F- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A0- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A1- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A2- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A3- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A4- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A5- ?
 0x00, 0x00, 0x36, 0x00, 0x00 ,   //A6- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A7- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A8- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //A9- �
 0x3E, 0x49, 0x49, 0x49, 0x22 ,   //AA- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //AB- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //AC- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //AD- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //AE- �
 0x44, 0x45, 0x7C, 0x45, 0x44 ,   //AF- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //B0- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //B1- �
 0x00, 0x41, 0x7F, 0x41, 0x00 ,   //B2- ?
 0x00, 0x44, 0x7D, 0x40, 0x00 ,   //B3- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //B4- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //B5- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //B6- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //B7- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //B8- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //B9- ?
 0x38, 0x54, 0x44, 0x28, 0x00 ,   //BA- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //BB- �
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //BC- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //BD- ?
 0x00, 0x00, 0x00, 0x00, 0x00 ,   //BE- ?
 0x4A, 0x48, 0x7A, 0x40, 0x40 ,   //BF- ?                             
    
 0x7E, 0x11, 0x11, 0x11, 0x7E , // A        /*192*/
 0x7F, 0x49, 0x49, 0x49, 0x31 , // ?
 0x7F, 0x49, 0x49, 0x49, 0x36 , // B
 0x7F, 0x01, 0x01, 0x01, 0x03 , // ?
 0x60, 0x3E, 0x21, 0x21, 0x7F , // ?
 0x7F, 0x49, 0x49, 0x49, 0x41 , // E
 0x63, 0x14, 0x7F, 0x14, 0x63 , // ?
 0x22, 0x49, 0x49, 0x49, 0x36 , // ?
 0x7F, 0x10, 0x08, 0x04, 0x7F , // ?
 0x7F, 0x10, 0x09, 0x04, 0x7F , // ?
 0x7F, 0x08, 0x14, 0x22, 0x41 , // K
 0x7C, 0x02, 0x01, 0x01, 0x7F , // ?
 0x7F, 0x02, 0x0C, 0x02, 0x7F , // M
 0x7F, 0x08, 0x08, 0x08, 0x7F , // H
 0x3E, 0x41, 0x41, 0x41, 0x3E , // O
 0x7F, 0x01, 0x01, 0x01, 0x7F , // ?
 0x7F, 0x09, 0x09, 0x09, 0x06 , // P
 0x3E, 0x41, 0x41, 0x41, 0x22 , // C
 0x01, 0x01, 0x7F, 0x01, 0x01 , // T
 0x07, 0x48, 0x48, 0x48, 0x3F , // ?
 0x1C, 0x22, 0x7F, 0x22, 0x1C , // ?
 0x63, 0x14, 0x08, 0x14, 0x63 , // X
 0x3F, 0x20, 0x20, 0x3F, 0x60 , // ?
 0x07, 0x08, 0x08, 0x08, 0x7F , // ?
 0x7F, 0x40, 0x7F, 0x40, 0x7F , // ?
 0x3F, 0x20, 0x3F, 0x20, 0x7F , // ?
 0x01, 0x7F, 0x48, 0x48, 0x30 , // ?
 0x7F, 0x48, 0x48, 0x30, 0x7F , // ?
 0x7F, 0x48, 0x48, 0x48, 0x30 , // ?
 0x22, 0x49, 0x49, 0x49, 0x3E , // ?
 0x7F, 0x08, 0x3E, 0x41, 0x3E , // ?
 0x46, 0x29, 0x19, 0x09, 0x7F , // ?
 0x20, 0x54, 0x54, 0x54, 0x78 , // a
 0x78, 0x54, 0x54, 0x54, 0x20 , // b
 0x7C, 0x54, 0x54, 0x54, 0x28 , // ?
 0x7C, 0x04, 0x04, 0x04, 0x00 , // ?
 0x60, 0x38, 0x24, 0x38, 0x60 , // ?
 0x38, 0x54, 0x54, 0x54, 0x18 , // e
 0x6C, 0x10, 0x7C, 0x10, 0x6C , // ?
 0x28, 0x44, 0x54, 0x54, 0x28 , // ?
 0x3C, 0x40, 0x40, 0x20, 0x7C , // ?
 0x3C, 0x40, 0x42, 0x20, 0x7C , // ?
 0x7C, 0x10, 0x10, 0x28, 0x44 , // ?
 0x60, 0x10, 0x08, 0x04, 0x7C , // ?
 0x7C, 0x08, 0x10, 0x08, 0x7C , // ?
 0x7C, 0x10, 0x10, 0x10, 0x7C , // ?
 0x38, 0x44, 0x44, 0x44, 0x38 , // o
 0x7C, 0x04, 0x04, 0x04, 0x7C , // ?
 0x7C, 0x14, 0x14, 0x14, 0x08 , // p
 0x38, 0x44, 0x44, 0x44, 0x20 , // c
 0x04, 0x04, 0x7C, 0x04, 0x04 , // ?
 0x0C, 0x50, 0x50, 0x50, 0x3C , // y
 0x18, 0x24, 0x7C, 0x24, 0x18 , // ?
 0x44, 0x28, 0x10, 0x28, 0x44 , // x
 0x3C, 0x20, 0x20, 0x3C, 0x60 , // ?
 0x0C, 0x10, 0x10, 0x10, 0x7C , // ?
 0x7C, 0x40, 0x7C, 0x40, 0x7C , // ?
 0x3C, 0x20, 0x3C, 0x20, 0x7C , // ?
 0x04, 0x7C, 0x48, 0x48, 0x30 , // ?
 0x7C, 0x48, 0x30, 0x00, 0x7C , // ?
 0x7C, 0x48, 0x48, 0x48, 0x30 , // ?
 0x28, 0x44, 0x54, 0x54, 0x38 , // ?
 0x7C, 0x10, 0x38, 0x44, 0x38 , // ?
 0x58, 0x24, 0x24, 0x24, 0x7C  //  ?
 };





//------------------------------LCD ------------------------------

static bool WaitISRBitSet(uint32_t Bit)																							{
  uint32_t time_out=10000; 
  do
  {
    if((I2C1->ISR &( I2C_ISR_NACKF | I2C_ISR_ARLO | I2C_ISR_BERR)) || (time_out--==0))
    {
     I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_ADDRCF;
		 I2C1->ISR |=1;//TXE 
			return false;
    }
  }
  while( !(I2C1->ISR & Bit) );
  return true;
}
void I2C_write(unsigned char byte_send)																			    		{//WRITE
	
		while(!(I2C1->ISR & WaitISRBitSet(I2C_ISR_TXE))){}; 
		I2C1->TXDR = byte_send;
		while(I2C1->ISR & I2C_ISR_TC){};
}
 
void I2C_Start(void)    																														{//Start
	  while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
		I2C1->CR2 &=(~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN)  & (~I2C_CR2_NACK);
	  I2C1->CR2 |= (LCD_Addr << I2C_CR2_SADD_Pos) | (0xFF<< I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
		while((I2C1->ISR & I2C_ISR_TC)){}; 
	 }

void I2C_Stop(void) 																																{//Stop
	 while(I2C1->ISR & !WaitISRBitSet(I2C_ISR_TXE)){}; // TX buf Empty
	 I2C1->CR2 |= I2C_CR2_STOP;
	 while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	 I2C1->ICR |= I2C_ICR_STOPCF;	
	 }
void CMD(unsigned char byte) 																												{
 I2C_write(COMM);
 I2C_write(byte);
 }
 
void LCD_Init(void) 																																{// INIT LCD T191
 I2C_Start();
CMD(0xAE);
for (unsigned char i=0;i<=sizeof(init_lcd);i++)
	{CMD(init_lcd[i]);}
I2C_Stop();
}

void LCD_Gotoxy ( unsigned char x, unsigned char y )														    {
       I2C_Start();
       
			 if (x>0xf) {CMD(0x10 |(x>>4));}
			 else {CMD(0x10);}
			 CMD(0x00 | (x&0xf));
			 CMD(0xb0 | y);  																							//Y - 0...7
       I2C_Stop();
    }
void LCD_mode(unsigned char mode)																										{ 
      
       I2C_Start();
       if(mode > 0)
       {CMD(0xA7);}
      else
       {CMD(0xA6);}
       I2C_Stop();
      }
void LCD_Char(char flash_o, unsigned char mode)															        {// Print Symbol
      	 unsigned char i,ch; 
         I2C_Start();
         I2C_write(BURST);
          for(i=0; i<5; i++)
      	  {
      	    ch = font[(flash_o - 32)*5+i];
      	     if(mode) { I2C_write(~ch); }
      			 else  	 { I2C_write(ch);}
      		 if(i == 4)
      			  {
      			    if(mode) { I2C_write(0xff); }
      				  else   {	 I2C_write(0x00); }
      			  }
      	   }
         I2C_Stop();
          }
void LCD_PrintStr(char *s, unsigned char mode)																	    {// Print String
          while (*s)
        {
          LCD_Char(*s, mode);
          s++;
        }
      }
void LCD_Clear(void)      																													{// Clear LCD
    	 unsigned char k, kk;
	for(kk=0;kk<8;kk++) 
	{
		I2C_Start();
		CMD(0x20);
		CMD(0x00);
		I2C_write(BURST);
		for(k=0;k<128;k++) I2C_write(0x00);	//LSB ??????, MSB ?????
		I2C_Stop();
	}
}       


void LCD_ClearStr(unsigned char y, unsigned char qnt )															{
unsigned char q,k;
	LCD_Gotoxy ( 0, y );
for (q=y;q<(qnt+y);q++){
		I2C_Start();
		CMD(0x20);
		CMD(0x00);
		CMD(0x10);
	  CMD(0xb0|q);
		I2C_write(BURST);
		for(k=0;k<128;k++) I2C_write(0x00);
		I2C_Stop();
}	
}
void LCD_PrintDec(long val,unsigned char mode) 																			{// Print Dec
	
	char i=1,d=0;
	unsigned char text[10];
//-------Added for support value less than '0' ----------------
	uint8_t minus_flag=0;
	uint32_t value;
	if (val>=0) {value=val;}
		else {value=val*(-1);minus_flag=1;}
//-------------------------------------------------	
	do 
  { 
    if (value >=10)  {
				d = value % 10; 																				
				text[i] = d + '0'; 																			
				value /= 10; 																						
			}
		else 
			{	text[i] = value + '0';
				value=0;
			}
 		i++;
  }
	while(value); 
//-------Added for support value less than '0'----------------
	if(minus_flag) {text[i]='-';}
	else {i--;}		
	
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}
			
void LCD_PrintHex(long val,unsigned char mode) 																			{// Print Hex
	
	char i=1,d=0;
	unsigned char text[10];
//-------------Added for support valur <0
	uint8_t minus_flag=0;
	uint32_t value;
	if (val>=0) {value=val;}
		else {value=val*(-1);minus_flag=1;}
//---------------------------------------		
	do 
  { 
    if (value >=0x10)  {
				d = value % 0x10; 																				
				if(d<0x0A) text[i] = d + '0'; 																			
				else 			 text[i] = d + 0x37;
				value /= 0x10; 																						
			}
		else 
			{	
				if(value < 0x0A)	text[i] = value + '0';			//0..9
				else 							text[i] = value + 0x37;			//A...F
				value=0;
			}
 		i++;
  }
	while(value); 
//-------Added for support valur <0----------------
	if(minus_flag) {text[i]='-';}
	else {i--;}		
//---------------------------------------		
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}

void LCD_PrintBin(unsigned char value,unsigned char mode) 													{// Print Bin
	
	unsigned char i=1,d=0;
	unsigned char text[8];
	do 
  { 
    if (value >=2)  {
				d = value % 2; 																				
				text[i] = d + '0'; 																			
				value /= 0x2; 																						
			}
		else 
			{	
				text[i] = value + '0';			//0..9
				value=0;
			}
 		i++;
  }
	while(i<9); 
	i--;			
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}	
	
void Flip_mode(unsigned char flip)																									{
I2C_Start();
if (flip>0) {CMD(0xA0);CMD(0xC0);}
else {CMD(0xA1);CMD(0xC8);}
I2C_Stop();
}
void slide_right(unsigned char start_page, unsigned char end_page, unsigned char interval)														{
I2C_Start();
CMD(0x26); CMD(0x00);
CMD(start_page);
CMD(interval);
CMD(end_page);
CMD(0x00);CMD(0xff);	
I2C_Stop();	
}
void slide_left(unsigned char start_page, unsigned char end_page, unsigned char interval)															{
I2C_Start();
CMD(0x27); CMD(0x00);
CMD(start_page);
CMD(interval);
CMD(end_page);
CMD(0x00);CMD(0xff);	
I2C_Stop();	
}
void stop_slide(void)																																{
I2C_Start();
CMD(0x2E); 
I2C_Stop();	
}
void start_slide(void)																															{
I2C_Start();
CMD(0x2F); 
I2C_Stop();	
}

void scroll_vright(unsigned char start_page, unsigned char end_page, unsigned char interval,unsigned char v_offset)		{
I2C_Start();
CMD(0x29); CMD(0x00);
CMD(start_page & 0x07);
CMD(interval & 0x07);
CMD(end_page & 0x07);
CMD(v_offset & 0x3f);
	
I2C_Stop();	
}
void scroll_vleft(unsigned char start_page, unsigned char end_page, unsigned char interval,unsigned char v_offset)		{
I2C_Start();
CMD(0x2A); CMD(0x00);
CMD(start_page & 0x07);
CMD(interval & 0x07);
CMD(end_page & 0x07);
CMD(v_offset & 0x3f);	
I2C_Stop();	
}
void scroll_area(unsigned char start_page, unsigned char end_page)									{
I2C_Start();
CMD(0xA3); 
CMD(start_page & 0x3f);
CMD(end_page & 0x7f);
I2C_Stop();	
}
