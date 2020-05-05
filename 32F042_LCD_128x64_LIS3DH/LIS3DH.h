#ifndef LIS3DH_H
#define LIS3DH_H

#include "stm32f0xx.h"

#define LIS3DH_ID								0x32   								//i2c addr -  30/32 depend of SDO pin If "1" - 0x32, if "0" - 0x30

#define LIS3DH_I2C_MODE 				0x00
#define LIS3DH_SPI_MODE 				0x01
#define LIS_ON					 				0x01
#define LIS_OFF									0x00

//===========================Registers===============
#define LIS3DH_STATUS_REG_AUX 	0x07
#define LIS3DH_OUT_ADC1_L 			0x08
#define LIS3DH_OUT_ADC1_H 			0x09
#define LIS3DH_OUT_ADC2_L 			0x0A
#define LIS3DH_OUT_ADC2_H 			0x0B
#define LIS3DH_OUT_ADC3_L 			0x0C
#define LIS3DH_OUT_ADC3_H 			0x0D
#define LIS3DH_INT_COUNTER_REG 	0x0E
#define LIS3DH_WHO_AM_I 				0x0F
#define LIS3DH_TEMP_CFG_REG 		0x1F
#define LIS3DH_CTRL_REG1 				0x20
#define LIS3DH_CTRL_REG2 				0x21
#define LIS3DH_CTRL_REG3 				0x22
#define LIS3DH_CTRL_REG4 				0x23
#define LIS3DH_CTRL_REG5 				0x24
#define LIS3DH_CTRL_REG6 				0x25
#define LIS3DH_REFERENCE 				0x26
#define LIS3DH_STATUS_REG 			0x27
#define LIS3DH_OUT_X_L 					0x28
#define LIS3DH_OUT_X_H 					0x29
#define LIS3DH_OUT_Y_L 					0x2A
#define LIS3DH_OUT_Y_H 					0x2B
#define LIS3DH_OUT_Z_L 					0x2C
#define LIS3DH_OUT_Z_H 					0x2D
#define LIS3DH_FIFO_CTRL_REG 		0x2E
#define LIS3DH_FIFO_SRC_REG 		0x2F
#define LIS3DH_INT1_CFG 				0x30

//===============DATA BIT MASK description===============
#define TEMP_CFG_REG_TEMP_BIT 	0x40
#define TEMP_CFG_REG_ADC_BIT 		0x80
#define	CTRL_REG4_BDU						0x80


struct lis3dh {											//Structure with communication settings and data
	int 					ax;																	
	int 					ay;		
	int 					az;
	int 					adc1; 
	int 					adc2; 
	int 					adc3;
	unsigned char mode;								// LIS3DH_I2C_MODE / LIS3DH_SPI_MODE
	int	temp;
};



//=====================I2C Section============================
void 						writeReg8(unsigned char reg, unsigned char value);											// Writes data into a register.
unsigned char 	readReg8(unsigned char reg);																						//Reads the value of a register.
void 						readMulti(unsigned char reg, unsigned char * dst, unsigned char count);	//Reads multi values of a multi registers.
//=====================END of I2C=============================
//=======================SPI Section=========================
#define CS_LO()									GPIOA->BRR 	|=1<<4					//PA4 - CS PIN  
#define CS_HI()									GPIOA->BSRR |=1<<4
#define SPI1_DR_8bit 						*((__IO uint8_t *)&SPI1->DR)		// Limit for spi bus 8 bit
#define SINGLE_WRITE					 	0x00
#define BURST_WRITE							0x40
#define SINGLE_READ					 		0x80
#define BURST_READ							0xC0

unsigned char 	SPI_Write(unsigned char reg, unsigned char value);
unsigned char 	SPI_Read(unsigned char reg);
unsigned char	 	SPI_Read_Multi(unsigned char reg, unsigned char * dst, unsigned char count);
//=====================END of SPI=============================


void 						init_lis3dh(struct lis3dh* data, unsigned char lis_mode);		  					//Init. interface
void 						readLisData(struct lis3dh* data);
void 						readLisADC(struct lis3dh* data);
void 						readLisTemp(struct lis3dh* data);
void 						Lis_ADC_CMD(struct lis3dh* data,unsigned char off_on);
void 						Lis_TEMP_CMD(struct lis3dh* data,unsigned char off_on);

#endif	/*LIS3DH_H*/
