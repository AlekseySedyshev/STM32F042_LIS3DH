#include "stm32f0xx.h"
#include "LIS3DH.h"

//================I2C-Functions============================

void 						writeReg8(unsigned char reg, unsigned char value)													{//Write a 8-bit register
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (LIS3DH_ID	<< I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){}; 	 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = value;	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
}
unsigned char 	readReg8(unsigned char reg)																								{//Read an 8-bit register
	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (LIS3DH_ID	 << I2C_CR2_SADD_Pos) | (0x80 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg;	while((I2C1->ISR & I2C_ISR_TC)){};
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;	
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (LIS3DH_ID	<< I2C_CR2_SADD_Pos) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while(!(I2C1->ISR & I2C_ISR_RXNE)){};		
	unsigned char value=I2C1->RXDR;
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
	return value; 		
}


void 						readMulti(unsigned char reg, unsigned char * dst, unsigned char count)		{// readMulti
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD) & (~I2C_CR2_RD_WRN); 	
 	I2C1->CR2 |= (LIS3DH_ID	 << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){}; I2C1->TXDR = reg | 0x80;	while((I2C1->ISR & I2C_ISR_TC)){}; 	
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY	
	I2C1->CR2 &= ~(I2C_CR2_NBYTES)  & (~I2C_CR2_SADD);	
	I2C1->CR2 |= I2C_CR2_RD_WRN;																				// Direction - data in
	I2C1->CR2 |= (LIS3DH_ID	<< I2C_CR2_SADD_Pos) | (count << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	while((I2C1->ISR & I2C_ISR_TC)){}; 
	while (count-- > 0) {
		
		while(!(I2C1->ISR &I2C_ISR_RXNE)){};
		*(dst++) = I2C1->RXDR;
	}
	while(!(I2C1->ISR & I2C_ISR_TXE)){};
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	I2C1->ICR |= I2C_ICR_STOPCF;
	I2C1->CR2 &=(~I2C_CR2_RD_WRN) &(~I2C_CR2_NACK);
}

//===============SPI-Functions=============================
unsigned char SPI_Write(unsigned char reg, unsigned char value)														{//SPI_Write
 unsigned char temp;
CS_LO();

while (!(SPI1->SR & SPI_SR_TXE)){};  
SPI1_DR_8bit = reg & 0x3F;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
temp=SPI1_DR_8bit;
	
while (!(SPI1->SR & SPI_SR_TXE)){};	 
SPI1_DR_8bit = value;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
temp=SPI1_DR_8bit;	

CS_HI();
return temp;                // return nRF24L01 status unsigned char
}
unsigned char SPI_Read(unsigned char reg)																									{//SPI read
  unsigned char reg_val;
CS_LO();
	
while (!(SPI1->SR & SPI_SR_TXE)){}; 
	SPI1_DR_8bit = reg | SINGLE_READ;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
reg = SPI1_DR_8bit;
while (!(SPI1->SR & SPI_SR_TXE)){};	 
SPI1_DR_8bit = 0x00;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){};
reg_val = SPI1_DR_8bit;

CS_HI();	
  
  return(reg_val);               // return register value
}





unsigned char SPI_Read_Multi(unsigned char reg, unsigned char * dst, unsigned char count)										{//SPI read Multi

unsigned char status;
	
CS_LO();
while (!(SPI1->SR & SPI_SR_TXE)){};	SPI1_DR_8bit = (reg|BURST_READ);
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; status=SPI1_DR_8bit;	

while (count-- > 0) {
		
	while (!(SPI1->SR & SPI_SR_TXE)){};	SPI1_DR_8bit = 0x00; 
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	*(dst++) = SPI1_DR_8bit;
	}
	 CS_HI();
	 return(status); 
}








//================DATA=====================================
void init_lis3dh(struct lis3dh* data, unsigned char lis_mode) 																																		{//init_lis3dh 

const unsigned char init_lis3d[]={0x57,		//LIS3DH_CTRL_REG1 - All axes, normal, 100Hz, 
																	0x00,		//LIS3DH_CTRL_REG2 -  No HighPass filter
																	0x00,		//LIS3DH_CTRL_REG3 - No interrupts
																	0x88,		//LIS3DH_CTRL_REG4 - BDU - on, HR - On // 0x00 - all defaults
																	0x00,		//LIS3DH_CTRL_REG5 - all defaults
																	0x00,		//LIS3DH_CTRL_REG6 - all defaults
																	0x80		//LIS3DH_TEMP_CFG_REG = TEMP_CFG_REG_ADC_BIT - ADC ENABLE //0x00 - ADC DISABLE
};
	
	if (lis_mode==LIS3DH_I2C_MODE) {
		data->mode=LIS3DH_I2C_MODE;
		writeReg8(LIS3DH_CTRL_REG1, 		init_lis3d[0]); 
		writeReg8(LIS3DH_CTRL_REG2, 		init_lis3d[1]); 
		writeReg8(LIS3DH_CTRL_REG3, 		init_lis3d[2]); 
		writeReg8(LIS3DH_CTRL_REG4, 		init_lis3d[3]); 
		writeReg8(LIS3DH_CTRL_REG5, 		init_lis3d[4]); 
		writeReg8(LIS3DH_CTRL_REG6, 		init_lis3d[5]);
		writeReg8(LIS3DH_TEMP_CFG_REG, 	init_lis3d[6]);		//ADC
		
	}
	else {
		data->mode=LIS3DH_SPI_MODE;
		SPI_Write(LIS3DH_CTRL_REG1, 		init_lis3d[0]); 
		SPI_Write(LIS3DH_CTRL_REG2, 		init_lis3d[1]); 
		SPI_Write(LIS3DH_CTRL_REG3, 		init_lis3d[2]); 
		SPI_Write(LIS3DH_CTRL_REG4, 		init_lis3d[3]); 
		SPI_Write(LIS3DH_CTRL_REG5, 		init_lis3d[4]);
		SPI_Write(LIS3DH_CTRL_REG6, 		init_lis3d[5]);
		SPI_Write(LIS3DH_TEMP_CFG_REG, 	init_lis3d[5]);		// ADC
	}
}

void readLisData(struct lis3dh* data)																											{//read data
	uint8_t buf[6];
	int16_t x,y,z;
	if(data->mode==LIS3DH_I2C_MODE){
		while((readReg8(LIS3DH_STATUS_REG) & 0x8) == 0 );
		readMulti(LIS3DH_OUT_X_L, buf, 6);
		}
	else {
		while((SPI_Read(LIS3DH_STATUS_REG) & 0x8) == 0 );
		SPI_Read_Multi(LIS3DH_OUT_X_L, buf, 6);
	}
	
	x = (int16_t)(buf[1]<<8 | buf[0]);
	y = (int16_t)(buf[3]<<8 | buf[2]);
	z = (int16_t)(buf[5]<<8 | buf[4]);
	
	data->ax = x/182;
	data->ay = y/182;
	data->az = z/182;

}


void Lis_ADC_CMD(struct lis3dh* data,unsigned char off_on)																{//ADC- on\off    1 - 0n, 0 - off
	
	if(data->mode==LIS3DH_I2C_MODE){
		if(off_on==1)	{	writeReg8(LIS3DH_TEMP_CFG_REG, TEMP_CFG_REG_ADC_BIT);}
		else					{	writeReg8(LIS3DH_TEMP_CFG_REG, 0x00);	}
	}
	else {
		if(off_on==1){ 	SPI_Write(LIS3DH_TEMP_CFG_REG, TEMP_CFG_REG_ADC_BIT);}
		else 				 {	SPI_Write(LIS3DH_TEMP_CFG_REG, 0x00);	}
	}
	
}

void Lis_TEMP_CMD(struct lis3dh* data,unsigned char off_on)																{//ADC- on\off    1 - 0n, 0 - off
	
	if(data->mode==LIS3DH_I2C_MODE){
		if(off_on==1)	{	writeReg8(LIS3DH_TEMP_CFG_REG, (TEMP_CFG_REG_TEMP_BIT |TEMP_CFG_REG_ADC_BIT));}
		else					{	writeReg8(LIS3DH_TEMP_CFG_REG, 0x00);}
	}
	else {
		if(off_on==1){ 	SPI_Write(LIS3DH_TEMP_CFG_REG, (TEMP_CFG_REG_TEMP_BIT |TEMP_CFG_REG_ADC_BIT));}
		else 				 {	SPI_Write(LIS3DH_TEMP_CFG_REG, 0x00);	}
	}

}


void readLisADC(struct lis3dh* data)																											{//READ ADC data
	uint8_t adc[6];
	if(data->mode==LIS3DH_I2C_MODE){
		readMulti(LIS3DH_OUT_ADC1_L, adc, 6);
	}
	else {
		SPI_Read_Multi(LIS3DH_OUT_ADC1_L, adc, 6);
	}
	
	data->adc1 =(int16_t)(adc[1]<<8 | adc[0]);
	data->adc2 =(int16_t)(adc[3]<<8 | adc[2]);
	data->adc3 =(int16_t)(adc[5]<<8 | adc[4]);
	
}

void readLisTemp(struct lis3dh* data)																											{//READ ADC data
	uint8_t adc[2];
	if(data->mode==LIS3DH_I2C_MODE){
	//readMulti(LIS3DH_OUT_ADC3_L, adc, 2);
	adc[0]=readReg8(LIS3DH_OUT_ADC3_L);adc[1]=readReg8(LIS3DH_OUT_ADC3_H);	
	}
	else {
	SPI_Read_Multi(LIS3DH_OUT_ADC3_L, adc, 2);
	}
	data->temp = (int16_t)(adc[1]<<8 | adc[0]);
}

	



