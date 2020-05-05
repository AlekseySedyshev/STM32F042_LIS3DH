#include "stm32f0xx.h"        // Device header
#include "SSD1306.h"
#include "LIS3DH.h"
#include <math.h>

//#define I2C_100				1	//		100 kHz i2C Freq
	#define I2C_400				1	//		400 kHz i2C Freq
	#define I2C_GPIOB			1	//		i2C PB6,PB7
//#define I2C_GPIOF			1	//		i2C PF0,PF1

uint8_t i,j,flag=0;

uint16_t TimingDelay,led_count,sec01_count=0,sec01_f=0,sec05_f=0,s=0;
uint16_t counter;
struct lis3dh lis_data;

void TimingDelayDec(void) 																													{ //msec - timer
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;sec05_f++;}
 if (!sec01_count) {sec01_count=100;sec01_f++;}
 led_count--;
 sec01_count--;
 }

void TIM17_IRQHandler(void)																													{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void TIM2_IRQHandler(void)																													{
if (TIM2->SR & TIM_SR_TIF) {
	//				counter = TIM2->CNT/469;
  //				TIM2->SR &=(~TIM_SR_TIF);
}
}
void delay_ms (uint16_t DelTime) 																										{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}



//int AX_offset = 0,AY_offset = 0,AZ_offset = 0;//1700;

const float Pi=3.1415926535897932384626433832795;
float	kalman_newx,kalman_newy,kalman_newz,cov_newx,cov_newy,cov_newz;
float	cov_oldx=1,cov_oldy=1,cov_oldz=1;
float Xkalman,Ykalman,Zkalman,
			Xkalman_old=0,Ykalman_old=0,Zkalman_old=0;

void kalman_filter (void)																														{

			kalman_newx = Xkalman;	cov_newx = cov_oldx + 0.50;
			kalman_newy = Ykalman;	cov_newy = cov_oldy + 0.50;
			kalman_newz = Zkalman;	cov_newz = cov_oldz + 0.50;
	
		float kalman_gainx 			= cov_newx / (cov_newx + 0.9);
		float kalman_gainy 			= cov_newy / (cov_newy + 0.9);
		float kalman_gainz 			= cov_newz / (cov_newz + 0.9);
	
	  Xkalman = kalman_newx + (kalman_gainx * (lis_data.ax - kalman_newx));
		Ykalman = kalman_newy + (kalman_gainy * (lis_data.ay - kalman_newy));
		Zkalman = kalman_newz + (kalman_gainz * (lis_data.az - kalman_newz));

	  cov_newx = (1 - kalman_gainx) * cov_oldx; cov_oldx = cov_newx;
		cov_newy = (1 - kalman_gainy) * cov_oldy; cov_oldy = cov_newy;
		cov_newz = (1 - kalman_gainz) * cov_oldz; cov_oldz = cov_newz;

	if(Xkalman>89) Xkalman=90;   
	if(Ykalman>89) Ykalman=90;
	if(Zkalman>89) Zkalman=90;

	if(Xkalman<-89) Xkalman=-90; 
	if(Ykalman<-89) Ykalman=-90;
	if(Zkalman<-89) Zkalman=-90;
	}


void initial (void)																																	{
//---------------TIM17------------------
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 								//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;								//Pb0-Out 
#ifdef I2C_GPIOB
//------------I2C1 GPIOB_SETTING ---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOBEN;
	GPIOB->MODER 		|=GPIO_MODER_MODER6_1 		| GPIO_MODER_MODER7_1; 							// Alt -mode /Pb6 -SCL , Pb7- SDA
	GPIOB->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR6 	| GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->OTYPER		|=GPIO_OTYPER_OT_6 				| GPIO_OTYPER_OT_7;
	GPIOB->AFR[0] 	|=(1<<GPIO_AFRL_AFRL6_Pos) |(1<<GPIO_AFRL_AFRL7_Pos);  				// I2C - Alternative PB7, PB6
#endif	
#ifdef I2C_GPIOF	
	//------------I2C1 GPIOF_SETTING---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOFEN;
	GPIOF->MODER 		|=GPIO_MODER_MODER0_1 		| GPIO_MODER_MODER1_1; 							// Alt -mode /Pf0 - SDA, Pf1- SCL
	GPIOF->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR0 	| GPIO_OSPEEDER_OSPEEDR1;
	GPIOF->OTYPER		|=GPIO_OTYPER_OT_0 				| GPIO_OTYPER_OT_1;
	GPIOF->AFR[0] 	|=(1<<GPIO_AFRL_AFRL0_Pos) |(1<<GPIO_AFRL_AFRL1_Pos);  				// I2C - Alternative
#endif	
	RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
#ifdef I2C_100	
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_PRESC_Pos); 	//100 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x13	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0xF	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x2	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x4	<<I2C_TIMINGR_SCLDEL_Pos);
#endif
#ifdef I2C_400	
	I2C1->TIMINGR |=(0x0	<<I2C_TIMINGR_PRESC_Pos); 	//400 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x9	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLDEL_Pos);
#endif	
	I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
	I2C1->CR1 |=I2C_CR1_PE;
	
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; 								//
	GPIOA->MODER |= GPIO_MODER_MODER1_0;							//Output mode;							
	
	GPIOA->MODER &=(~GPIO_MODER_MODER0);							//Input_mode
	GPIOA->PUPDR |=GPIO_PUPDR_PUPDR0_1;								//Pull-Up
	
//------------------------SPI-----------------------------------
	RCC->AHBENR 		|=RCC_AHBENR_GPIOAEN;
	GPIOA->OSPEEDR 	|= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 
										|GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOA->MODER 		|=GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 	//Pa2, Pa4 - out,Pa5..7 - Alt_mode 
	GPIOA->AFR[0] 	|=(0<<GPIO_AFRL_AFRL7_Pos) |(0<<GPIO_AFRL_AFRL6_Pos) | (0<<GPIO_AFRL_AFRL5_Pos);  // SPI - Alternative
	GPIOA->MODER 		|=GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0; //Pa3, Pa4 - out, Pa0 Input inerrupt
	
	RCC->APB2ENR |=RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |=SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | (2<<SPI_CR1_BR_Pos);  // if HSI8 - SpiSpeed (BR=2) - 1MHz
	SPI1->CR2 |=SPI_CR2_FRXTH;
	SPI1->CR1 |=SPI_CR1_SPE;
	CS_HI();
/*	
//----------------------EXTI-----------------------------------
	RCC->APB2ENR |=RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->FTSR |= EXTI_FTSR_TR0; //Falling
	
  NVIC_SetPriority(EXTI0_1_IRQn, 2); 
  NVIC_EnableIRQ(EXTI0_1_IRQn); 
	//EXTI->IMR |= EXTI_IMR_MR0;				//Cannot activate immediately, because need switch GD0 to Rx\Tx buf Interrupt mode
	__enable_irq ();	
*/
} 



int main(void)																																			{

initial();
delay_ms (200);	
LCD_Init();LCD_Clear();
LCD_Gotoxy (5,0);LCD_PrintStr("TEST LIS3DH",1); LCD_PrintStr(" ID 0x",0);

init_lis3dh(&lis_data,LIS3DH_I2C_MODE);			//Init
delay_ms(10);	
if (lis_data.mode==LIS3DH_I2C_MODE) {LCD_PrintHex(readReg8(LIS3DH_WHO_AM_I),0);}
else {LCD_PrintHex(SPI_Read(LIS3DH_WHO_AM_I),0);}
Lis_ADC_CMD(&lis_data,LIS_OFF);	
//-----------------------------initial data----------------------------------

while (1)  /* Main loop */
{
 if (sec05_f)											{// Run - 1 time in second
		sec05_f=0;
	 LCD_Gotoxy (10,7);LCD_PrintStr("Cnt ",0);LCD_PrintDec(counter++,0);LCD_PrintStr("   ",0);
 }
 if(sec01_f) 											{//Run - 10 times per second
	 sec01_f=0;
	 
readLisData(&lis_data);
kalman_filter();

	 
//Lis_ADC_CMD(&lis_data,LIS_ON);delay_ms(5);
//readLisADC(&lis_data);

//Lis_TEMP_CMD(&lis_data,LIS_ON);delay_ms(5);	 
//readLisTemp(&lis_data);

//Lis_ADC_CMD(&lis_data,LIS_OFF);	 
		 
//	double pitch	= 	180*atan(lis_data.ax/sqrt(lis_data.ay*lis_data.ay + lis_data.az*lis_data.az))/Pi;
//	double roll  	=  	180*atan(lis_data.ay/sqrt(lis_data.ax*lis_data.ax + lis_data.az*lis_data.az))/Pi;
//	double yaw 		= 	180*atan(lis_data.az/sqrt(lis_data.ax*lis_data.ax + lis_data.az*lis_data.az))/Pi;
	
LCD_Gotoxy (1,2);LCD_PrintStr("aX ",0);LCD_PrintDec(lis_data.ax,0);LCD_PrintStr("    ",0); 
LCD_Gotoxy (1,3);LCD_PrintStr("aY ",0);LCD_PrintDec(lis_data.ay,0);LCD_PrintStr("    ",0); 
LCD_Gotoxy (1,4);LCD_PrintStr("aZ ",0);LCD_PrintDec(lis_data.az,0);LCD_PrintStr("    ",0); 

LCD_Gotoxy (50,2);LCD_PrintStr("Xc ",0);LCD_PrintDec(Xkalman,0);LCD_PrintStr("   ",0); 
LCD_Gotoxy (50,3);LCD_PrintStr("Yc ",0);LCD_PrintDec(Ykalman,0);LCD_PrintStr("   ",0); 
LCD_Gotoxy (50,4);LCD_PrintStr("Zc ",0);LCD_PrintDec(Zkalman,0);LCD_PrintStr("   ",0); 

int32_t summ=sqrt(((double)(lis_data.ax*lis_data.ax*33124) + (double)(lis_data.ay*lis_data.ay*33124) +(double)(lis_data.az*lis_data.az*33124))/3);
	
LCD_Gotoxy (1,6);LCD_PrintStr("SUMM =  ",0);LCD_PrintDec(summ,0);LCD_PrintStr("   ",0);
 
}
		

} // end - main loop 
} // end - Main  
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
