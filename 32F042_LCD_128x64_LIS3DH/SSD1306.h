#ifndef SSD1306_h
#define SSD1306_h

#include "stm32f0xx.h"  

void I2C_write(unsigned char byte_send);										//WRITE
void I2C_Start(void);																				//Start
void I2C_Stop(void); 																				//Stop
void CMD(unsigned char byte);
void LCD_Init(void);																				// INIT LCD T191
void LCD_Gotoxy ( unsigned char x, unsigned char y );														    						
void LCD_mode(unsigned char mode);	
void LCD_Char(unsigned char flash_o, unsigned char mode);		// Print Symbol
void LCD_PrintStr(char *s, unsigned char mode);							// Print String
void LCD_Clear(void);      																	// Clear LCD
void LCD_ClearStr(unsigned char y, unsigned char qnt );																					
void LCD_PrintDec(long val,unsigned char mode); 					// Print Dec
void LCD_PrintHex(long val,unsigned char mode); 					// Print Hex
void LCD_PrintBin(unsigned char value,unsigned char mode) ;	// Print Bin
void Flip_mode(unsigned char flip);
void slide_right(unsigned char start_page, unsigned char end_page, unsigned char interval);	
void slide_left(unsigned char start_page, unsigned char end_page, unsigned char interval);						
void stop_slide(void);																																
void start_slide(void);																															
void scroll_vright(unsigned char start_page, unsigned char end_page, unsigned char interval,unsigned char v_offset);
void scroll_vleft(unsigned char start_page, unsigned char end_page, unsigned char interval,unsigned char v_offset);
void scroll_area(unsigned char start_page, unsigned char end_page);

#endif
