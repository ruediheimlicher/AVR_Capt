//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
//#define F_CPU 4000000UL  // 4 MHz
#include <avr/delay.h>
#include "lcd.c"
#include "adc.c"
#include "utils.c"

#include "defines.h"
uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;

#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4
#define SCLPIN		5



volatile uint8_t					Programmstatus=0x00;
uint8_t Tastenwert=0;
uint8_t TastaturCount=0;
volatile uint16_t					Manuellcounter=0; // Countr fuer Timeout	
uint16_t TastenStatus=0;
uint16_t Tastencount=0;
uint16_t Tastenprellen=0x01F;





void delay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

// ACD https://www.avrprogrammers.com/howto/attiny-comparator
#define DRIVE_PORT PORTB
#define DRIVE_PORT_DDR DDRB
#define DRIVE_PIN_MSK 0x04

volatile uint16_t captured_value;
volatile uint8_t captured;
volatile uint8_t overflow=0;
volatile uint8_t captcounter=0;
volatile uint16_t mittelwert[4];
volatile uint8_t mpos=0;

// end ACD

void timer1_comp()
{
   // Set pin for driving resistor low.
   DRIVE_PORT_DDR |= DRIVE_PIN_MSK;
   DRIVE_PORT &= ~DRIVE_PIN_MSK;
   
   // Disable the digital input buffers.
   //   DIDR = (1<<AIN1D) | (1<<AIN0D);
   
   SFIOR |= (1<<ACME);
   ADCSRA &= ~(1<<ADEN);
   
   
   // Comparator enabled, no bandgap, input capture.
   ACSR =    (0<<ACIE) | (1<<ACIC) | (1<<ACIS1) | (1<<ACIS0);
   
   // Timer...
   TCCR1A = 0;
   
   // Input capture on rising edge, sysclk/1.
   TCCR1B =  (1<<ICES1) | (1<<CS10);
   
   TCNT1 = 0;
   
   // Timer interrupts on capture and overflow.
   TIMSK |= (1<<TOIE1) | (1<<TICIE1);
   }

ISR(TIMER1_CAPT_vect)
{
   // Save the captured value and drop the drive line.
   if (captured == 0)
   {
      // captured_value = ICR1;
      mittelwert[mpos++] = ICR1;
      mpos &= 0x03;      //TIFR |= (1<<ICF1);
      captcounter++;
   
      DRIVE_PORT &= ~DRIVE_PIN_MSK;
      TCNT1 = 0;
      captured = 1;
      }
   //TCNT1 = 0;
}

ISR(TIMER1_OVF_vect)
{
   overflow++;
   //DRIVE_PORT &= ~DRIVE_PIN_MSK;
   // If we overflowed, the capacitor is bigger than
   // this range supports. Use a smaller series resistor.
}



void slaveinit(void)
{

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD
   LOOPLED_DDR |= (1<<LOOPLED_PIN);

   DDRD &= ~(1<<6);
   DDRD &= ~(1<<7);
}

uint16_t floatmittel(uint16_t* werte)
{
   uint8_t pos=4;
   uint16_t mittel =0;
   while (pos--)
   {
      mittel += werte[pos];
   }
   mittel /= 4;
   return mittel;
}

int main (void)
{
	/* INITIALIZE */
//	LCD_DDR |=(1<<LCD_RSDS_PIN);
//	LCD_DDR |=(1<<LCD_ENABLE_PIN);
//	LCD_DDR |=(1<<LCD_CLOCK_PIN);
	
	
	
	slaveinit();
	
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	delay_ms(1000);
	lcd_cls();
	lcd_puts("READY");
	
	delay_ms(1000);
   lcd_gotoxy(0,0);
   lcd_puts("     ");
	uint8_t i=0;
   timer1_comp();
   sei();

#pragma mark while
	while (1) 
	{
		
		loopCount0 ++;
		//_delay_ms(2);
      if(ACSR & 0x20)
      {
         captcounter;
      }
		if (loopCount0 >=0x0AFF)
		{
			
			//LOOPLED_PORT ^= (1<<LOOPLED_PIN);
			loopCount1++;
			
			if ((loopCount1 >0x0080) && (!(Programmstatus & (1<<MANUELL))))
			{
            
				{
               
               ADMUX |= (1<<MUX2) | (1<<MUX0);
               lcd_gotoxy(0,0);
               lcd_putint(overflow);
               captured_value=0;
               captured = 0;
               
               TCNT1 = 0;
               PORTB |= DRIVE_PIN_MSK;
                while (!captured);
               lcd_gotoxy(6,0);
               lcd_putint16(floatmittel(mittelwert));
               //lcd_putint16((captured_value-11000)/2);
               //captured_value=0;
               lcd_gotoxy(16,0);
               lcd_putint(captcounter);

               captured = 0;
               
					LOOPLED_PORT ^= (1<<LOOPLED_PIN);
					loopCount1=0;
				}

			}
			
			loopCount0 =0;
		}
		
      
      
#pragma mark Tastatur 
		/* ******************** */
      if (TASTATUR_ON)
      {
		initADC(TASTATURPIN);
		Tastenwert=(readKanal(TASTATURPIN)>>2);
		
//		lcd_gotoxy(3,1);
//		lcd_putint(Tastenwert);
//		Tastenwert=0;
		if (Tastenwert>5)
		{
			/*
			 0:											1	2	3
			 1:											4	5	6
			 2:											7	8	9
			 3:											x	0	y
			 4: Schalterpos -
			 5: Manuell ein
			 6: Schalterpos +
			 7: 
			 8: 
			 9: 
			 
			 12: Manuell aus
			 */
			 
			TastaturCount++;
			if (TastaturCount>=200)
			{
				
				 
				 lcd_gotoxy(17,1);
				 lcd_puts("T:  \0");
				 //lcd_putint(Tastenwert);
				 
				uint8_t Taste=Tastenwahl(Tastenwert);
				//Taste=0;
				 lcd_gotoxy(19,1);
				 lcd_putint1(Taste);
				 //delay_ms(600);
				// lcd_clr_line(1);
				 

				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
//				lcd_gotoxy(18,1);
//				lcd_putint2(Taste);
				
				switch (Taste)
				{
					case 0:// Schalter auf Null-Position
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
							Manuellcounter=0;
							Programmstatus |= (1<<MANUELLNEU);
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SI:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SP\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							*/
						}
						
					}break;
						
					case 1:	//	
					{ 
					if (Programmstatus & (1<<MANUELL))
						{
						Manuellcounter=0;
						
						}
					}break;
						
					case 2://
					{ 
					
						if (Programmstatus & (1<<MANUELL))
						{
						Manuellcounter=0;
						
						
						}
						
					}break;
						
					case 3: //	Uhr aus
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
						uint8_t i=0;
						Manuellcounter=0;
						

						}
					}break;
						
					case 4://
					{ 
                  uint8_t i=0;

					}break;
						
					case 5://
					{ 
						Programmstatus |= (1<<MANUELL);	// MANUELL ON
						Manuellcounter=0;
						MANUELL_PORT |= (1<<MANUELLPIN);
						Programmstatus |= (1<<MANUELLNEU);
						lcd_clr_line(1);
						/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
						*/
					}break;
						
					case 6://
					{ 
					
					}break;
						
					case 7:// Schalter rückwaerts
					{ 
						if ((Programmstatus & (1<<MANUELL)) )
						{
							Manuellcounter=0;
							Programmstatus |= (1<<MANUELLNEU);
							//OSZIALO;
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							//OSZIAHI;
							*/
						}
						else 
						{
							
						}
	
					}break;
						
					case 8://
					{ 
						
					}break;
						
					case 9:// Schalter vorwaerts
					{ 
						Manuellcounter=0;
						if ((Programmstatus & (1<<MANUELL)) )
						{
							//OSZIALO;
							Programmstatus |= (1<<MANUELLNEU);
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer

							//OSZIAHI;
							*/
							
						}
						else 
						{
							//lcd_gotoxy(10,0);
							//lcd_puts("S:!\0");
						}
					

					}break;

					case 10:// *
					{ 
						
					}break;

					case 11://
					{ 
						
					}break;
						
					case 12: // # Normalbetrieb einschalten
					{
						Programmstatus &= ~(1<<MANUELL); // MANUELL OFF
						Programmstatus &= ~(1<<MANUELLNEU);
						MANUELL_PORT &= ~(1<<MANUELLPIN);
					}
						
				}//switch Tastatur
				
//				delay_ms(400);
//				lcd_gotoxy(18,1);
//				lcd_puts("  ");		// Tastenanzeige loeschen

			}//if TastaturCount	
			
		}
      } // if TASTATUR_ON
	}
	
	
	return 0;
}
