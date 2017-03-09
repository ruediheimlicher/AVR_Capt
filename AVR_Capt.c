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
#include <avr/pgmspace.h>
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

/*
#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4
#define SCLPIN		5
*/

// Syntax: xy = pgm_read_word(&KTY[xy])
#define KTY_OFFSET   30             // Offset, Start bei bei -32 °C
#define ADC_OFFSET   207            // Startwert der ADC-Messung
#define KTY_FAKTOR   96             // 0x60, Multiplikator
const uint16_t KTY[] PROGMEM =
{
   0x1D,	0x1FC,	0x3E8,	0x5B2,	0x789,	0x942,	0xB08,	0xCB2,
   0xE6A,	0x1007,	0x11B4,	0x1346,	0x14E8,	0x1672,	0x180B,	0x198D,
   0x1B1E,	0x1C9A,	0x1E24,	0x1F9A,	0x211F,	0x228F,	0x23FD,	0x257B,
   0x26E5,	0x285F,	0x29C6,	0x2B3D,	0x2CA0,	0x2E14,	0x2F75,	0x30E7,
   0x3246,	0x33B5,	0x3513,	0x3681,	0x37DD,	0x394A,	0x3AA5,	0x3C11,
   0x3D6B,	0x3ED7,	0x4031,	0x419C,	0x42F6,	0x4461,	0x45BB,	0x4716,
   0x4882,	0x49DD,	0x4B49,	0x4CA5,	0x4E13,	0x4F70,	0x50DE,	0x523D,
   0x53AD,	0x550C,	0x567F,	0x57E0,	0x5954,	0x5AB7,	0x5C2D,	0x5D93,
   0x5F0B,	0x6073,	0x61EF,	0x6359,	0x64D7,	0x6645,	0x67C6,	0x6936,
   0x6AA8,	0x6C2F,	0x6DA4,	0x6F2E,	0x70A7,	0x7235,	0x73B2,	0x7544,
   0x76C5,	0x785C,	0x79E2,	0x7B7D,	0x7D07,	0x7EA8,	0x8037,	0x81DD,
   0x8371,	0x851D,	0x86B6,	0x8868,	0x8A07,	0x8BBF,	0x8D65,	0x8F23,
   0x90CF,	0x927E,	0x9447,	0x95FE,	0x97CE,	0x998B,	0x9B63,	0x9D29,
   0x9F09,	0xA0D7,	0xA2C0,	0xA496,	0xA688,	0xA867,	0xAA63,	0xAC4C,
   0xAE52,	0xB045,	0xB256,	0xB453,	0xB66F,	0xB877,	0xBAA0,	0xBCB3,
   0xBEE8,	0xC109,	0xC32F,	0xC578,	0xC7AC,	0xCA04,	0xCC46,	0xCEAD,
   0xD0FE,	0xD375,	0xD5D7,	0xD85F,	0xDAD1,	0xDD6C,	0xDFF0,	0xE29E,
   0xE535,	0xE7F7,	0xEAA3,	0xED7B,	0xF03C,	0xF32C,	0xF604,	0xF90E,
   
};


#define PT_OFFSET   30             // Offset, Start bei bei -30 °C
#define PT_AC_OFFSET   880            // Startwert der ADC-Messung
#define PT_FAKTOR   32             // 0x60, Multiplikator

// Syntax: xy = pgm_read_word(&PT[xy])
const uint16_t PT[] PROGMEM =
{
   0x0,	0x2F,	0x70,	0xB1,	0xF2,	0x133,	0x174,	0x1B5,
   0x1F6,	0x238,	0x279,	0x2BA,	0x2FC,	0x33D,	0x37F,	0x3C0,
   0x402,	0x443,	0x485,	0x4C6,	0x508,	0x54A,	0x58C,	0x5CD,
   0x60F,	0x651,	0x693,	0x6D5,	0x717,	0x759,	0x79B,	0x7DD,
   0x81F,	0x862,	0x8A4,	0x8E6,	0x929,	0x96B,	0x9AD,	0x9F0,
   0xA32,	0xA75,	0xAB7,	0xAFA,	0xB3D,	0xB7F,	0xBC2,	0xC05,
   0xC48,	0xC8B,	0xCCE,	0xD11,	0xD54,	0xD97,	0xDDA,	0xE1D,
   0xE60,	0xEA3,	0xEE7,	0xF2A,	0xF6D,	0xFB1,	0xFF4,	0x1038,
   0x107B,	0x10BF,	0x1102,	0x1146,	0x118A,	0x11CD,	0x1211,	0x1255,
   0x1299,	0x12DD,	0x1321,	0x1365,	0x13A9,	0x13ED,	0x1431,	0x1475,
   0x14BA,	0x14FE,	0x1542,	0x1586,	0x15CB,	0x160F,	0x1654,	0x1698,
   0x16DD,	0x1722,	0x1766,	0x17AB,	0x17F0,	0x1835,	0x187A,	0x18BF,
   0x1903,	0x1949,	0x198E,	0x19D3,	0x1A18,	0x1A5D,	0x1AA2,	0x1AE8,
   0x1B2D,	0x1B72,	0x1BB8,	0x1BFD,	0x1C43,	0x1C88,	0x1CCE,	0x1D14,
   0x1D59,	0x1D9F,	0x1DE5,	0x1E2B,	0x1E71,	0x1EB7,	0x1EFD,	0x1F43,
   0x1F89,	0x1FCF,	0x2015,	0x205C,	0x20A2,	0x20E8,	0x212F,	0x2175,
   0x21BC,	0x2202,	0x2249,	0x228F,	0x22D6,	0x231D,	0x2364,	0x23AA,
   0x23F1,	0x2438,	0x247F,	0x24C6,	0x250E,	0x2555,	0x259C,	0x25E3,
   0x262A,	0x2672,	0x26B9,	0x2701,	0x2748,	0x2790,	0x27D7,	0x281F,
   0x2867,	0x28AF,	0x28F6,	0x293E,	0x2986,	0x29CE,	0x2A16,	0x2A5E,
   0x2AA7,	0x2AEF,	0x2B37,	0x2B7F,	0x2BC8,	0x2C10,	0x2C59,	0x2CA1,
   0x2CEA,	0x2D32,	0x2D7B,	0x2DC4,	0x2E0C,	0x2E55,	0x2E9E,	0x2EE7,
   0x2F30,	0x2F79,	0x2FC2,	0x300C,	0x3055,	0x309E,	0x30E7,	0x3131,
   0x317A,	0x31C4,	0x320D,	0x3257,	0x32A1,	0x32EB,	0x3334,	0x337E,
   0x33C8,	0x3412,	0x345C,	0x34A6,	0x34F0,	0x353B,	0x3585,	0x35CF,
   0x3619,	0x3664,	0x36AE,	0x36F9,	0x3744,	0x378E,	0x37D9,	0x3824,
   0x386F,	0x38B9,	0x3904,	0x394F,	0x399B,	0x39E6,	0x3A31,	0x3A7C,
   0x3AC8,	0x3B13,	0x3B5E,	0x3BAA,	0x3BF5,	0x3C41,	0x3C8D,	0x3CD8,
   0x3D24,	0x3D70,	0x3DBC,	0x3E08,	0x3E54,	0x3EA0,	0x3EED,	0x3F39,
   0x3F85,	0x3FD1,	0x401E,	0x406A,	0x40B7,	0x4104,	0x4150,	0x419D,
   0x41EA,	0x4237,	0x4284,	0x42D1,	0x431E,	0x436B,	0x43B8,	0x4405,
   0x4453,	0x44A0,	0x44EE,	0x453B,	0x4589,	0x45D6,	0x4624,	0x4672,
   0x46C0,	0x470E,	0x475C,	0x47AA,	0x47F8,	0x4846,	0x4895,	0x48E3,
};



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
#define COMP_PORT PORTB
#define COMP_DDR DDRB

// Pins fuer Drive der RC
#define COMP_DRIVE_PIN_A  1
#define COMP_DRIVE_PIN_B  2

#define COMP_ADC_PORT PORTC
#define COMP_ADC_DDR DDRC

#define COMP_ADC_PIN_A  4
#define COMP_ADC_PIN_B  5

#define COMP_AIN_PORT   PORTD
#define COMP_AIN_DDR    DDRD
#define COMP_AIN0       6
#define COMP_AIN1       7


#define MULTIPLEX 1

volatile uint16_t captured_value;
volatile uint8_t captured;
volatile uint8_t overflow=0;
volatile uint8_t captcounter=0;
volatile uint16_t mittelwertA[4];
volatile uint16_t mittelwertB[4];
volatile uint8_t mposA=0;
volatile uint8_t mposB=0;
volatile uint8_t adckanal=0;

// end ACD

void timer1_comp(void)
{
   // Set pin for driving resistor low.
   COMP_DDR |= (1<<COMP_DRIVE_PIN_A);
   COMP_PORT &= ~(1<<COMP_DRIVE_PIN_A);
   COMP_DDR |= (1<<COMP_DRIVE_PIN_B);
   COMP_PORT &= ~(1<<COMP_DRIVE_PIN_B);
   
   // Disable the digital input buffers.
   //   DIDR = (1<<AIN1D) | (1<<AIN0D);
   if (MULTIPLEX)
   {
      // ADC-Eingaenge fuer Capt
      COMP_ADC_DDR &= ~(1<<COMP_ADC_PIN_A);
      COMP_ADC_PORT &= ~(1<<COMP_ADC_PIN_A);
      
      COMP_ADC_DDR &= ~(1<<COMP_ADC_PIN_B);
      COMP_ADC_PORT &= ~(1<<COMP_ADC_PIN_B);
      
      // AIN0, AIN1 Eingang
      COMP_AIN_DDR &= ~(1<<COMP_AIN0);
      COMP_AIN_DDR &= ~(1<<COMP_AIN1);
      
      
      SFIOR |= (1<<ACME);
      //ADMUX = 3;
   }
   
   
   //ADCSRA =0;//| = (1<<ADEN);                    // disable ADC if necessary
   ACSR =   (1<<ACIC) | (1<<ACIS1) | (1<<ACIS0);   // Comparator enabled, no bandgap, input capture.
   // Timer...
   TCCR1A = 0;
   TCCR1B =   (1<<CS10);                        // F_CPU / 1
   //TCCR1B =  (1<<ICES1);                      // Input capture on rising edge
   TCNT1 = 0;
   TIMSK |= (1<<TOIE1) | (1<<TICIE1);           // Timer interrupts on capture and overflow.
}

ISR(TIMER1_CAPT_vect)
{
   // Save the captured value and drop the drive line.
   if (captured == 0)
   {
      // captured_value = ICR1;
      captcounter++;
      
      if (adckanal == COMP_ADC_PIN_A)
      {
         mittelwertA[mposA++] = ICR1;           // Ringbuffer fuer gleitenden Mittelwert
         mposA &= 0x03;                         // position incrementieren
         COMP_PORT &= ~(1<<COMP_DRIVE_PIN_A);   // auf 4 beschraenken
      }
      
      if (adckanal == COMP_ADC_PIN_B)
      {
         mittelwertB[mposB++] = ICR1;
         mposB &= 0x03;
         COMP_PORT &= ~(1<<COMP_DRIVE_PIN_B);
      }
      TCNT1 = 0;
      captured = 1;
   }
   //TCNT1 = 0;
}

ISR(TIMER1_OVF_vect)
{
   overflow++;
   COMP_PORT &= ~(1<<COMP_DRIVE_PIN_A);
   COMP_PORT &= ~(1<<COMP_DRIVE_PIN_B);
   // If we overflowed, the capacitor is bigger than
   // this range supports. Use a smaller series resistor.
}

void switchChannel (uint8_t channel) // https://github.com/Teknoman117/avr/blob/master/unsorted/mxx4lib/acomp.h
{ //switched the negative input pin of the Analog comparator in multiplexer mode
   ADMUX = channel & 0x07;     //set the multiplexer channel and mask unused buts
   __asm ("NOP");              //wait 2 clock cycles
   __asm ("NOP");
}

void slaveinit(void)
{
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD
   LOOPLED_DDR |= (1<<LOOPLED_PIN);
   DDRB |= (1<<0); // HI fuer Spannungsteiler
   PORTB |= (1<<0);
} 

uint16_t floatmittel(uint16_t* werte)
{
   uint8_t pos=4;
   uint16_t mittel =0;
   while (pos--)
   {
      mittel += werte[pos]/4;
   }
   return mittel;
}

int main (void)
{
	slaveinit();
	
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	delay_ms(1000);
	lcd_cls();
	lcd_puts("READY");
	
	delay_ms(1000);
   lcd_gotoxy(0,0);
   lcd_puts("     ");
   
   // timer 1 einrichten
   timer1_comp();
   sei();

#pragma mark while
	while (1) 
	{
		
		loopCount0 ++;
		//_delay_ms(2);
      
		if (loopCount0 >=0x0AFF)
      {
         
         //LOOPLED_PORT ^= (1<<LOOPLED_PIN);
         loopCount1++;
         
         if ((loopCount1 >0x0080) && (!(Programmstatus & (1<<MANUELL))))
         {
            
            if (MULTIPLEX)
            {
               // Werte reset
               captured_value=0;
               captured = 0;
               // Kanal waehlen
               adckanal = COMP_ADC_PIN_A;
               ADMUX = COMP_ADC_PIN_A & 0x07; // 4
               // counter reset
               TCNT1 = 0;
               // Pin HI
               COMP_PORT |= (1<<COMP_DRIVE_PIN_A);
               while (!captured); // warten, captured wird in ISR gesetzt
               
               _delay_us(100);
               
               captured_value=0;
               captured = 0;
               adckanal = COMP_ADC_PIN_B;
               
               ADMUX = COMP_ADC_PIN_B & 0x07; // 5
               TCNT1 = 0;
               COMP_PORT |= (1<<COMP_DRIVE_PIN_B);
               while (!captured);
            }
            //lcd_gotoxy(0,0);
            //lcd_putint(overflow);
            //               captured_value=0;
            
            lcd_gotoxy(0,0);
            lcd_puts("A:");
            lcd_putint16(floatmittel((void*)mittelwertA));
            uint16_t PT_temperatur = floatmittel((void*)mittelwertA);
            
            
            // http://embeddedgurus.com/stack-overflow/2009/06/division-of-integers-by-constants/
            // file:///Users/ruediheimlicher/Documents/Elektronik/Netzteil/division Stack-Overflow.webarchive
            
            
            //PT_temperatur = (((uint32_t)PT_temperatur * (uint32_t)0xE38F) >> 16) >> 3; // /9
             //PT_temperatur /= 2;
            
            //PT_temperatur = PT_temperatur*10;
            //PT_temperatur /= 83;
            //PT_temperatur/=176;
            
            PT_temperatur /= 18; // Umrechnen auf Ohm
            lcd_putc(' ');
            lcd_putint12(PT_temperatur);
            
            uint16_t PT_tableindex = ((PT_temperatur - PT_AC_OFFSET)>>3); // abrunden auf Intervalltakt
            lcd_putc(' ');
            // lcd_putint(tableindex);
            uint8_t PT_col = (PT_temperatur - PT_AC_OFFSET) & 0x07;

            
            uint16_t PT_wert = pgm_read_word(&PT[PT_tableindex]); // Wert in Tabelle, unterer Wert
            
            lcd_gotoxy(0,1);
            lcd_putint(PT_tableindex);
            lcd_putc(' ');
            lcd_putint2(PT_col);
            lcd_putc(' ');
            lcd_putint16(PT_wert);
           
            uint16_t PT_diff = 0;
            if (PT_col) // nicht exakter wert, interpolieren
            {
               PT_diff = (pgm_read_word(&PT[PT_tableindex+1]))-PT_wert;
               
               PT_diff = (PT_diff * PT_col)>>3;
               PT_wert += (PT_diff * PT_col)>>3;
            }
            lcd_putc(' ');
            lcd_putint12(PT_diff);
            
            //PT_wert -=91;
            
            lcd_putc(' ');
            lcd_gotoxy(16,0);
            lcd_putint12((PT_wert/PT_FAKTOR)-PT_OFFSET);

            
//            lcd_gotoxy(0,1);
//            lcd_puts("B:");
 //           lcd_putint16(floatmittel((void*)mittelwertB));
            
            
           // lcd_gotoxy(16,0);
           // lcd_putint(captcounter);
            
            captured = 0;
            
            LOOPLED_PORT ^= (1<<LOOPLED_PIN);
            loopCount1=0;
            
            
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
