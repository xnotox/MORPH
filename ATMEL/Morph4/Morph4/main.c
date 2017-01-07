#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "bitop.h"
#include "hardware.h"


//EARLY STAGE ; SEEMS TO WORK ; NOT OPTIMIZED

#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#define BAUD 9600UL
#define MYUBRR F_CPU/16/BAUD-1


#define  debug 0

#define  LED1_ON BIT_SET8(&PORTC,5)
#define  LED1_OFF BIT_CLR8(&PORTC,5)
#define  LED2_ON BIT_SET8(&PORTC,4)
#define  LED2_OFF BIT_CLR8(&PORTC,4)
#define  LED3_ON BIT_SET8(&PORTD,2)
#define  LED3_OFF BIT_CLR8(&PORTD,2)
#define  LED4_ON BIT_SET8(&PORTD,3)
#define  LED4_OFF BIT_CLR8(&PORTD,3)
#define SELECT_DAC1 PORTB &= ~(1<<PB0)
#define DESELECT_DAC1 PORTB |= (1<<PB0)
#define SELECT_DAC2 PORTB &= ~(1<<PB1)
#define DESELECT_DAC2 PORTB |= (1<<PB1)
#define LPF_DEPTH 6

char*  dbg_msg;
int32_t cv=0;
int32_t calc_fader;
uint8_t mux=0;
uint16_t ACC_A,ACC_B,c,d;
uint8_t i;
poti width,position;
fader crossfader[4];



void SYSTEM_init(void)
{
	ADC_Init();
	crossfader[0].target=0;
	crossfader[0].target=1;
	crossfader[0].target=2;
	crossfader[0].target=3;
	DDRB |= (1<<PB0); // chip select for DAC1
	DDRB |= (1<<PB1); // chip select for DAC2
	DDRD = 0xFE;
	DDRC |= (1<<PC5);
	DDRC |= (1<<PC4);
	sei();
	DESELECT_DAC1;
	DESELECT_DAC2;
	SPI_init();
	if(debug == 1)
	{	Uart_init(MYUBRR);
		Uart_puts("Debug Mode");
	}
}

void LPF_Read(poti p)
{
	
	p.raw = ADC_Read(p.adc_channel);
	p.raw <<= p.lpf_fp_precision;
	p.smoothed = (p.smoothed << p.lpf_beta) - p.smoothed;
	p.smoothed += p.raw;
	p.smoothed >>= p.lpf_beta;
	p.smoothed >>= p.lpf_fp_precision;
}
	


uint16_t rect(int16_t x)
{
	if (x<0) return 0;
	else return (uint16_t) x;
}

uint16_t calc(int16_t peak)
{
	calc_fader=(abs(peak-cv));
	calc_fader=calc_fader*width;
	calc_fader=333-(calc_fader>>9);
	calc_fader=rect(calc_fader);
	calc_fader=calc_fader<<3;
	return (uint16_t) calc_fader;
}

int main(void){

	SYSTEM_init();
	
	while (1) {
		
		
		PORTD = PORTD & 0x0F;
		PORTD = PORTD | (mux<<4);  	// SWITCH MULTIPLEXER
		if((PINC&((1<<PC3)))==0x00)
		{   ch = (mux >> 2);		// MAP CHANNEL
			switch(ch)
			{	case 0 : ch=2;tar = 3-(mux % 4);break;
				case 1 : ch=3;tar = 3-(mux % 4);break;
				case 2 : ch=0;tar = (mux % 4);break;
				case 3 : ch=1;tar = (mux % 4);break;
			}
			
			crossfader[ch].target=tar;
		}
		
		if(debug==1) {
			int j=0;
			for(j = 0 ; j < 16; j++)
			{
				PORTD = PORTD & 0x0F;
				PORTD = PORTD | (j<<4);
				Uart_putc('d');
			}
			
			Uart_puts("\r\n");
		}
		
		
		
		mux = ((mux + 1) % 16);		// INCREASE MUX COUNTER
		cv=ADC_Read(1);			// READ_CV INPUT
		cv=cv-512;
		pos = ADC_Read(0);			// READ AND ADD POSITION POT
		cv = cv + (pos-512);
		
		width = ADC_Read(2);
		
		
		
		
		//NOTE LPF THE ADC
		
		crossfader[0].val = calc(-512);
		if(crossfader[0].val > 1024) LED1_ON; else LED1_OFF;
		
		
		crossfader[1].val = calc(-179);
		if(crossfader[1].val > 1024) LED2_ON; else LED2_OFF;
		
		
		crossfader[2].val = calc(179);
		if(crossfader[2].val > 1024) LED3_ON; else LED3_OFF;
		
		
		crossfader[3].val = calc(512);
		if(crossfader[3].val > 1024) LED4_ON; else LED4_OFF;
		
		
		for(i=0;i<4;i++)
		{   if(crossfader[i].target==0) ACC_A+=crossfader[i].val;
			if(crossfader[i].target==1) ACC_B+=crossfader[i].val;
			if(crossfader[i].target==2) c+=crossfader[i].val;
			if(crossfader[i].target==3) d+=crossfader[i].val;
		}
		
		DAC_write(0,ACC_A);
		DAC_write(1,ACC_B);
		DAC_write(2,c);
		DAC_write(3,d);
		ACC_A=0;ACC_B=0;c=0;d=0;
		
	}
}

