/*
 * hardware.c
 *
 * Created: 06.01.2017 16:27:04
 *  Author: db
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>


#define SELECT_DAC1 PORTB &= ~(1<<PB0)
#define DESELECT_DAC1 PORTB |= (1<<PB0)
#define SELECT_DAC2 PORTB &= ~(1<<PB1)
#define DESELECT_DAC2 PORTB |= (1<<PB1)


void Uart_init(unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<UCSZ00) | (1 << UCSZ01);
}



int Uart_putc(unsigned char c)
{
	while (!(UCSR0A & (1<<UDRE0))) 	{
	}
	UDR0 = c;            
	//_delay_ms(1000);          
	return 0;
}

void Uart_puts (char *s)
{
	while (*s)
	{   
		Uart_putc(*s);
		s++;
	}
}

void ADC_Init()
{
	
	ADMUX = (1<<REFS1) | (1<<REFS0);
	ADCSRA = (1<<ADPS1) | (1<<ADPS0);
	ADCSRA |= (1<<ADEN);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) {
	}
	(void) ADCW;
}

uint16_t ADC_Read( uint8_t channel )
{
	ADMUX = (channel & 0xFF);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) {
	}
	return ADCW;
}

void SPI_init(){

	DDRB |= (1<<PB5);
	DDRB &= ~(1<<PB4);
	DDRB |= (1<<PB3);  //MOSI
	DDRB |= (1<<PB2);  //SS_ENABLE
	SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

uint8_t SPI_send(uint8_t data){

	SPDR = data;
	while (!(SPSR & (1<<SPIF)));
	return SPDR;
}

void DAC_write(uint8_t dev_nr,uint16_t val)
{
	switch(dev_nr)
	{
		case 0 :   DESELECT_DAC2;SELECT_DAC1;SPI_send(0b11010000|(val>>8));SPI_send(val);DESELECT_DAC1;break;
		case 1 :   DESELECT_DAC2;SELECT_DAC1;SPI_send(0b01010000|(val>>8));SPI_send(val);DESELECT_DAC1;break;
		case 2 :   DESELECT_DAC1;SELECT_DAC2;SPI_send(0b11010000|(val>>8));SPI_send(val);DESELECT_DAC2;break;
		case 3 :   DESELECT_DAC1;SELECT_DAC2;SPI_send(0b01010000|(val>>8));SPI_send(val);DESELECT_DAC2;break;
	}
}

