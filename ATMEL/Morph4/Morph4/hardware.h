/*
 * hardware.h
 *
 * Created: 06.01.2017 16:24:08
 *  Author: db
 */ 


#ifndef HARDWARE_H_
#define HARDWARE_H_

#include <inttypes.h>

typedef struct fader
{
	uint16_t val;
	uint8_t  target;
	uint8_t  channel;
}fader;

typedef struct poti
{
	uint32_t	smoothed;
	uint16_t    raw;
	uint8_t		adc_channel;
	uint8_t     lpf_beta;
	uint8_t     lpf_fp_precision;
}poti;

void		Uart_init(unsigned int ubrr);
int			Uart_putc(unsigned char c);
void		Uart_puts (char *s);

void		ADC_Init();
uint16_t	ADC_Read( uint8_t channel );

void		SPI_init();
uint8_t		SPI_send(uint8_t data);

void		DAC_write(uint8_t dev_nr,uint16_t val);






#endif /* HARDWARE_H_ */