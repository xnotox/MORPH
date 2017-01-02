#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>


//EARLY STAGE ; SEEMS TO WORK ; NOT OPTIMIZED

#ifndef F_CPU
    #define F_CPU 2000000UL  
#endif


#define _BV(a) ((1 << a))

#define SELECT_DAC1 PORTB &= ~(1<<PB0)
#define DESELECT_DAC1 PORTB |= (1<<PB0)

#define SELECT_DAC2 PORTB &= ~(1<<PB1)
#define DESELECT_DAC2 PORTB |= (1<<PB1)

int16_t x=0;
uint8_t mux=0;
uint16_t a,b,c,d;
uint8_t i;
uint8_t ch,tar;
uint8_t pstate;
uint16_t width,pos;


typedef struct fader
{
   uint16_t val;
   uint8_t  target;
}fader;

fader crossfader[4];
    
// BIT_MANIPULATION 
//
static inline void BIT_SET8(volatile uint8_t *target, uint8_t bit) __attribute__((always_inline));
static inline void BIT_SET8(volatile uint8_t *target, uint8_t bit){
	*target |= (1<<bit);
};

static inline void BIT_CLR8(volatile uint8_t *target, uint8_t bit) __attribute__((always_inline));
static inline void BIT_CLR8(volatile uint8_t *target, uint8_t bit){
   *target &= ~(1<<bit);
};

static inline void BIT_TGL8(volatile uint8_t *target, uint8_t bit) __attribute__((always_inline));
static inline void BIT_TGL8(volatile uint8_t *target, uint8_t bit){
	*target ^= (1<<bit);
};

// SIMPLE_MATH
uint16_t abs_val(int16_t x)
{
   if(x < 0) return -x;
   else return x;
   }
   
uint16_t rectify(int16_t x)
{
   if(x< 0) return 0;
   else return x;
   }
   
// HARDWARE INITIAlIZATIOPM
   
void ADC_Init(void)
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

void SYSTEM_init()
{
    ADC_Init();
    crossfader[0].target=0;
    crossfader[0].target=1;
    crossfader[0].target=2;
    crossfader[0].target=3;
    DDRB |= (1<<PB0); // chip select for DAC1
    DDRB |= (1<<PB1); // chip select for DAC2
    DDRD = 0xFF;
    sei(); 
    DESELECT_DAC1;
    DESELECT_DAC2;
    SPI_init();
   
}



int main(void){

    SYSTEM_init();
   
  while (1) {
     
     
     PORTD = PORTD & 0x0F;
     PORTD = PORTD | (mux<<4);  	// SWITCH MULTIPLEXER
     pstate=(PINC&_BV(PC3));		// FETCH MUX OUTPUT
     if(pstate==0x00)
     {   ch = (mux / 4);		// MAP CHANNEL
	 switch(ch)
	  { case 0 : ch=2;break;
	    case 1 : ch=3;break;
	    case 2 : ch=0;break;
	    case 3 : ch=1;break;
	  }
	 tar = (mux % 4);		// SET TARGET ACCORDING TO SWITCH
	 crossfader[ch].target=tar;
      }
     mux = ((mux + 1) % 16);		// INCREASE MUX COUNTER
     x=ADC_Read(1);			// READ_CV INPUT
					  
     pos = ADC_Read(0);			// READ AND ADD POSITION POT
     x = x + (pos-512);
     
     width = ADC_Read(2)>>3;
     width=339-width;
    
     
     
     crossfader[0].val = (uint16_t)((rectify((width-abs_val(x - 0))))<<3);
     if(crossfader[0].val > 1024) BIT_SET8(&PORTD,0); else BIT_CLR8(&PORTD,0);
     crossfader[1].val = (uint16_t)((rectify((width-abs_val(x - 341))))<<3);
     if(crossfader[1].val > 1024) BIT_SET8(&PORTD,1); else BIT_CLR8(&PORTD,1);
     crossfader[2].val = (uint16_t)((rectify((width-abs_val(x - 682))))<<3);
     if(crossfader[2].val > 1024) BIT_SET8(&PORTD,2); else BIT_CLR8(&PORTD,2);
     crossfader[3].val = (uint16_t)((rectify((width-abs_val(x - 1023))))<<3);
     if(crossfader[3].val > 1024) BIT_SET8(&PORTD,3); else BIT_CLR8(&PORTD,3);
     for(i=0;i<4;i++)
     { if(crossfader[i].target==0) a+=crossfader[i].val;
	if(crossfader[i].target==1) b+=crossfader[i].val;
	   if(crossfader[i].target==2) c+=crossfader[i].val;
	      if(crossfader[i].target==3) d+=crossfader[i].val;

      }
     DAC_write(0,a);
     DAC_write(1,b);
     DAC_write(2,c);
     DAC_write(3,d);
     a=0;b=0;c=0;d=0;
  
  }
}

