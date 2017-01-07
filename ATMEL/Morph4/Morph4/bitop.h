/*
 * bitop.h
 *
 * Created: 06.01.2017 16:12:54
 *  Author: db
 */ 


#ifndef BITOP_H_
#define BITOP_H_

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




#endif /* BITOP_H_ */