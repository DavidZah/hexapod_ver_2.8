/*
 * pca9517.h
 *
 * Created: 27.12.2018 18:56:45
 *  Author: David
 */ 


#ifndef PCA9517_H_
#define PCA9517_H_


#include "driver_init.h"


#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD


void pca9517_setPWMFreq(uint8_t freq);
void pca9517_setPWM(struct i2c_m_sync_desc *i2c , uint8_t num, uint16_t on, uint16_t off);





#endif /* PCA9517_H_ */