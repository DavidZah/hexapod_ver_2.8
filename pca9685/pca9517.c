/*
 * pca9517.c
 *
 * Created: 27.12.2018 18:56:28
 *  Author: David
 */ 



/* 
    @brief  Setups the I2C interface and hardware
*/

#include "pca9517.h"
/**************************************************************************/
/*! 
    @brief  Sends a reset command to the PCA9685 chip over I2C
*/
/**************************************************************************/
void pca9517_reset() {
	
	
  //write8(PCA9685_MODE1, 0x80);
  uint8_t buffer = 0x80; 
  
}

/**************************************************************************/
/*! 
    @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
    @param  freq Floating point frequency that we will attempt to match
*/
/**************************************************************************/
void pca9517_setPWMFreq(uint8_t freq) {
	
	struct io_descriptor *I2C_0_io;
	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	i2c_m_sync_set_slaveaddr(&I2C_0, 0x40,I2C_M_SEVEN); 
	
	uint8_t buffer[10]; 
	buffer[0] = PCA9685_MODE1; 
	buffer[1] = 0x18; 
	io_write(I2C_0_io,buffer,2); 
	buffer[0] = PCA9685_PRESCALE;
	buffer[1] = 0x79;  
	io_write(I2C_0_io,buffer,2);
	io_write(I2C_0_io,buffer,1); 
	io_read(I2C_0_io,buffer,1); 
	
	
}

/**************************************************************************/
/*! 
    @brief  Sets the PWM output of one of the PCA9685 pins
    @param  num One of the PWM output pins, from 0 to 15
    @param  on At what point in the 4096-part cycle to turn the PWM output ON
    @param  off At what point in the 4096-part cycle to turn the PWM output OFF
*/
/**************************************************************************/
void pca9517_setPWM(struct i2c_m_sync_desc *i2c ,uint8_t num, uint16_t on, uint16_t off) {
	uint8_t buffer[5]; 
	
	buffer[0] = LED0_ON_L+4*num;
	buffer[1] = on; 
	buffer[2] = on >> 8;
	buffer[3] = off; 
	buffer[4] = off>>8; 
	
	/*
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(LED0_ON_L+4*num);
  _i2c->write(on);
  _i2c->write(on>>8);
  _i2c->write(off);
  _i2c->write(off>>8);
  _i2c->endTransmission();
  */
}

/**************************************************************************/
/*! 
    @brief  Helper to set pin PWM output. Sets pin without having to deal with on/off tick placement and properly handles a zero value as completely off and 4095 as completely on.  Optional invert parameter supports inverting the pulse for sinking to ground.
    @param  num One of the PWM output pins, from 0 to 15
    @param  val The number of ticks out of 4096 to be active, should be a value from 0 to 4095 inclusive.
    @param  invert If true, inverts the output, defaults to 'false'
*/
/**************************************************************************/
/*
void pca9517_setPin(struct i2c_m_sync_desc *i2c,uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, (uint16_t)4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      pca9517_setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      pca9517_setPWM(num, 0, 4096);
    }
    else {
      pca9517_setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      pca9517_setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      pca9517_setPWM(num, 0, 4096);
    }
    else {
      pca9517_setPWM(num, 0, val);
    }
  }
}
*/