#include <atmel_start.h>
#include <pca9517.h>

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	pca9517_setPWMFreq(0x79);
	/* Replace with your application code */
	while (1) { 
	}
}
