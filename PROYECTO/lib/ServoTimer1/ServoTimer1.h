#ifndef SERVO_TIMER1_H
#define SERVO_TIMER1_H

#include <Arduino.h>

#define MIN_PULSE_WIDTH 	700

#define MAX_PULSE_WIDTH		2300

#define DEFUALT_PULSE_WIDTH	1500

#define PULSE_PERIOD		20000

#define MAX_CHANNELS 		8

typedef struct{
   uint8_t pin;			// pin servo is connected to
   uint8_t isActive;		// checks if channel is active
   uint16_t pulse_width;        // pulse width is defined as 2*ms (DEFUALT_PULSE_WIDTH = 3000)
   				// this adds higher PWM resolution
   uint16_t pulse_end; 		// end of the pulse period
} channel;

class ServoTimer1{
    public:
	ServoTimer1();
	uint8_t attach(uint8_t);
	void write(uint16_t);	
	uint16_t get();
    private:
	uint8_t obj_id;
};



#endif
