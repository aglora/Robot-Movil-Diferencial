#include "ServoTimer1.h"

static void initInt();
static uint16_t et();
static void statwrite(uint8_t,uint16_t);

channel s[4];
uint8_t c_size = 0;
uint8_t c_index = 0;
uint16_t c_init = 1000;
bool start = false;

uint16_t pulses[2];
uint16_t tmp;


SIGNAL(TIMER1_COMPA_vect){
    if(c_size <= 0) return;
    tmp = (OCR1AH<<8|OCR1AL);
    if(c_index >= c_size){
	PORTD &= ~(_BV(s[c_index-1].pin));
    	TCNT1H = 0x00;
    	TCNT1L = 0x00;
	c_index = 0;
	tmp = (uint16_t)PULSE_PERIOD*2 - tmp + c_init;
	OCR1AH = tmp>>8;
	OCR1AL = tmp;
	c_init = (OCR1AH<<8)|OCR1AL;
    }else if(c_index <= 0){
    	PORTD |= _BV(s[c_index].pin);
	tmp += s[c_index].pulse_width;
	OCR1AH = tmp>>8;
	OCR1AL = tmp;
	c_index++;
    }else{
    	PORTD |= _BV(s[c_index].pin);
	PORTD &= ~(_BV(s[c_index-1].pin));
	tmp += s[c_index].pulse_width;
	OCR1AH = tmp>>8;
	OCR1AL = tmp;
	c_index++;
    }
    pulses[c_index] = OCR1AH<<8|OCR1AL;
}

/*
________|||||||_________________	channel 0
_______________|||||||__________	channel 1
______________________|||||||___	channel 2
	      ...			
	      ...			etc
*/
ServoTimer1::ServoTimer1(){
    if(c_size < MAX_CHANNELS){
	this->obj_id = c_size++;
    }else{
    	this->obj_id = -1;
    }
    Serial.print("init");
}

uint8_t ServoTimer1::attach(uint8_t pin){
    if(!start) initInt();
    DDRD |= _BV(pin);
    s[this->obj_id].pin = pin;
    s[this->obj_id].pulse_width = (uint16_t)DEFUALT_PULSE_WIDTH*2;
}

static void initInt(){
    TIMSK1 = 0x00;		//turn off interrupts
    TCCR1A = 0x00;		//set counter mode
    TCCR1B = 0x02;		//set counter mode and prescale
    OCR1AH = c_init>>8;		//high 8bits of first period width
    OCR1AL = c_init&0xff;		//low 8bits of first period width
    TIMSK1 |= 0x02;		//turn on ocr1a interrupts
    start = true;
}

uint16_t ServoTimer1::get(){
	return pulses[1];//OCiR1AH<<8|OCR1AL;
}

static uint16_t et(){
	return pulses[0];
}

void ServoTimer1::write(uint16_t pulse_width){
//    s[this->obj_id].pulse_width = pulse_width*2;
	statwrite(this->obj_id, pulse_width);
}

static void statwrite(uint8_t index, uint16_t pulse_width){
	s[index].pulse_width = pulse_width*2;
}
