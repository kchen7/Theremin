/*
 * Theremin.c
 *
 * Created: 2/13/2019 5:37:09 PM
 * Author : kevin
 */ 

#include <avr/io.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// UART /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

char String[64] = "";

void USART_init(void) {
	/*Set baud rate */
	UBRR0H = (unsigned char)(BAUD_PRESCALLER>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALLER;
	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_send( unsigned char data) {
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void USART_putstring(char* StringPtr) {
	while(*StringPtr != 0x00) {
		USART_send(*StringPtr);
		StringPtr++;
	}
}
////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// END UART ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

volatile char isHigh;

volatile char isStart; 
volatile unsigned short start; 
volatile unsigned short end; 
volatile unsigned long pulseWidth;

volatile char isContinuous; 
volatile char isPressed; 

ISR(TIMER1_COMPA_vect) {
	if (isHigh) {
		OCR1A = TCNT1 + 16000; // 1000 us low pulse 
		isHigh = 0; 
// 		TIMSK1 = 0; // disables after one pulse
// 		TCCR1A = 0; // disables after one pulse
	} else {
		OCR1A = TCNT1 + 160; // 10 us high pulse 
		isHigh = 1; 
	}
}

ISR(TIMER1_CAPT_vect) {
	if (isStart) {
		start = ICR1; 
		isStart = 0; 
	} else {
		end = ICR1; 
		isStart = 1; 
	}
	TCCR1B ^= (1 << ICES1); // toggle edge detect 
}

void init() {
	
	DDRB |= (1 << PORTB1); // PB1 is output (pin 9) 
	DDRD |= (1 << PORTD6); // PD6 is buzzer output (pin 6) 
	DDRB |= (1 << PORTB2); // set up these registers for ADC volume control 
	DDRB |= (1 << PORTB3);
	DDRB |= (1 << PORTB4);

// 	DDRD |= (1 << PORTD0); // PD0 for tests 
		
	// rangefinder setup	
	TCCR1A |= (1 << COM1A0); // set output compare mode to toggle
	TCCR1B |= (1 << CS10); // start timer without prescale	
	TIMSK1 |= (1 << OCIE1A); // enable Output Compare Match A
	
	TCCR1B |= (1 << ICES1); // set input capture as detect rising edge
	TIMSK1 |= (1 << ICIE1); // enable input capture interrupt 
	
	// buzzer setup
	TCCR0A |= (1 << COM0A0); // set output compare mode to toggle
	TCCR0A |= (1 << WGM01); // set to CTC mode
	TCCR0B |= (1 << CS01); //scale timer to 64
	TCCR0B |= (1 << CS00); 
	TCNT0 = 0; // initialize timer to 0
	OCR0A = 119; // initialize buzzer at far frequency
	
	// photoresistor ADC setup 
	ADMUX |= (1 << REFS0); // set Avcc as voltage reference
	// by default, reading from ADC0, set using ADMUX
	// by default, in free running mode, set using ADCSRB 
	DIDR0 |= (1 << ADC0D); // disable digital input buffer of pin 0
	ADCSRA = (1 <<ADEN) | (1 << ADPS2) | (1 <<ADPS1); // ADEN to enable ADC, set ADC clock division factor to 64
	ADCSRA |= (1 << ADATE); // ADC autotrigger 
	ADCSRA |= (1 << ADSC); // start conversion 
	
	sei(); 
	
	isHigh = 0; 
	isStart = 1;
	isContinuous = 1; 
	isPressed = 0; 
	OCR1A = TCNT1 + 16; // pull output high to start process 
}

unsigned long continuousFrequency(unsigned long pulse) {
	double pulseRange = 60000 - 5500; 
	double CTCRange = 119 - 60; 
	
	return (unsigned long) (pulse*CTCRange / pulseRange + 60); 
}

unsigned long discreteFrequency(unsigned long pulse) {
	if (pulse < 12313) {
		return 119; 
	} else if (pulse < 19125) {
		return 106; 
	} else if (pulse < 25938) {
		return 95; 
	} else if (pulse < 32750) {
		return 89; 
	} else if (pulse < 39563) {
		return 80; 
	} else if (pulse < 46375) {
		return 71;
	} else if (pulse < 53188) {
		return 63; 
	} else {
		return 60; 
	}
}

void lightSensorToVolume(unsigned int light) {
	//900 darkness, 530 ambient, 50 flashlight 
	if (light < 576) {
		// ambient light = loudest (flashlight range not used) 
		PORTB |= (1<< PORTB4); // MSB
		PORTB |= (1<< PORTB3); 
		PORTB |= (1<< PORTB2); // LSB		
	} else if (light < 623) {
		PORTB |= (1<< PORTB4); // MSB
		PORTB |= (1<< PORTB3);
		PORTB &= ~(1<< PORTB2); // LSB
	} else if (light < 669) {
		PORTB |= (1<< PORTB4); // MSB
		PORTB &= ~(1<< PORTB3);
		PORTB |= (1<< PORTB2); // LSB
	} else if (light < 715) {
		PORTB |= (1<< PORTB4); // MSB
		PORTB &= ~(1<< PORTB3);
		PORTB &= ~(1<< PORTB2); // LSB
	} else if (light < 761) {
		PORTB &= ~(1<< PORTB4); // MSB
		PORTB |= (1<< PORTB3);
		PORTB |= (1<< PORTB2); // LSB
	} else if (light < 808) {
		PORTB &= ~(1<< PORTB4); // MSB
		PORTB |= (1<< PORTB3);
		PORTB &= ~(1<< PORTB2); // LSB
	} else if (light < 854) {
		PORTB &= ~(1<< PORTB4); // MSB
		PORTB &= ~(1<< PORTB3);
		PORTB |= (1<< PORTB2); // LSB
	} else { // darkness
		// darkness is softest 
		PORTB &= ~(1<< PORTB4); // MSB
		PORTB &= ~(1<< PORTB3);
		PORTB &= ~(1<< PORTB2); // LSB
	}
	
}

int main(void) {
 	USART_init(); 
	init();
			
	while (1) {
 		sprintf(String,"%u \n", ADC); 
 		USART_putstring(String);
		
		// button pressing with debouncing 
		if (!isPressed) {
			if (PIND & (1 << PORTD7)) {
				isContinuous = !isContinuous;
				isPressed = 1;
			}
		} else {
			if (!(PIND & (1 << PORTD7))) {
				isPressed = 0;
			}
		}
		
		// determine pulse width 
		if (isStart) {
			if (end > start) {
				pulseWidth = (end - start);
			} else {
				pulseWidth = (65535u - start + end);
			}
		}
		
		// determine volume from light sensor ADC
		lightSensorToVolume(ADC); 
		
		// play music 
		if (isContinuous) {
			OCR0A = continuousFrequency(pulseWidth);
		} else {
			OCR0A = discreteFrequency(pulseWidth);
		}
	}
}

