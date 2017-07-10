#define F_CPU 16000000UL //16MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//#define PUMP_PIN		   6
//#define PUMP_PWM  		   OCR0A
//#define ELECTRIC_VALVE_PIN     1
//#define ELECTRIC_VALVE_PWM     OCR0B

#define WELD_WIRE_ENGINE_DDR	 DDRD
#define WELD_WIRE_ENGINE_PORT    PORTD
#define WELD_WIRE_ENGINE_PIN     PORTD5
#define WELD_WIRE_ENGINE_PWM     OCR1A


#define ELECTRIC_VALVE_CONTROL_DDR	DDRB
#define ELECTRIC_VALVE_CONTROL_PORT	PORTB
#define ELECTRIC_VALVE_CONTROL_PIN	PORTB2


#define WELD_CURRENT_ON_DDR		DDRB
#define WELD_CURRENT_ON_PORT	PORTB
#define WELD_CURRENT_ON_PIN		PORTB3


#define WELD_TYPE_SWITCH_DDR	DDRC
#define WELD_TYPE_SWITCH_PORT	PORTC
#define WELD_TYPE_SWITCH_PINS	PINC
#define WELD_TYPE_SWITCH_PIN	PINC2


#define WELD_WIRE_KEY_DDR		DDRC
#define WELD_WIRE_KEY_PORT		PORTC
#define WELD_WIRE_KEY_PINS		PINC
#define WELD_WIRE_KEY_PIN		PINC3

#define PURGE_TIME_MS	2000

uint16_t adc_result;
uint16_t purge_timer;

typedef enum
{
	STATE_STOP=0,
	STATE_SIMPLE_WELD,
	STATE_PURGE_START,
	STATE_PURGE_WAIT,
	STATE_WIRE_WELD,
}enStateMachine;

enStateMachine StateMachine;

void pin_init(void)
{
	WELD_WIRE_ENGINE_DDR |= (1<<WELD_WIRE_ENGINE_PIN);
	WELD_WIRE_ENGINE_PORT &= ~(1<<WELD_WIRE_ENGINE_PWM);

	ELECTRIC_VALVE_CONTROL_DDR |= (1<<ELECTRIC_VALVE_CONTROL_PIN);
	ELECTRIC_VALVE_CONTROL_PORT&=~(1<<ELECTRIC_VALVE_CONTROL_PIN);

	WELD_WIRE_KEY_DDR&=~(1<<WELD_WIRE_KEY_PIN);
	WELD_WIRE_KEY_PORT|=(1<<WELD_WIRE_KEY_PIN);

	WELD_TYPE_SWITCH_DDR&=(1<<WELD_TYPE_SWITCH_PIN);
	WELD_TYPE_SWITCH_PORT|=(WELD_TYPE_SWITCH_PIN);
}

void timer0_init(void)
{
	TCCR0A |= (1 << COM0A1)|((1 << COM0B1));
	// set none-inverting mode

	TCCR0A |= /*(1 << WGM01) |*/ (1 << WGM00);
	// set fast PWM Mode

	TCCR0B |= (1 << CS01)|(1 << CS00);


	TCNT0 = 0x00;
	//    ICR0 = 0xFF;
	OCR0A = 0x00;
	OCR0B = 0x00;
}

//void timer1_init(void)
//{
	//TCCR1A |= (1 << COM1A1) /*| (1 << WGM11)*/| (1 << WGM10);
	//TCCR1B |= /*(1 << WGM13) | (1 << WGM12) |*/ (1 << CS11)|(1 << CS10);
	//TCNT1 = 0x00;
	//ICR1 = 0xFF;
	//OCR1A = 0x00;
//}



void adc_init(void)
{
	ADMUX|=(1<<REFS0)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0);//REF internal
	ADCSRA|=(1<<ADEN)|(1<<ADIE)|(1<<ADSC);//enable, free running cycle, interrupt enable
	ADCSRA|=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);//prescaler ADC=128
}

static uint8_t WELD_TYPE_SWITCH_PIN_prev=1;
static uint8_t WELD_WIRE_KEY_PIN_prev=1;

void keys_read(void)
{	
	if((WELD_TYPE_SWITCH_PINS&(1<<WELD_TYPE_SWITCH_PIN))!=WELD_TYPE_SWITCH_PIN_prev)
	{
		_delay_ms(20);
		if((WELD_TYPE_SWITCH_PINS&(1<<WELD_TYPE_SWITCH_PIN))!=WELD_TYPE_SWITCH_PIN_prev)
		{
			WELD_TYPE_SWITCH_PIN_prev=(WELD_TYPE_SWITCH_PINS&(1<<WELD_TYPE_SWITCH_PIN));
			if((WELD_TYPE_SWITCH_PINS&(1<<WELD_TYPE_SWITCH_PIN))==0)
			{
				
			}
			else
			{

			}
		}
	}
	
	if((WELD_WIRE_KEY_PINS&(1<<WELD_WIRE_KEY_PIN))!=WELD_WIRE_KEY_PIN_prev)
	{
		_delay_ms(20);
		if((WELD_WIRE_KEY_PINS&(1<<WELD_WIRE_KEY_PIN))!=WELD_WIRE_KEY_PIN_prev)
		{
			WELD_WIRE_KEY_PIN_prev=(WELD_WIRE_KEY_PINS&(1<<WELD_WIRE_KEY_PIN));
			if((WELD_WIRE_KEY_PINS&(1<<WELD_WIRE_KEY_PIN))==0)
			{
		
			}
			else
			{
		
			}
		}
	}


}

void StateMachineCycle(void)
{
	switch(StateMachine)
	{
		case STATE_STOP:
		{
			WELD_WIRE_ENGINE_PWM=0;	//stop wire
			PORTB&=~(1<<ELECTRIC_VALVE_CONTROL_PIN);//close valve
			PORTB&=~(1<<WELD_CURRENT_ON_PIN);//current off
		}
		break;

		case STATE_SIMPLE_WELD:
		{
			WELD_WIRE_ENGINE_PWM=0;	//stop wire
			PORTB&=~(1<<ELECTRIC_VALVE_CONTROL_PIN);//close valve
			PORTB|=(1<<WELD_CURRENT_ON_PIN);//current on
		}
		break;

		case STATE_PURGE_START:
		{
			WELD_WIRE_ENGINE_PWM=0;	//stop wire
			PORTB&=~(1<<WELD_CURRENT_ON_PIN);//current off
			PORTB|=(1<<ELECTRIC_VALVE_CONTROL_PIN);//open valve
			StateMachine=STATE_PURGE_WAIT;
			purge_timer=PURGE_TIME_MS;
		}
		break;

		case STATE_PURGE_WAIT:
		{
			PORTB|=(1<<ELECTRIC_VALVE_CONTROL_PIN);//open valve
			
			if((purge_timer-=10)==0)//wait a time
			{
				StateMachine=STATE_WIRE_WELD;
			}			
		}
		break;

		case STATE_WIRE_WELD:
		{
			WELD_WIRE_ENGINE_PWM=(adc_result>>2);//run wire
			PORTB|=(1<<ELECTRIC_VALVE_CONTROL_PIN);//open valve
			PORTB|=(1<<WELD_CURRENT_ON_PIN);//current on
		}
		break;
	}
}

int main(void)
{
	StateMachine=STATE_STOP;

	pin_init();
	timer0_init();
	adc_init();

	WELD_WIRE_ENGINE_PWM=0;
	sei();
//	timer1_init();
	while(1)
	{
		_delay_ms(10);
		keys_read();
		StateMachineCycle();
	}
}


ISR(ADC_vect)
{
	if(!(ADMUX&(1<<ADLAR)))
	{
		adc_result=ADCL;
		adc_result|=ADCH<<8;
	}
	else
	{
		adc_result=ADCL>>6;
		adc_result|=ADCH<<2;
	}
	ADCSRA|=0x40;
}