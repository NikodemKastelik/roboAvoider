#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <inttypes.h>
//#include "lcd_by_Harimaru.h"
#include "macros_by_Harimaru.h"

// <<------------- GLOBAL VARIABLES SECTION ------------->>

volatile uint16_t measurement_1;					// variable used to capture timer on rising edge
volatile uint16_t measurement_2;					// variable used to capture timer on falling edge

volatile uint16_t distance_front =  FRONT_TRIGGER_DISTANCE;		// variable used to measure distance from the obstacles in front of robot
volatile uint16_t distance_left = SIDE_TRIGGER_DISTANCE;		// -||- left
volatile uint16_t distance_right = SIDE_TRIGGER_DISTANCE;		// -||- right

volatile uint8_t flag=2;						// flag which controls proper flow of program -
									// directs which sensor is working at moment
volatile int measurement_finish_flag = 1;				// control flag used in waiting-for-measurement routines in main function




// <<------------------------------------------------------>>

// <<------------- INIT SECTION ------------->>

void mux_4052_init()	// 2:4 multiplexer initialization
{
	BIT_SET(mux_control_ddr, (mux_a_pin | mux_b_pin) );
	BIT_CLEAR(mux_control_port, (mux_a_pin | mux_b_pin) );
}

void timer2_init()		//timer 2 initialized for overflowing - control proper sensor launching
{
	//ASSR |= _BV(AS2);
	TCCR2 |=  _BV(CS21) | _BV(CS22); // prescaler clk/256 => 0.064sec
	TCNT2 = 0;
	TIMSK |= _BV(TOIE2);
}

void timer1_init()		// timer 1 initialized for ICP function
{
	TCCR1A = 0;
	TCCR1B |= _BV(ICNC1) | _BV(ICES1) | _BV(CS10) ;
	TIMSK |= _BV(TICIE1) ;
	TCNT1 = 0;
}


void hcsr04_init()		// ultrasonic sensor initialization
{
	BIT_SET(hcsr04_trigger_ddr, hcsr04_trigger_pin );

	BIT_CLEAR(hcsr04_echo_ddr,hcsr04_echo_pin);

}

void L293D_init()		// motor controller initialization
{
	BIT_SET( motor_ddr , motor_pins);

	BIT_SET( motor_port ,  right_motor_pin_1);
	BIT_SET( motor_port ,  left_motor_pin_1);
}



// <<------------- LCD SECTION ------------->>

// lcd routines used for debugging and troubleshooting

/*void distance_print()
{
	lcd_print(0x80 + 0x40 + 0x00, 0, 0);
	for(int i=0 ; i<16 ; i++) lcd_print(space,1,0);
	lcd_print(0x80 + 0x40 + 0x00, 0, 0);

	lcd_number_print(distance_right);
	lcd_print(space,1,0);
	lcd_number_print(distance_left);
	lcd_print(space,1,0);
	lcd_number_print(distance_front);
}*/

/*
void lcd_update()
{
	if(flag == 0)
	{
		lcd_print(0x80 + 0x40 + 0x00, 0, 0);

		for(int i=0 ; i<4 ; i++) lcd_print(space,1,0);
		lcd_print(0x80 + 0x40 + 0x00, 0, 0);
		//lcd_number_print(PORTB);
		//lcd_number_print(distance_right);
		//flag = 1;
		//measurement_finish_flag = 0;

	}

	else if(flag == 1)
	{
		lcd_print(0x80 + 0x40 + 0x06, 0, 0);

		for(int i=0 ; i<4 ; i++) lcd_print(space,1,0);
		lcd_print(0x80 + 0x40 + 0x06, 0, 0);
		//lcd_number_print(PORTB);
		//lcd_number_print(distance_left);
		//flag = 2;
		//measurement_finish_flag = 0;
	}

	else if(flag == 2)
	{
		lcd_print(0x80 + 0x40 + 0x0B, 0, 0);

		for(int i=0 ; i<4 ; i++) lcd_print(space,1,0);
		lcd_print(0x80 + 0x40 + 0x0B, 0, 0);
		//lcd_number_print(PORTB);
		//lcd_number_print(distance_front);
		//flag = 0;
	}
}
*/

// <<------------------------------------------------------>>

// <<------------- MOTOR CONTROL SECTION ------------->>

void hcsr04_trigger_pulse()				// routine pulsing hc-sr04
{
	BIT_SET( hcsr04_trigger_port , hcsr04_trigger_pin);
	_delay_us(10);
	BIT_CLEAR( hcsr04_trigger_port , hcsr04_trigger_pin);
}

void steer_left(volatile uint16_t *distance , int trigger_distance)
{

	BIT_CLEAR( motor_port , left_motor_pin_1 ); // left wheel reverse rotation
	BIT_SET( motor_port , left_motor_pin_2 );

	while(*distance < trigger_distance)
	{
		asm volatile ("nop");			// spin as long as there is enough space in front of robot
	}


	BIT_SET( motor_port , left_motor_pin_1 ); 	// left wheel rotate normally
	BIT_CLEAR( motor_port , left_motor_pin_2 );
}



void steer_right(volatile uint16_t *distance , int trigger_distance)
{

	BIT_CLEAR( motor_port , right_motor_pin_1 );	// right wheel reverse rotation
	BIT_SET( motor_port , right_motor_pin_2 );

	while(*distance < trigger_distance)
	{
		asm volatile ("nop");			// spin as long as there is enough space in front of robot
	}

	BIT_SET( motor_port , right_motor_pin_1 );	// right wheel rotate normally
	BIT_CLEAR( motor_port , right_motor_pin_2 );
}


// <<------------------------------------------------------>>

// <<------------- INTERRUPT ROUTINES SECTION ------------->>

ISR(TIMER1_CAPT_vect) // Interrupt from Input Capture event - used for pulse width measurement
{

		if( TCCR1B & _BV(ICES1) )		// if bit set then a rising edge has been detected
			{
				measurement_1 = ICR1;	// remember counter value
				BIT_CLEAR(TCCR1B, _BV(ICES1) );		// clear bit in order to detect falling edge next time
			}

			else						// if bit cleared then a falling edge has been detected
			{
				measurement_2 = ICR1;			// remember counter value

				BIT_SET(TCCR1B, _BV(ICES1) );		// set bit in order to detect rising edge next time

				if(flag == 0)				// choose from which sensor we got measurement on basis of 'flag' var
				{
					distance_right =   ( measurement_2 - measurement_1 ) / 58 ;	// calculate distance

					flag = 1;
					measurement_finish_flag = 0;
				}
				else if(flag == 1)
				{
					distance_left =   ( measurement_2 - measurement_1 ) / 58 ;

					flag = 2;
					measurement_finish_flag = 0;
				}
				else if(flag == 2)
				{
					distance_front =   ( measurement_2 - measurement_1 ) / 58 ;

					flag = 0;
				}

			}

}

ISR(TIMER2_OVF_vect) 			// Interrupt from timer2 overflow event -
{					//it provides proper working cycle between sensors: right->left->middle->right etc. and constant time

	BIT_CLEAR( mux_control_port , (mux_a_pin | mux_b_pin) );

	if(flag == 0)
	{

		BIT_SET( mux_control_port , hcsr04_right_mux );		// set multiplexer
		hcsr04_trigger_pulse();					// enable sensor

	}

	else if(flag == 1)
	{

		BIT_SET( mux_control_port , hcsr04_left_mux );
		hcsr04_trigger_pulse();

	}

	else if(flag == 2)
	{

		BIT_SET( mux_control_port , hcsr04_middle_mux );
		hcsr04_trigger_pulse();

	}



}

// <<------------- MAIN SECTION ------------->>

int main()
{
	_delay_ms(1000);			// wait for voltage to stabilise
	//lcd_init();

	hcsr04_init();
	mux_4052_init();
	timer2_init();
	timer1_init();
	L293D_init();

	//lcd_clear();
	//lcd_print_string("RIGHT ");
	//lcd_print_string("LEFT ");
	//lcd_print_string("MIDLE");
	//lcd_print(0x80 + 0x40,0,0);


	sei();					// enable global interrupts

	while(1)
	{

		if(distance_front < FRONT_TRIGGER_DISTANCE)	// check if there is an obstacle in front of robot
		{						// if true check distances from left and right side to decide where to go

			BIT_CLEAR( mux_control_port , (mux_a_pin | mux_b_pin) );

			BIT_SET( mux_control_port , hcsr04_right_mux );		// prepare mux for right sensor
			flag = 0;
			measurement_finish_flag = 1;


			hcsr04_trigger_pulse();

			while( measurement_finish_flag )			// measure right distance
			asm volatile ("nop");

			BIT_CLEAR( mux_control_port , (mux_a_pin | mux_b_pin) );


			BIT_SET( mux_control_port , hcsr04_left_mux );		// prepare mux for left sensor
			flag = 1;
			measurement_finish_flag = 1;

			hcsr04_trigger_pulse();

			while( measurement_finish_flag )			// measure left distance
			asm volatile ("nop");

			BIT_CLEAR( mux_control_port , (mux_a_pin | mux_b_pin) ); // reset mux

			// Decision where to go:

			if(distance_right > distance_left)
			{
				steer_right(&distance_front , TURNING_TRIGGER_DISTANCE);
			}

			else steer_left(&distance_front , TURNING_TRIGGER_DISTANCE);

			distance_left = SIDE_TRIGGER_DISTANCE;		// reset global variables to prevent firing this routine second time from same source
			distance_right = SIDE_TRIGGER_DISTANCE;
		}

		if(distance_left < SIDE_TRIGGER_DISTANCE)		// check if there is an obstacle from left
		{
			steer_right(&distance_left , SIDE_TRIGGER_DISTANCE - 5);	// if true steer right in order to avoid
		}

		else if(distance_right < SIDE_TRIGGER_DISTANCE)		// check if there is an obstacle from right
		{
			steer_left(&distance_right , SIDE_TRIGGER_DISTANCE - 5);	// if true steer left in order to avoid
		}



	}

	return 0;
}
