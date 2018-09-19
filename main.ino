#include <SoftwareSerial.h>
#include <Servo.h>


// Left Joystick Pins
#define LEFT_JOY_X A0
#define LEFT_JOY_Y A1

// Right Joystick Pins
#define RIGHT_JOY_X A2
#define RIGHT_JOY_Y A3

// Stepper motor pins
#define T_STEP 0b00000010 //9
#define T_DIR 0b00000001 //8

#define R_STEP 0b00001000 //11
#define R_DIR 0b00000100 //10

#define Z_STEP 0b00100000 //13
#define Z_DIR 0b00010000 //12

// Servo pin
#define SERVO_PIN 6

// Speed thresholds for axes
#define z_thresh 2
#define r_thresh 2
#define t_thresh 2

// Speeds and delays for stepper motors
int z_speed;
int r_speed;
int t_speed;

unsigned long z_last = 0;
unsigned long r_last = 0;
unsigned long t_last = 0;

unsigned long z_next = 0;
unsigned long r_next = 0;
unsigned long t_next = 0;

/* Variables to hold the states of the digital ports
  * each bit represents the state of a port
  * least significant bit is lowest port number
  * most  significant bit is highest port number
*/
unsigned char ports_0_7 = 0b00000000;
unsigned char ports_8_13 = 0b00000000;

// Jostick read timer
unsigned long last_check = 0;

// Endstop communication
#define ENDSTOP 7
#define COM_TX 50 //won't be used
SoftwareSerial mySerial(ENDSTOP, COM_TX);

// Endstop variables
bool z_stop;
bool r_stop;
int z_endstop_dir = 1;
int r_endstop_dir = -1;

// Servo
#define SERVO 6
Servo gripper;
int servo_pos = 130;

void setup()
{
	/* Set the direction register for digital ports 0 - 7
	  * 1 sets to OUTPUT - 0 sets to INPUT
	  * 6 set to OUTPUT for servo
	*/
	DDRD = DDRD | 0b01000000;

	// Set the direction register for digital ports 8 - 13
	DDRB = DDRB | 0b00111111;

	// Begins the Software Serial on port 7 for endstop detection
	mySerial.begin(9600);

	// Setup the servo and move to starting position
	gripper.attach(SERVO);
	gripper.write(servo_pos);
}

void loop()
{
	// Check joysticks every 1/10th of a second
	if ((unsigned long)(millis() - last_check) >= 100)
	{
		getRSpeed();
		getZSpeed();
		getTSpeed();

		last_check = millis();
	}
	
	checkEndstops();

	if (abs(r_speed) > r_thresh) step_R(r_speed); //(abs(r_speed) - r_thresh) * ((int)(r_speed > 0) * 2 - 1)); //TODO: simplify
	if (abs(z_speed) > z_thresh) step_Z(z_speed);
	if (abs(t_speed) > t_thresh) step_T(t_speed);
}


/* Helper functions for the step function
  * allows it to be used for all three motors
  * the relative speeds of the motors can be set with the last argument
*/
void step_R(int speed)
{
	step(r_stop, R_STEP, R_DIR, r_endstop_dir, r_last, r_next, speed, 1000);
}
void step_Z(int speed)
{
	step(z_stop, Z_STEP, Z_DIR, z_endstop_dir, z_last, z_next, speed, 1000);
}
void step_T(int speed)
{
	step(false, T_STEP, T_DIR, 0, t_last, t_next, speed, 4000);
}


void step(bool endstop, int step_pin, int dir_pin, int endstop_dir,
	unsigned long& last_action, unsigned long& next_action, int speed, int delay)
{
	if (endstop)
	{
		if (speed * endstop_dir > 0) return; //same sign - cannot move that way
	}

	if ((unsigned long)(millis() - last_action) > next_action)
	{
		/* Write to the direction pin
		  * zero out the bit corresponnding to the pin
		  * replace its value with the direction
		    * determined by the sign of the speed
		*/		
		unsigned char byte = PORTB & (~dir_pin);
		PORTB = byte | (dir_pin * (int)(speed > 0));

		/* Write to the step pin
		  * flip the bit corresponding to the pin
		  * creates a square wave
		*/
		PORTB = PORTB ^ step_pin;

		last_action = millis();
		next_action = delay / abs(speed); //TODO: test timing
	}
}

void getRSpeed()
{
	r_speed = map(analogRead(RIGHT_JOY_Y), 1023, 0, -10, 10);
}

void getZSpeed()
{
	z_speed = map(analogRead(LEFT_JOY_Y), 0, 1023, 10, -10);
}

void getTSpeed()
{
	t_speed = map(analogRead(LEFT_JOY_X), 0, 1023, -10, 10);
}

void checkEndstops()
{
	if (mySerial.available())
	{
		int byte = mySerial.read();
    
		if (byte == 'R') r_stop = true;
		else if (byte == 'Z') z_stop = true;
		else if (byte == '&') r_stop = z_stop = true;
		else if (byte == '0') r_stop = z_stop = false;
	}
}
