#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    powerE,         sensorAnalog)
#pragma config(Sensor, dgtl1,  ultrasonic,     sensorSONAR_cm)
#pragma config(Sensor, I2C_1, driveLeftEncoder ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2, driveRightEncoder ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_3, shootRightEncoder ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_4, shootLeftEncoder ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           up,            tmotorVex393, openLoop)
#pragma config(Motor,  port2,           left1,         tmotorVex393, openLoop, encoder, encoderPort, I2C_4, 1000)
#pragma config(Motor,  port3,           left2,         tmotorVex393, openLoop)
#pragma config(Motor,  port4,           driveLeft2,    tmotorVex393, openLoop)
#pragma config(Motor,  port5,           driveLeft,     tmotorVex393, openLoop, encoder, encoderPort, I2C_1, 1000)
#pragma config(Motor,  port6,           driveRight,    tmotorVex393, openLoop, reversed, encoder, encoderPort, I2C_2, 1000)
#pragma config(Motor,  port7,           driveRight2,   tmotorVex269, openLoop, reversed)
#pragma config(Motor,  port8,           right1,        tmotorVex393, openLoop, reversed, encoder, encoderPort, I2C_3, 1000)
#pragma config(Motor,  port9,           right2,        tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port10,          intake,        tmotorVex393, openLoop, reversed)
#pragma platform(VEX)


#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

// Update inteval (in mS) for the flywheel control loop
#define FW_LOOP_SPEED              25

// Maximum power we want to send to the flywheel motors
#define FW_MAX_POWER              127

// encoder counts per revolution depending on motor
#define MOTOR_TPR_269           240.448
#define MOTOR_TPR_393R          261.333
#define MOTOR_TPR_393S          392
#define MOTOR_TPR_393T          627.2
#define MOTOR_TPR_QUAD          360.0

// encoder tick per revolution
float           ticks_per_rev;          ///< encoder ticks per revolution

// Encoder
long            encoder_countsR;         ///< current encoder count
long            encoder_counts_lastR;    ///< current encoder count
long            encoder_countsL;
long            encoder_counts_lastL;

// velocity measurement
float           motor_velocityR;         ///< current velocity in rpm
float           motor_velocityL;
long            nSysTime_lastR;          ///< Time of last velocity calculation
long            nSysTime_lastL;

// TBH control algorithm variables
long            target_velocityR;        ///< target_velocity velocity
long            target_velocityL;
float           current_errorR;          ///< error between actual and target_velocity velocities
float           current_errorL;          ///< error between actual and target_velocity velocities
float           last_errorR;             ///< error last time update called
float           last_errorL;
float           gain;                   ///< gain
float           driveR;                  ///< final drive out of TBH (0.0 to 1.0) RIGHT
float           driveL;                  /// LEFT
float           drive_at_zeroR;          ///< drive at last zero crossing RIGHT
float           drive_at_zeroL;          ///LEFT
long            first_crossR;            ///< flag indicating first zero crossing RIGHT
long            first_crossL;            /// LEFT
float           drive_approxR;           ///< estimated open loop drive RIGHT
float           drive_approxL;           ///LEFT

// final motor drive
long            motor_driveR;            ///< final motor control value RIGHT
long            motor_driveL;            /// LEFT

int a=35; // startup speed / slowdown speed
int deadzone=15; // Deadzone for joysticks
int b=43; // main shooting speed
float driveTrainSpeedControl = 1;
int sonarValue = 0;
float powerExpanderBatteryV = 2;

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
	// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.


	nMotorEncoder[driveLeftEncoder] = 0;
	nMotorEncoder[driveRightEncoder] = 0;
	nMotorEncoder[shootLeftEncoder] = 0;
	nMotorEncoder[shootRightEncoder] = 0;
	bStopTasksBetweenModes = true;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

task autonomous()
{
	motor[left1] =  a;
	motor[left2] = a;
	motor[right1] = a;
	motor[right2] = a;
	wait1Msec(1000);
	motor[left1]= b;
	motor[left2]= b;
	motor[right1]= b;
	motor[right2]= b;
	wait1Msec(3000);
	motor[up] = 75;
	motor[intake] = 75;
	wait1Msec(3000);
	motor[up] = 50;



}



/*Set the flywheen motors RIGHT  */
void
FwMotorSetR( int valueR )
{
	motor[right1] = valueR;
	motor[right2] = valueR;
}

/*Set the flywheen motors LEFT */
void
FwMotorSetL( int valueL )
{
	motor[left1] = valueL;
	motor[left2] = valueL;
}


/*Get the flywheen motor encoder count RIGHT*/
long
FwMotorEncoderGetR()
{
	return( nMotorEncoder[right1] );
}


/*Get the flywheen motor encoder count LEFT */
long
FwMotorEncoderGetL()
{
	return( nMotorEncoder[left1] );
}


/*Set the controller position RIGHT */
void
FwVelocitySetR( int velocityR, float predicted_driveR )
{
	// set target_velocity velocity (motor rpm)
	target_velocityR = velocityR;

	// Set error so zero crossing is correctly detected
	current_errorR = target_velocityR - motor_velocityR;
	last_errorR    = current_errorR;

	// Set predicted open loop drive value
	drive_approxR  = predicted_driveR;
	// Set flag to detect first zero crossing
	first_crossR   = 1;
	// clear tbh variable
	drive_at_zeroR = 0;
}


/*Set the controller position LEFT  */
void
FwVelocitySetL( int velocityL, float predicted_driveL )
{
	// set target_velocity velocity (motor rpm)
	target_velocityL = velocityL;

	// Set error so zero crossing is correctly detected
	current_errorL = target_velocityL - motor_velocityL;
	last_errorL    = current_errorL;

	// Set predicted open loop drive value
	drive_approxL  = predicted_driveL;
	// Set flag to detect first zero crossing
	first_crossL   = 1;
	// clear tbh variable
	drive_at_zeroL = 0;
}

/*Calculate the current flywheel motor velocity*/
void
FwCalculateSpeed()
{
	int     delta_msR;
	int     delta_msL;
	int     delta_encR;
	int     delta_encL;

	// Get current encoder value
	encoder_countsR = FwMotorEncoderGetR();
	encoder_countsL = FwMotorEncoderGetL();

	// This is just used so we don't need to know how often we are called
	// how many mS since we were last here
	delta_msR = nSysTime - nSysTime_lastR;
	nSysTime_lastR = nSysTime;
	delta_msL = nSysTime - nSysTime_lastL;
	nSysTime_lastL = nSysTime;

	// Change in encoder count
	delta_encR = (encoder_countsR - encoder_counts_lastR);
	delta_encL = (encoder_countsL - encoder_counts_lastL);

	// save last position
	encoder_counts_lastR = encoder_countsR;
	encoder_counts_lastL = encoder_countsL;

	// Calculate velocity in rpm
	motor_velocityR = (1000.0 / delta_msR) * delta_encR * 60.0 / ticks_per_rev;
	motor_velocityL = (1000.0 / delta_msL) * delta_encL * 60.0 / ticks_per_rev;
}

/*Update the velocity tbh controller variables RIGHT*/
void
FwControlUpdateVelocityTbhR()
{
	// calculate error in velocity
	// target_velocity is desired velocity
	// current is measured velocity
	current_errorR = target_velocityR - motor_velocityR;

	// Calculate new control value
	driveR =  driveR + (current_errorR * gain);

	// Clip to the range 0 - 1.
	// We are only going forwards
	if( driveR > 1 )
		driveR = 1;
	if( driveR < 0 )
		driveR = 0;

	// Check for zero crossing
	if( sgn(current_errorR) != sgn(last_errorR) ) {
		// First zero crossing after a new set velocity command
		if( first_crossR ) {
			// Set drive to the open loop approximation
			driveR = drive_approxR;
			first_crossR = 0;
		}
		else
			driveR = 0.5 * ( driveR + drive_at_zeroR );

		// Save this drive value in the "tbh" variable
		drive_at_zeroR = driveR;
	}

	// Save last error
	last_errorR = current_errorR;
}


/*Update the velocity tbh controller variables */
void
FwControlUpdateVelocityTbhL()
{
	// calculate error in velocity
	// target_velocity is desired velocity
	// current is measured velocity
	current_errorL = target_velocityL - motor_velocityL;

	// Calculate new control value
	driveL =  driveL + (current_errorL * gain);

	// Clip to the range 0 - 1.
	// We are only going forwards
	if( driveL > 1 )
		driveL = 1;
	if( driveL < 0 )
		driveL = 0;

	// Check for zero crossing
	if( sgn(current_errorL) != sgn(last_errorL) ) {
		// First zero crossing after a new set velocity command
		if( first_crossL ) {
			// Set drive to the open loop approximation
			driveL = drive_approxL;
			first_crossL = 0;
		}
		else
			driveL = 0.5 * ( driveL + drive_at_zeroL );

		// Save this drive value in the "tbh" variable
		drive_at_zeroL = driveL;
	}

	// Save last error
	last_errorL = current_errorL;
}

/*Task to control the velocity of the flywheel */
task
FwControlTask()
{
	// Set the gain
	gain = 0.00001;

	// We are using Speed geared motors
	// Set the encoder ticks per revolution
	ticks_per_rev = MOTOR_TPR_393T;

	while(1)
	{
		// Calculate velocity
		FwCalculateSpeed();

		// Do the velocity TBH calculations
		FwControlUpdateVelocityTbhR() ;
		FwControlUpdateVelocityTbhL() ;

		// Scale drive into the range the motors need
		motor_driveR  = (driveR * FW_MAX_POWER) + 0.5;
		motor_driveL  = (driveL * FW_MAX_POWER) + 0.5;

		// Final Limit of motor values - don't really need this
		if( motor_driveR >  127 ) motor_driveR =  127;
		if( motor_driveR < -127 ) motor_driveR = -127;
		if( motor_driveL >  127 ) motor_driveL =  127;
		if( motor_driveL < -127 ) motor_driveL = -127;

		// and finally set the motor control value
		FwMotorSetR( motor_driveR );
		FwMotorSetL( motor_driveL );

		// Run at somewhere between 20 and 50mS
		wait1Msec( FW_LOOP_SPEED );
	}
}


task usercontrol()
{
	// Start the flywheel control task
	StartTask( FwControlTask );

	// Main user control loop
	while(true)
	{
		int intakeControl = vexRT[Ch3];
		int driveLeftControl = vexRT[Ch3Xmtr2] * driveTrainSpeedControl;
		int driveRightControl = vexRT[Ch2Xmtr2] * driveTrainSpeedControl;
		int upControl = vexRT[Ch2];
		int startShooterControl = vexRT[Btn8D];
		int shooter30SpeedControl = vexRT[Btn6D];
		int shooter40SpeedControl = vexRT[Btn6U];
		int shooter43SpeedControl = vexRT[Btn7D];
		int shooter45SpeedControl = vexRT[Btn7L];
		int shooter47SpeedControl = vexRT[Btn7U];
		int shooter50SpeedControl = vexRT[Btn7R];
		int shooter55SpeedControl = vexRT[Btn5D];
		int shooter60SpeedControl = vexRT[Btn5U];
		int shooter35SpeedControl = vexRT[Btn8DXmtr2];
		int shooter37SpeedControl = vexRT[Btn8RXmtr2];



		//Drive Train Speed Control

		if(vexRT[Btn7DXmtr2] == 1)
		{
			driveTrainSpeedControl = 0.3;
		}
		else if(vexRT[Btn7UXmtr2] == 1)
		{
			driveTrainSpeedControl = 1;
		}


		// INTAKE
		if(abs(intakeControl) > deadzone)
		{
			motor[intake] = intakeControl;
			// control Intake
		}
		else
		{
			motor[intake] = 0;
			// stop Intake
		}
		//DRIVETRAIN LEFT
		if(abs(driveLeftControl) > deadzone)
		{
			motor[driveLeft] = driveLeftControl;
			motor[driveLeft2] = driveLeftControl;
		}
		else
		{
			motor[driveLeft] = 0;
			motor[driveLeft2] = 0;
		}
		//DRIVETRAIN RIGHT
		if(abs(driveRightControl) > deadzone)
		{
			motor[driveRight] = driveRightControl;
			motor[driveRight2] = driveRightControl;
		}
		else
		{
			motor[driveRight] = 0;
			motor[driveRight2] = 0;
		}
		//UP
		if(abs(upControl) > deadzone)
		{
			motor[up] = upControl;
			// start UP
		}
		else
		{
			motor[up] = 0;
			// stop UP
		}
		//START SHOOTER
		if(startShooterControl == 1)
		{
			motor[left1] = a;
			motor[left2] = a;
			motor[right1] = a;
			motor[right2] = a;
			wait1Msec(1000);
			motor[left1] = b ;
			motor[left2] = b ;
			motor[right1] = b ;
			motor[right2] = b ;
		}
		else if(vexRT[Btn8L] == 1)
		{

			motor[left1] = a;
			motor[left2] = a;
			motor[right1] = a;
			motor[right2] = a;
			wait1Msec(500);
			motor[left1] = a-5;
			motor[left2] = a-5;
			motor[right1] = a-5;
			motor[right2] = a-5;
			wait1Msec(500);
			motor[left1] = a-10;
			motor[left2] = a-10;
			motor[right1] = a-10;
			motor[right2] = a-10;
			wait1Msec(500);
			motor[left1] = a-15;
			motor[left2] = a-15;
			motor[right1] = a-15;
			motor[right2] = a-15;
			wait1Msec(500);
			motor[left1] = a-20;
			motor[left2] = a-20;
			motor[right1] = a-20;
			motor[right2] = a-20;
			wait1Msec(500);
			motor[left1] = a-30;
			motor[left2] = a-30;
			motor[right1] = a-30;
			motor[right2] = a-30;
			wait1Msec(500);
			motor[left1] = 0;
			motor[left2] = 0;
			motor[right1] = 0;
			motor[right2] = 0;
		}
		else if(shooter35SpeedControl == 1)
		{
			motor[left1] = 35;
			motor[left2] = 35;
			motor[right1] = 35;
			motor[right2] = 35;
		}
		else if(shooter37SpeedControl == 1)
		{
			motor[left1] = 37;
			motor[left2] = 37;
			motor[right1] = 37;
			motor[right2] = 37;
		}
		else if(shooter55SpeedControl == 1)
		{
			motor[left1] = 55;
			motor[left2] = 55;
			motor[right1] = 55;
			motor[right2] = 55;
		}
		else if(shooter30SpeedControl == 1)
		{
			motor[left1] = 30;
			motor[left2] = 30;
			motor[right1] = 30;
			motor[right2] = 30;
		}
		else if(shooter40SpeedControl == 1)
		{
			motor[left1] = 40;
			motor[left2] = 40;
			motor[right1] = 40;
			motor[right2] = 40;
		}
		else if(shooter50SpeedControl == 1)
		{
			motor[left1] = 50;
			motor[left2] = 50;
			motor[right1] = 50;
			motor[right2] = 50;
		}
		else if(shooter43SpeedControl == 1)
		{
			motor[left1] = 43;
			motor[left2] = 43;
			motor[right1] = 43;
			motor[right2] = 43;
		}
		else if(shooter47SpeedControl == 1)
		{
			motor[left1] = 47;
			motor[left2] = 47;
			motor[right1] = 47;
			motor[right2] = 47;
		}
		else if(shooter45SpeedControl == 1)
		{
			motor[left1] = 45;
			motor[left2] = 45;
			motor[right1] = 45;
			motor[right2] = 45;
		}
		else if(shooter60SpeedControl == 1)
		{
			motor[left1] = 60;
			motor[left2] = 60;
			motor[right1] = 60;
			motor[right2] = 60;
		}
		if(vexRT[Btn8U]) //* Emergency Stop *//
		{
			motor[left1] = 0;
			motor[left2] = 0;
			motor[right1] = 0;
			motor[right2] = 0;
		}

		/* ULTRASONIC READING */

		sonarValue = SensorValue(ultrasonic);



		if(vexRT[Btn8R] == 0)
		{
			// Displays Battery Levels to VEX Remote Screen
			powerExpanderBatteryV = SensorValue[in1]/.28;
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDPos(0,0);
			displayNextLCDString("Cortex Bat:");
			displayNextLCDNumber(nAvgBatteryLevel);
			displayLCDPos(1,0);
			displayNextLCDString("PowerE Bat:");
			displayNextLCDNumber(powerExpanderBatteryV);
		}
		else if(vexRT[Btn8R] == 1)
		{
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDPos(0,0);
			displayNextLCDString("Ultrasonic:");
			displayNextLCDNumber(sonarValue);
			displayLCDPos(1,0);
			displayNextLCDString("Exp. Bat");
			displayNextLCDNumber(powerExpanderBatteryV);
		}




	// Different speeds set by buttons
		if( vexRT[ Btn8L ] == 1 )
		{
			FwVelocitySetR( 144, 0.55 );
			FwVelocitySetL( 144, 0.55 );
		}
		if( vexRT[ Btn8U ] == 1 )
		{
			FwVelocitySetR( 120, 0.38 );
			FwVelocitySetL( 120, 0.38 );
		}
		if( vexRT[ Btn8R ] == 1 )
		{
			FwVelocitySetR( 50, 0.2 );
			FwVelocitySetL( 50, 0.2 );
		}
		if( vexRT[ Btn8D ] == 1 )
		{
			FwVelocitySetR( 00, 0 );
			FwVelocitySetL( 00, 0 );
		}


		// Don't hog the cpu :)
		wait1Msec(10);
	}
}
