#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    powerE,         sensorAnalog)
#pragma config(Sensor, dgtl1,  ultrasonic,     sensorSONAR_cm)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           up,            tmotorVex393, openLoop)
#pragma config(Motor,  port2,           left1,         tmotorVex393HighSpeed, openLoop, reversed, encoder, encoderPort, I2C_4, 1000)
#pragma config(Motor,  port3,           left2,         tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port4,           driveLeft2,    tmotorVex393, openLoop)
#pragma config(Motor,  port5,           driveLeft,     tmotorVex393, openLoop, encoder, encoderPort, I2C_1, 1000)
#pragma config(Motor,  port6,           driveRight,    tmotorVex393, openLoop, reversed, encoder, encoderPort, I2C_2, 1000)
#pragma config(Motor,  port7,           driveRight2,   tmotorVex269, openLoop, reversed)
#pragma config(Motor,  port8,           right1,        tmotorVex393HighSpeed, openLoop, encoder, encoderPort, I2C_3, 1000)
#pragma config(Motor,  port9,           right2,        tmotorVex393, openLoop)
#pragma config(Motor,  port10,          intake,        tmotorVex393, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

#include "Vex_Competition_Includes.c"
#define PID_DRIVE_MAX    100
#define PID_DRIVE_MIN     0

#define PID_INTEGRAL_LIMIT  50
#define incValue			5 // Value buttons increment up or down

#define a 35 // startup speed / slowdown speed
#define deadzone 15 // Deadzone for joysticks
#define b 43 // main shooting speed

float driveTrainSpeedControl = 1;
int sonarValue = 0;
float powerExpanderBatteryV;

long timeSinceLastRun;
int deltaTicksRight;
int lastRunTicksRight;
int thisRunTicksRight;
int deltaTicksLeft;
int lastRunTicksLeft;
int thisRunTicksLeft;
float flyVelocityL;
float flyVelocityR;

float pidDrive; // Actual power to motors from PID
float pidDriveR; // Actual power to motors from PID (right)
bool incUpLastPress = 0;
bool incDownLastPress = 0;
int requestedShooterSpeed = 0;

//Will eventually be controlled
int firstcross = 1;

int tbh_drivePrediction = a;

float targetSpeed; //PLACEHOLDER FOR NOW
int driveAtZero = 0; // drive at last zero crossing

int firstcrossR = 1;
int driveAtZeroR = 0; // drive at last zero crossing

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"
float gain = 0.001;
float gainR = 0.001;


static int pidRunning = 1;

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
	// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.


	//SensorValue[driveLeftEncoder] = 0;
	//SensorValue[driveRightEncoder] = 0;
	nMotorEncoder[left1] = 0;
	nMotorEncoder[right1]= 0;
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

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  pid control tasks                                                          */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
void calculateSpeed()
{
	//Velocity reading
	int flywheelRightEncoder = SensorValue[I2C_3];
	int flywheelLeftEncoder = -1 * SensorValue[I2C_4];

	lastRunTicksRight = thisRunTicksRight;
	thisRunTicksRight = flywheelRightEncoder;
	deltaTicksRight = thisRunTicksRight - lastRunTicksRight;

	lastRunTicksLeft = thisRunTicksLeft;
	thisRunTicksLeft = flywheelLeftEncoder;
	deltaTicksLeft = thisRunTicksLeft - lastRunTicksLeft;

	timeSinceLastRun = time10[T1];
	flyVelocityL =  (deltaTicksLeft * 50)/(timeSinceLastRun * 3);

	flyVelocityR =  (deltaTicksRight* 50)/(timeSinceLastRun * 3);

	ClearTimer(T1);
}

task pidController()
{
	float  pidError;
	float  pidLastError = 0;
	float  pidIntegral = 0;
	float  pidDerivative;


	float  pidErrorR;
	float  pidLastErrorR = 0;
	float  pidIntegralR = 0;
	float  pidDerivativeR;

	while( true )
	{
		// Is PID control active ?
		if( pidRunning )
		{
			calculateSpeed();
			targetSpeed = requestedShooterSpeed;

			// calculate error
			pidErrorR = targetSpeed - flyVelocityR;

			pidDriveR += pidErrorR * gainR;

			if(sgn(pidErrorR) != sgn(pidLastErrorR))
			{
				if(firstcrossR == 1){
					pidDriveR = tbh_drivePrediction;
					firstcrossR = 0;
				}
				else {
					pidDriveR = 0.5 * (pidDriveR + driveAtZeroR);
				}
				driveAtZeroR = pidDriveR;
			}


			pidLastErrorR = pidErrorR;

			//limit drive
			if( pidDriveR > PID_DRIVE_MAX )
				pidDriveR = PID_DRIVE_MAX;
			if( pidDriveR < PID_DRIVE_MIN )
				pidDriveR = PID_DRIVE_MIN;



			// calculate error
			pidError = targetSpeed - flyVelocityL;

			pidDrive += pidError * gain;

			if(sgn(pidError) != sgn(pidLastError))
			{
				if(firstcross == 1){
					pidDrive = tbh_drivePrediction;
					firstcross = 0;
				}
				else {
					pidDrive = 0.5 * (pidDrive + driveAtZero);
				}
				driveAtZero = pidDrive;
			}


			pidLastError = pidError;

			//limit drive
			if( pidDrive > PID_DRIVE_MAX )
				pidDrive = PID_DRIVE_MAX;
			if( pidDrive < PID_DRIVE_MIN )
				pidDrive = PID_DRIVE_MIN;
		}
		else
		{
			// clear all
			pidError      = 0;
			pidLastError  = 0;
			pidIntegral   = 0;
			pidDerivative = 0;
			pidErrorR      = 0;
			pidLastErrorR  = 0;
			pidIntegralR   = 0;
			pidDerivativeR = 0;
		}

		wait1Msec( 30 );
	}
}



task usercontrol()
{
	// begin TBH requested value
	targetSpeed = 45;
	StartTask( pidController );

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
		int shooter35SpeedControl = vexRT[Btn6DXmtr2];
		int shooter37SpeedControl = vexRT[Btn6UXmtr2];

		// Incrementing requestedShooterSpeed

		int incUpCtrl = vexRT[Btn5UXmtr2];
		int incDownCtrl = vexRT[Btn5DXmtr2];


		if((incUpCtrl == 1) && (requestedShooterSpeed < 100) && (incUpLastPress == false))
		{
			incUpLastPress = true;
			requestedShooterSpeed += incValue;
			wait1Msec(200);
		}
		else if(incUpCtrl == 0)
		{
			incUpLastPress = false;
		}

		if((incDownCtrl == 1) && (requestedShooterSpeed > 0)  && (incDownLastPress == false))
		{
			incDownLastPress = true;
			requestedShooterSpeed = requestedShooterSpeed - incValue;
			wait1Msec(200);
			}
		else if(incDownCtrl == 0)
		{
			incDownLastPress = false;
		}

		if(requestedShooterSpeed > 100)
		{
			requestedShooterSpeed = 100;
		}

		if(requestedShooterSpeed < 0)
		{
			requestedShooterSpeed = 0;
		}

		//End increment

		if(vexRT[Btn7LXmtr2] == 1)
		{
			motor[left1] = pidDrive;
			motor[left2] = pidDrive;
			motor[right1] = pidDriveR;
			motor[right2] = pidDriveR;
		}

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

		if(vexRT[Btn8R] == 0)
		{
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
			sonarValue = SensorValue(ultrasonic);
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDPos(0,0);
			displayNextLCDString("Ultrasonic:");
			displayNextLCDNumber(sonarValue);
			displayLCDPos(1,0);
			displayNextLCDString("Exp. Bat");
			displayNextLCDNumber(powerExpanderBatteryV);
		}

		wait1Msec(25);
	}
}
