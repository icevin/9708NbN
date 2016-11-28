#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    powerE,         sensorAnalog)
#pragma config(Sensor, dgtl1,  ultrasonic,     sensorSONAR_inch)
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
#define TBH_DRIVE_MAX    100
#define TBH_DRIVE_MIN     0

#define TBH_INTEGRAL_LIMIT  50
#define incValue			5 // Value buttons increment up or down
#define driveIncValue  3 //

#define a 45 // startup speed / slowdown speed
#define deadzone 15 // Deadzone for joysticks
#define b 58 // main shooting speed11

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

float TBHDrive; // Actual power to motors from TBH
float TBHDriveR; // Actual power to motors from TBH (right)
bool incUpLastPress = 0;
bool incDownLastPress = 0;
int requestedShooterSpeed = 120;

int firstcross = 1;

int tbh_drivePrediction = b;

float targetSpeed; //PLACEHOLDER FOR NOW
int driveAtZero = 0; // drive at last zero crossing

int firstcrossR = 1;
int driveAtZeroR = 0; // drive at last zero crossing

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"
float gain = 0.009;
float gainR = 0.009;


static int TBHRunning = 1;

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



/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  TBH control tasks                                                          */
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

task TBHController()
{
	float  TBHError;
	float  TBHLastError = 0;
	float  TBHIntegral = 0;
	float  TBHDerivative;


	float  TBHErrorR;
	float  TBHLastErrorR = 0;
	float  TBHIntegralR = 0;
	float  TBHDerivativeR;

	while( true )
	{
		// Is TBH control active ?
		if( TBHRunning )
		{
			calculateSpeed();
			targetSpeed = requestedShooterSpeed;

			// calculate error
			TBHErrorR = targetSpeed - flyVelocityR;

			TBHDriveR += TBHErrorR * gainR;

			if(sgn(TBHErrorR) != sgn(TBHLastErrorR))
			{
				if(firstcrossR == 1){
					TBHDriveR = tbh_drivePrediction;
					firstcrossR = 0;
				}
				else {
					TBHDriveR = 0.5 * (TBHDriveR + driveAtZeroR);
				}
				driveAtZeroR = TBHDriveR;
			}


			TBHLastErrorR = TBHErrorR;

			//limit drive
			if( TBHDriveR > TBH_DRIVE_MAX )
				TBHDriveR = TBH_DRIVE_MAX;
			if( TBHDriveR < TBH_DRIVE_MIN )
				TBHDriveR = TBH_DRIVE_MIN;



			// calculate error
			TBHError = targetSpeed - flyVelocityL;

			TBHDrive += TBHError * gain;

			if(sgn(TBHError) != sgn(TBHLastError))
			{
				if(firstcross == 1){
					TBHDrive = tbh_drivePrediction;
					firstcross = 0;
				}
				else {
					TBHDrive = 0.5 * (TBHDrive + driveAtZero);
				}
				driveAtZero = TBHDrive;
			}


			TBHLastError = TBHError;

			//limit drive
			if( TBHDrive > TBH_DRIVE_MAX )
				TBHDrive = TBH_DRIVE_MAX;
			if( TBHDrive < TBH_DRIVE_MIN )
				TBHDrive = TBH_DRIVE_MIN;

			if(targetSpeed == 0)
			{
				if(TBHError == 0)
				{
					TBHDrive = 0;
				}

				if(TBHErrorR == 0)
				{
					TBHDriveR = 0;
				}
			}
		}
		else
		{
			// clear all
			TBHError      = 0;
			TBHLastError  = 0;
			TBHIntegral   = 0;
			TBHDerivative = 0;
			TBHErrorR      = 0;
			TBHLastErrorR  = 0;
			TBHIntegralR   = 0;
			TBHDerivativeR = 0;
		}

		wait1Msec( 30 );
	}
}

task autonomous()
{
	requestedShooterSpeed = 120;
	StartTask( TBHController );

	ClearTimer(T4);
	while(time1[T4] < 2500)
	{
		motor[left1] = TBHDrive;
		motor[left2] = TBHDrive;
		motor[right1] = TBHDriveR;
		motor[right2] = TBHDriveR;
	}

	ClearTimer(T2);
	while(time1[T2] < 12000)
	{
		motor[left1] = TBHDrive;
		motor[left2] = TBHDrive;
		motor[right1] = TBHDriveR;
		motor[right2] = TBHDriveR;
		motor[intake] = 60;
		motor[up] = 70;
	}
	ClearTimer(T4);
	ClearTimer(T2);

}

task usercontrol()
{
	ClearTimer(T1);
		ClearTimer(T2);
	ClearTimer(T3);
	ClearTimer(T4);
	// begin TBH requested valuef
	StartTask( TBHController );
	requestedShooterSpeed = 120;
	while(true)
	{
		int intakeControl = vexRT[Ch3];
		int driveLeftControl = vexRT[Ch3Xmtr2] * driveTrainSpeedControl;
		int driveRightControl = vexRT[Ch2Xmtr2] * driveTrainSpeedControl;
		int upControl = vexRT[Ch2];
		int startShooterControl = vexRT[Btn8D];
		int shooterSPPointsSpeedControl = vexRT[Btn7DXmtr2];
		int shooterUpCloseSpeedControl = vexRT[Btn7UXmtr2];
		int shooterMidLineSpeedControl = vexRT[Btn7LXmtr2];
		int shooterFullFieldSpeedControl = vexRT[Btn7RXmtr2];

		int incUpCtrlSmall = vexRT[Btn6U];
		int incDownCtrlSmall = vexRT[Btn6D];

		// Incrementing requestedShooterSpeed

		int incUpCtrl = vexRT[Btn5U];
		int incDownCtrl = vexRT[Btn5D];


		if((incUpCtrl == 1) && (requestedShooterSpeed < 150) && (incUpLastPress == false))
		{
			incUpLastPress = true;
			requestedShooterSpeed += incValue;
			// tbh_drivePrediction += driveIncValue;

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
			// tbh_drivePrediction = tbh_drivePrediction - driveIncValue;
			wait1Msec(200);
		}
		else if(incDownCtrl == 0)
		{
			incDownLastPress = false;
		}

		int incUpLastPressSmall = 0;
		int incDownLastPressSmall = 0;
		int incValueSmall = 1;

		if((incUpCtrlSmall == 1) && (requestedShooterSpeed < 150) && (incUpLastPressSmall == false))
		{
			incUpLastPressSmall = true;
			requestedShooterSpeed += incValueSmall;
			// tbh_drivePrediction += driveIncValue;

			wait1Msec(200);
		}
		else if(incUpCtrlSmall == 0)
		{
			incUpLastPressSmall = false;
		}

		if((incDownCtrlSmall == 1) && (requestedShooterSpeed > 0)  && (incDownLastPressSmall == false))
		{
			incDownLastPressSmall = true;
			requestedShooterSpeed = requestedShooterSpeed - incValueSmall;
			// tbh_drivePrediction = tbh_drivePrediction - driveIncValue;
			wait1Msec(200);
		}
		else if(incDownCtrlSmall == 0)
		{
			incDownLastPressSmall = false;
		}





		if(requestedShooterSpeed > 150)
		{
			requestedShooterSpeed = 150;
		}

		if(requestedShooterSpeed < 0)
		{
			requestedShooterSpeed = 0;
		}

		//End increment

		int isTBHUpdating;

		if(vexRT[Btn7L] == 1)
		{
			isTBHUpdating = 1;
			requestedShooterSpeed = 130;
		}

		if(vexRT[Btn7R] == 1)
		{
			isTBHUpdating = 0;
		}

		if(isTBHUpdating == 1)
		{
			motor[left1] = TBHDrive;
			motor[left2] = TBHDrive;
			motor[right1] = TBHDriveR;
			motor[right2] = TBHDriveR;
		}

		//Drive Train Speed Control

		if(vexRT[Btn6DXmtr2] == 1)
		{
			driveTrainSpeedControl = 0.3;
		}
		else if(vexRT[Btn6UXmtr2] == 1)
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
		else if(shooterSPPointsSpeedControl == 1)
		{
			requestedShooterSpeed = 115;
		}
		else if(shooterUpCloseSpeedControl == 1)
		{
			requestedShooterSpeed = 115;
		}
		else if(shooterMidLineSpeedControl == 1)
		{
			requestedShooterSpeed = 113;
		}
		else if(shooterFullFieldSpeedControl == 1)
		{
			requestedShooterSpeed = 134;
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
