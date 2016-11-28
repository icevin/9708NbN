#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    powerE,         sensorAnalog)
#pragma config(Sensor, dgtl1,  quadLeft,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  quadRight,      sensorQuadEncoder)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           up,            tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           driveLeft1,    tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port3,           left1,         tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_4)
#pragma config(Motor,  port4,           left2,         tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           intake,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           right1,        tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port7,           right2,        tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           driveRight1,   tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port10,          up2,           tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)


#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!
#define PID_DRIVE_MAX 110
#define PID_DRIVE_MIN 0

#define PID_INTEGRAL_LIMIT 40
#define incValue 10
#define incValueSmall 5
#define SMLIB_TPR_393Turbo 261.333
#define SMLIB_TPR_393Speed 392.0
#define SMLIB_TPR_393Torque 627.2
#define SMLIB_TPR_393Quad 360.0
#define PID_VEL_MAX 700

int deadzone = 20;
float driveTrainSpeedControl = 1.0;
float powerExpanderBatteryV = 2;

float flyVelocityL;
float flyVelocityR;

float targetSpeed;
float pidDriveL;
float pidDriveR;
int incUpLastPress = 0;
int incDownLastPress = 0;
int incUpLastPressSmall = 0;
int incDownLastPressSmall = 0;
int requestedShooterSpeed = 0;

float pid_Kp = 0.2;
float pid_Ki = 0.0;
float pid_Kd = 1.0;

float pid_Kp_right = 0.2;
float pid_Ki_right = 0.0;
float pid_Kd_right = 1.0;

static int pidRunning = 1;
long timeSinceLastRun;
int deltaTicksRight;
int lastRunTicksRight;
int thisRunTicksRight;
int deltaTicksLeft;
int lastRunTicksLeft;
int thisRunTicksLeft;
int updatePID;

/////////////////////////////////////////////////////////////////////////////////////////
//
//                          Pre-Autonomous Functions
//
// You may want to perform some actions before the competition starts. Do them in the
// following function.
//
/////////////////////////////////////////////////////////////////////////////////////////
void slowDown()
{
	updatePID = 0;
	for(int i = 49; i >= 0; i-= 2)
	{
		motor[left1] = i;
		motor[left2] = i;
		motor[right1] = i;
		motor[right2] = i;
		wait1Msec(50);
	}
	motor[left1] = 0;
	motor[left2] = 0;
	motor[right1] = 0;
	motor[right2] = 0;
}

void startup()
{
	updatePID = 0;
	for(int i = 0; i <= 32; i+= 2)
	{
		motor[left1] = i;
		motor[left2] = i;
		motor[right1] = i;
		motor[right2] = i;
		wait1Msec(50);
	}
}
void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
	// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
	nMotorEncoder[left1] = 0;
	nMotorEncoder[right1] = 0;
	nMotorEncoder[driveLeft1] = 0;
	nMotorEncoder[driveRight1] = 0;
	SensorValue[quadLeft] = 0;
	SensorValue[quadRight] = 0;
	bStopTasksBetweenModes = true;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}
void calculateSpeed()
{
	//Velocity reading
	int flywheelRightEncoder = -1 * SensorValue[quadRight];
	int flywheelLeftEncoder =SensorValue[quadLeft];

	lastRunTicksRight = thisRunTicksRight;
	thisRunTicksRight = flywheelRightEncoder;
	deltaTicksRight = thisRunTicksRight - lastRunTicksRight;

	lastRunTicksLeft = thisRunTicksLeft;
	thisRunTicksLeft = flywheelLeftEncoder;
	deltaTicksLeft = thisRunTicksLeft - lastRunTicksLeft;

	timeSinceLastRun = time10[T1];
	flyVelocityL =  (deltaTicksLeft * 50)/(timeSinceLastRun * 3);

	flyVelocityR =  (deltaTicksRight* 50)/(timeSinceLastRun * 3);

	clearTimer(T1);
}

task pidController()
{
	float pidErrorL;
	float pidLastErrorL;
	float pidIntegralL;
	float pidDerivativeL;

	float pidErrorR;
	float pidLastErrorR;
	float pidIntegralR;
	float pidDerivativeR;

	pidLastErrorL = 0;
	pidIntegralL = 0;

	pidLastErrorR = 0;
	pidIntegralR = 0;

	while (true)
	{
		if(pidRunning)
		{

			targetSpeed = requestedShooterSpeed;
			pidErrorL = targetSpeed-flyVelocityL;

			if(pid_Ki!=0)
			{
				if(fabs(pidErrorL) < PID_INTEGRAL_LIMIT)
					pidIntegralL = pidIntegralL + pidErrorL;
				else
					pidIntegralL = 0;

			}
			else
				pidIntegralL = 0;
			if(pidErrorL !=0)
			{
				pidDerivativeL = pidErrorL - pidLastErrorL;
				pidLastErrorL = pidErrorL;
			}
			else
			{
				pidDerivativeL = 0;
			}
			if(pidErrorL < 2.0)
			{
				pidIntegralL = 0;
			}
			pidDriveL+= (pid_Kp*pidErrorL) + (pid_Ki*pidIntegralL) + (pid_Kd * pidDerivativeL);

			if(pidDriveL > PID_DRIVE_MAX)
				pidDriveL = PID_DRIVE_MAX;
			if(pidDriveL < PID_DRIVE_MIN)
				pidDriveL = PID_DRIVE_MIN;

			//right

			pidErrorR = targetSpeed-flyVelocityR;

			if(pid_Ki!=0)
			{
				if(fabs(pidErrorR) <PID_INTEGRAL_LIMIT)
					pidIntegralR = pidIntegralR + pidErrorR;
				else
					pidIntegralR = 0;

			}
			else
				pidIntegralR = 0;
			if(pidErrorR !=0)
			{
				pidDerivativeR = pidErrorR - pidLastErrorR;
				pidLastErrorR = pidErrorR;
			}
			else
			{
				pidDerivativeR = 0;
			}
			if(pidErrorR < 2.0)
			{
				pidIntegralR = 0;
			}
			pidDriveR+= (pid_Kp_right*pidErrorR) + (pid_Ki_right*pidIntegralR) + (pid_Kd_right * pidDerivativeR);

			if(pidDriveR > PID_DRIVE_MAX)
				pidDriveR = PID_DRIVE_MAX;
			if(pidDriveR < PID_DRIVE_MIN)
				pidDriveR = PID_DRIVE_MIN;

			calculateSpeed();

		}
		else
		{
			pidErrorL = 0;
			pidLastErrorL = 0;
			pidIntegralL = 0;
			pidDerivativeL = 0;
			pidErrorR = 0;
			pidLastErrorR = 0;
			pidIntegralR = 0;
			pidDerivativeR = 0;
		}
		wait1Msec(100);

	}
}
/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task autonomous()
{
	// .....................................................................................
	// Insert user code here.
	// .....................................................................................
	requestedShooterSpeed = 453;
	startTask(pidController);
	clearTimer(T2);
	while(time1[T2] < 3000)
	{
		motor[left1] = pidDriveL;
		motor[left2] = pidDriveL;
		motor[right1] = pidDriveR;
		motor[right2] = pidDriveR;
	}
	motor[intake] = 60;
	motor[up] = 70;
	motor[up2] = 70;
	clearTimer(T2);
	while(time1[T2] < 7000)
	{
		motor[left1] = pidDriveL;
		motor[left2] = pidDriveL;
		motor[right1] = pidDriveR;
		motor[right2] = pidDriveR;
	}
	slowDown();
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task
//
// This task is used to control your robot during the user control phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task usercontrol()
{
	requestedShooterSpeed = 450;

	startTask(pidController);
	int intakeControl;
	int driveLeftControl;
	int driveRightControl;
	int upControl;
	int startShooterControl;

	int incUpCtrl;
	int incDownCtrl;
	int incUpCtrlSmall;
	int incDownCtrlSmall;


	while (true)
	{
		intakeControl = vexRT[Ch3];
		driveLeftControl = vexRT[Ch3Xmtr2] * driveTrainSpeedControl;
		driveRightControl = vexRT[Ch2Xmtr2] * driveTrainSpeedControl;
		upControl = vexRT[Ch2];
		startShooterControl = vexRT[Btn8D];

		incUpCtrl = vexRT[Btn5U];
		incDownCtrl = vexRT[Btn5D];

		incUpCtrlSmall = vexRT[Btn6U];
		incDownCtrlSmall = vexRT[Btn6D];

		if((incUpCtrl == 1) && (requestedShooterSpeed<PID_VEL_MAX)&&(incUpLastPress == false))
		{
			incUpLastPress = true;
			requestedShooterSpeed += incValue;
			wait1Msec(200);
		}
		else if(incUpCtrl == 0)
		{
			incUpLastPress = false;
		}
		if((incDownCtrl == 1) && (requestedShooterSpeed>0)&&(incDownLastPress == false))
		{
			incDownLastPress = true;
			requestedShooterSpeed -= incValue;
			wait1Msec(200);
		}
		else if(incDownCtrl == 0)
		{
			incDownLastPress = false;
		}



		if((incUpCtrlSmall == 1) && (requestedShooterSpeed<PID_VEL_MAX)&&(incUpLastPressSmall == false))
		{
			incUpLastPressSmall = true;
			requestedShooterSpeed += incValueSmall;
			wait1Msec(200);
		}
		else if(incUpCtrlSmall == 0)
		{
			incUpLastPressSmall = false;
		}
		if((incDownCtrlSmall == 1) && (requestedShooterSpeed>0)&&(incDownLastPressSmall == false))
		{
			incDownLastPressSmall = true;
			requestedShooterSpeed -= incValueSmall;
			wait1Msec(200);
		}
		else if(incDownCtrlSmall == 0)
		{
			incDownLastPressSmall = false;
		}




		if(requestedShooterSpeed > PID_VEL_MAX)
		{
			requestedShooterSpeed = PID_VEL_MAX;
		}
		if(requestedShooterSpeed < 0)
		{
			requestedShooterSpeed = 0;
		}

		if(vexRT[Btn7U] == 1)
		{
			requestedShooterSpeed = 460;
			pid_Kp = 0.2;
			pid_Ki = 0.0;
			pid_Kd = 1.0;

			pid_Kp_right = 0.2;
			pid_Ki_right = 0.0;
			pid_Kd_right = 1.0;
		}

		if(vexRT[Btn7D] == 1)
		{
			requestedShooterSpeed = 370;
			pid_Kp = 0.05;
			pid_Ki = 0.0;
			pid_Kd = 0.9;

			pid_Kp_right = 0.05;
			pid_Ki_right = 0.0;
			pid_Kd_right = 0.9;
		}

		if(vexRT[Btn8R] == 1)
		{
			pid_Kp = 0.02;
			pid_Ki = 0.0;
			pid_Kd = 0.3;

			pid_Kp_right = 0.02;
			pid_Ki_right = 0.0;
			pid_Kd_right = 0.3;
		}
		if(vexRT[Btn7L] == 1)
		{
			updatePID = 1;
		}
		else if(vexRT[Btn7R] == 1)
		{
			updatePID = 0;
		}

		if(updatePID == 1)

		{
			motor[left1] = pidDriveL;
			motor[left2] = pidDriveL;
			motor[right1] = pidDriveR;
			motor[right2] = pidDriveR;
		}

		if(vexRT[Btn7DXmtr2] == 1)
		{
			driveTrainSpeedControl = 0.3;
		}
		else if(vexRT[Btn7UXmtr2] == 1)
		{
			driveTrainSpeedControl = 1;
		}
		if(abs(intakeControl) > deadzone)
		{
			motor[intake] = intakeControl;
			motor[up2] = intakeControl;
		}
		else
		{
			motor[intake] = 0;
			motor[up2] = 0;
		}
		if(abs(driveLeftControl) > deadzone)
		{
			motor[driveLeft1] = driveLeftControl;
		}
		else
		{
			motor[driveLeft1] = 0;
		}
		if(abs(driveRightControl) > deadzone)
		{
			motor[driveRight1] = driveRightControl;
		}
		else
		{
			motor[driveRight1] = 0;
		}
		if(abs(upControl) > deadzone)
		{
			motor[up] = upControl;
		}
		else
		{
			motor[up] = 0;
		}
		if(startShooterControl == 1)
		{
			startup();
		}
		else if(vexRT[Btn8L] == 1)
		{
			slowDown();
		}

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

		datalogDataGroupStart();
		datalogAddValue( 0, flyVelocityL );
		datalogAddValue( 1, flyVelocityR );
		datalogAddValue(2, pidDriveL);
		datalogAddValue(3, pidDriveR);
		datalogDataGroupEnd();
		wait1Msec(25);
	}
}