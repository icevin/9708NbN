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
#pragma config(Motor,  port5,           driveLeft,     tmotorVex393, openLoop, encoder, encoderPort, I2C_3, 1000)
#pragma config(Motor,  port6,           driveRight,    tmotorVex393, openLoop, reversed, encoder, encoderPort, I2C_2, 1000)
#pragma config(Motor,  port7,           driveRight2,   tmotorVex269, openLoop, reversed)
#pragma config(Motor,  port8,           right1,        tmotorVex393, openLoop, reversed, encoder, encoderPort, I2C_1, 1000)
#pragma config(Motor,  port9,           right2,        tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port10,          intake,        tmotorVex393, openLoop, reversed)
#pragma platform(VEX)


#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

// Update inteval (in mS) for the flywheel control loop
#define FW_LOOP_SPEED              100

// Maximum power we want to send to the flywheel motors
#define FW_MAX_POWER              127

// encoder counts per revolution depending on motor
#define MOTOR_TPR_269           240.448
#define MOTOR_TPR_393R          261.333
#define MOTOR_TPR_393S          392
#define MOTOR_TPR_393T          627.2
#define MOTOR_TPR_QUAD          360.0

int a=35; // startup speed / slowdown speed
int deadzone=15; // Deadzone for joysticks
int b=43; // main shooting speed
float driveTrainSpeedControl = 1;
int sonarValue = 0;
float powerExpanderBatteryV = 2;

int deltaTicksRight;
int lastRunTicksRight;
int thisRunTicksRight;
int deltaTicksLeft;
int lastRunTicksLeft;
int thisRunTicksLeft;
int flyVelocityL;
int flyVelocityR;
float timeInMinutes;
float targetSpeed = 47; //PLACEHOLDER FOR NOW

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
	// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.


//	nMotorEncoder[driveLeftEncoder] = 0;
//	nMotorEncoder[driveRightEncoder] = 0;
//	nMotorEncoder[shootLeftEncoder] = 0;
// nMotorEncoder[shootRightEncoder] = 0;
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


// PID using optical shaft encoder
//
// Shaft encoder has 360 pulses per revolution
//

#define PID_SENSOR_INDEX    myEncoder
#define PID_SENSOR_SCALE    1

#define PID_MOTOR_INDEX     driveLeft
#define PID_MOTOR_SCALE     -1

#define PID_DRIVE_MAX       127
#define PID_DRIVE_MIN     (-127)

#define PID_INTEGRAL_LIMIT  50

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"
float  pid_Kp = 2.0;
float  pid_Ki = 0.04;
float  pid_Kd = 0.0;

static int   pidRunning = 1;
static float pidRequestedValue;

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  pid control task                                                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
void calculateSpeed()
{
	//Velocity reading
		int flywheelRightEncoder = SensorValue[I2C_3];
		int flywheelLeftEncoder = SensorValue[I2C_1];

		lastRunTicksRight = thisRunTicksRight;
		thisRunTicksRight = flywheelRightEncoder;
		deltaTicksRight = thisRunTicksRight - lastRunTicksRight;

		lastRunTicksLeft = thisRunTicksLeft;
		thisRunTicksLeft = flywheelLeftEncoder;
		deltaTicksLeft = thisRunTicksLeft - thisRunTicksRight;

		float timeSinceLastRun = time10[T1];
		timeInMinutes = timeSinceLastRun * 6000;
		flyVelocityL =  deltaTicksLeft/timeInMinutes;

		flyVelocityR =  deltaTicksRight/timeInMinutes;


		ClearTimer(T1);
}

task pidController()
{

    float  pidError;
    float  pidLastError;
    float  pidIntegral;
    float  pidDerivative;
    float  pidDrive;

    // Init the variables - thanks Glenn :)
    pidLastError  = 0;
    pidIntegral   = 0;

    while( true )
        {
        // Is PID control active ?
        if( pidRunning )
            {

            // calculate error
            pidError = targetSpeed - flyVelocityR;

            // integral - if Ki is not 0
            if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( abs(pidError) < PID_INTEGRAL_LIMIT )
                    pidIntegral = pidIntegral + pidError;
                else
                    pidIntegral = 0;
                }
            else
                pidIntegral = 0;

            // calculate the derivative
            pidDerivative = pidError - pidLastError;
            pidLastError  = pidError;

            // calculate drive
            pidDrive += (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

            // limit drive
            if( pidDrive > PID_DRIVE_MAX )
                pidDrive = PID_DRIVE_MAX;
            if( pidDrive < PID_DRIVE_MIN )
                pidDrive = PID_DRIVE_MIN;

            // send to motor
            motor[driveLeft] = pidDrive * PID_MOTOR_SCALE;
            }
        else
            {
            // clear all
            pidError      = 0;
            pidLastError  = 0;
            pidIntegral   = 0;
            pidDerivative = 0;
            motor[driveLeft] = 0;
            }

        // Run at 50Hz
        wait1Msec( 25 );
        }
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  main task                                                                  */
/*                                                                             */
/*-----------------------------------------------------------------------------*/


task usercontrol()
{

    // send the motor off somewhere
    pidRequestedValue = 1000;

    // start the PID task
    StartTask( pidController );

    // use joystick to modify the requested position
    while( true )
        {
        // maximum change for pidRequestedValue will be 127/4*20, around 640 counts per second
        // free spinning motor is 100rmp so 1.67 rotations per second
        // 1.67 * 360 counts is 600

        pidRequestedValue = pidRequestedValue + (vexRT[ Ch2 ]/4);

        wait1Msec(50);
        }


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
		int shooter35SpeedControl = vexRT[Btn6DXmtr2];
		int shooter37SpeedControl = vexRT[Btn6UXmtr2];



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

		// Don't hog the cpu :)
		wait1Msec(10);
	}
}