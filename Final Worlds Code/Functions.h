#ifndef FUNCTIONS
#define FUNCTIONS

#define PID_DRIVE_MAX 120
#define PID_DRIVE_MIN 0

#define PID_INTEGRAL_LIMIT 40
#define incValue 10
#define incValueSmall 5
#define PID_VEL_MAX 700
#define deadzone 20

float driveTrainSpeedControl = 1.0;
float powerExpanderBatteryV = 2;

float flyVelocityL;
float flyVelocityR;
int flyStartup = 1;

float pidDriveL, pidDriveR;
int incUpLastPress = 0;
int incDownLastPress = 0;
int incUpLastPressSmall = 0;
int incDownLastPressSmall = 0;
int requestedShooterSpeed = 0;

int startspeed=40;

float pid_Kp = 0.05;
float pid_Ki = 0.0;
float pid_Kd = 0.4;

float pid_Kp_right = 0.05;
float pid_Ki_right = 0.0;
float pid_Kd_right = 0.41;


float	pid_Kp_selected = 0.0235;
float	pid_Ki_selected = 0.0;
float	pid_Kd_selected = 0.758;

float	pid_Kp_selectedR = 0.025;
float	pid_Ki_selectedR = 0.0;
float	pid_Kd_selectedR = 0.75;


long timeSinceLastRun = 1;
int deltaTicksRight;
int lastRunTicksRight;
int thisRunTicksRight;
int deltaTicksLeft;
int lastRunTicksLeft;
int thisRunTicksLeft;
int updatePID;

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
	if (timeSinceLastRun) {
		flyVelocityL =  (deltaTicksLeft * 50)/(timeSinceLastRun * 3);
		flyVelocityR =  (deltaTicksRight* 50)/(timeSinceLastRun * 3);
	}
	//	lastSysTime = nSysTime;
	clearTimer(T1);
}

void pidSelect()
{
	pid_Kp = pid_Kp_selected;
	pid_Ki = pid_Ki_selected;
	pid_Kd = pid_Kd_selected;

	pid_Kp_right = pid_Kp_selectedR;
	pid_Ki_right = pid_Ki_selectedR;
	pid_Kd_right = pid_Kd_selectedR;
}

task music() {
	while(1) {
		if(flyVelocityL > 200 && updatePID ==1){
			clearSounds();
			playImmediateTone(flyVelocityL + 150, 50);
		}
		wait1Msec(30);
	}
}

task pidController()
{
	startTask(music, 4);
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
		if(flyStartup == 1 && (flyVelocityR>requestedShooterSpeed))
			flyStartup = 0;
		else if(flyStartup==0)
			pidSelect();

		pidErrorL = requestedShooterSpeed-flyVelocityL;

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
		if(pidErrorL < 4.0)
		{
			pidIntegralL = 0;
		}
		pidDriveL+= (pid_Kp*pidErrorL) + (pid_Ki*pidIntegralL) + (pid_Kd * pidDerivativeL);

		if(pidDriveL > PID_DRIVE_MAX)
			pidDriveL = PID_DRIVE_MAX;
		if(pidDriveL < PID_DRIVE_MIN)
			pidDriveL = PID_DRIVE_MIN;

		//right

		pidErrorR = requestedShooterSpeed-flyVelocityR;

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
		if(pidErrorR < 4.0)
		{
			pidIntegralR = 0;
		}
		pidDriveR+= (pid_Kp_right*pidErrorR) + (pid_Ki_right*pidIntegralR) + (pid_Kd_right * pidDerivativeR);

		if(pidDriveR > PID_DRIVE_MAX)
			pidDriveR = PID_DRIVE_MAX;
		if(pidDriveR < PID_DRIVE_MIN)
			pidDriveR = PID_DRIVE_MIN;

		calculateSpeed();

		////LEDs
		//if(pidErrorR < 5)
		//	SensorValue(ledGreen) = 1;
		//else
		//	SensorValue(ledGreen) = 0;

		//if(pidErrorR < 15)
		//	SensorValue(ledYellow) = 1;
		//else
		//	SensorValue(ledYellow) = 0;

		//if(pidErrorR > 30)
		//	SensorValue(ledRed) = 1;
		//else
		//	SensorValue(ledRed) = 0;

		wait1Msec(100);

	}
	//pidErrorL = 0;
	//pidLastErrorL = 0;
	//pidIntegralL = 0;
	//pidDerivativeL = 0;
	//pidErrorR = 0;
	//pidLastErrorR = 0;
	//pidIntegralR = 0;
	//pidDerivativeR = 0;
}

void clearEncoders() {
	nMotorEncoder[driveLeft1] = 0;
	nMotorEncoder[driveRight1] = 0;
}

void drive(int encoderCounts, int speed)
{
	clearEncoders();
	//While both of the encoders are less than the specified amount
	while(abs(nMotorEncoder[driveRight1]) < abs(encoderCounts))
	{
		//If the two encoder values are equal
		if(abs(nMotorEncoder[driveRight1]) == abs(nMotorEncoder[driveLeft1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = speed;
			motor[driveLeft1] = speed;

		}
		if(abs(nMotorEncoder[driveRight1]) < abs(nMotorEncoder[driveLeft1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = speed;
			motor[driveLeft1] = speed+10;

		}
		else if(abs(nMotorEncoder[driveRight1]) > abs(nMotorEncoder[driveLeft1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = speed+10;
			motor[driveLeft1] = speed;

		}
	}
	//Stop the robot
	motor[driveRight1] = 0;
	motor[driveLeft1] = 0;
}

//Turn the robot left for the specified encoder counts
//at a specified speed
void turnLeft(int encoderCounts, int speed)
{
	//Clear the encoders before using them
	clearEncoders();
	//While the absolute value of the right motor's encoder is less
	//than the specified amount
	while(abs(nMotorEncoder[driveLeft1]) < encoderCounts)
	{
		//If the two encoder values are equal
		if(abs(nMotorEncoder[driveLeft1]) == abs(nMotorEncoder[driveLeft1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = speed;
			motor[driveLeft1] = -speed;

		}
		if(abs(nMotorEncoder[driveLeft1]) < abs(nMotorEncoder[driveRight1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = speed;
			motor[driveLeft1] = -(speed+10);

		}
		else if(abs(nMotorEncoder[driveLeft1]) > abs(nMotorEncoder[driveRight1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = speed+10;
			motor[driveLeft1] = -speed;

		}
	}
	//Stop the robot
	motor[driveRight1] = 0;
	motor[driveLeft1] = 0;
}

//Turn the robot left for the specified encoder counts
//at a specified speed
void turnRight(int encoderCounts, int speed)
{

	//Clear the encoders
	clearEncoders();

	//While the absolute value of the left motor's encoder is less
	//than the specified amount

	//Turn the robot to the right at the specified speed
	while(abs(nMotorEncoder[driveRight1]) < encoderCounts)
	{
		//If the two encoder values are equal
		if(abs(nMotorEncoder[driveRight1]) == abs(nMotorEncoder[driveLeft1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = -speed;
			motor[driveLeft1] = speed;

		}
		if(abs(nMotorEncoder[driveRight1]) < abs(nMotorEncoder[driveLeft1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = -speed;
			motor[driveLeft1] = speed+10;

		}
		else if(abs(nMotorEncoder[driveRight1]) > abs(nMotorEncoder[driveLeft1]))
		{
			//Move the robot forward at the specified speed
			motor[driveRight1] = -(speed+10);
			motor[driveLeft1] = speed;

		}
	}
	//Stop the robot
	motor[driveRight1] = 0;
	motor[driveLeft1] = 0;
}

void slowDown()
{
	updatePID = 0;
	stopTask(pidController);
	stopTask(music);
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
	stopTask(pidController);
	for(int i = 0; i <= startspeed; i+= 2)
	{
		motor[left1] = i;
		motor[left2] = i;
		motor[right1] = i;
		motor[right2] = i;
		wait1Msec(50);
	}
}


void driverLoads(int rSS, int timeToShoot) {
	requestedShooterSpeed = rSS;
	flyStartup = 1;
	pid_Kp = 0.05;
	pid_Ki = 0.0;
	pid_Kd = 0.4;

	pid_Kp_right = 0.05;
	pid_Ki_right = 0.0;
	pid_Kd_right = 0.41;
	flyVelocityL = 0;
	flyVelocityR = 0;
	updatePID = 1;
	bPlaySounds = true;
	if(rSS > 400) {
		pid_Kp_selected = 0.0235;
		pid_Ki_selected = 0.0;
		pid_Kd_selected = 0.758;

		pid_Kp_selectedR = 0.02;
		pid_Ki_selectedR = 0.0;
		pid_Kd_selectedR = 0.857;
	}
	else if(rSS == 380) {
		pid_Kp_selected = 0.028;
		pid_Ki_selected = 0.0;
		pid_Kd_selected = 0.96;

		pid_Kp_selectedR = 0.025;
		pid_Ki_selectedR = 0.0;
		pid_Kd_selectedR = 0.98;
	}
	else {
		pid_Kp_selected = 0.028;
		pid_Ki_selected = 0.0;
		pid_Kd_selected = 1;

		pid_Kp_selectedR = 0.025;
		pid_Ki_selectedR = 0.0;
		pid_Kd_selectedR = 1;
	}
	startTask(pidController);
	clearTimer(T2);
	while(time1[T2] < 1900)
	{
		motor[left1] = pidDriveL;
		motor[left2] = pidDriveL;
		motor[right1] = pidDriveR;
		motor[right2] = pidDriveR;
	}
	motor[up] = 70;
	motor[intake] = 110;
	motor[intake2] = 110;
	clearTimer(T2);
	while(time1[T2] < timeToShoot)
	{
		motor[left1] = pidDriveL;
		motor[left2] = pidDriveL;
		motor[right1] = pidDriveR;
		motor[right2] = pidDriveR;
	}
	stopTask(pidController);
	slowDown();
}

void leftLeftAuton() {
	//partly tested
	drive(1450, 70);
	wait1Msec(100);
	turnLeft(460, 70);
	motor[intake] = 100;
	motor[intake2] = 100;
	motor[up] = 0;
	drive(500, 70);
	drive(450, 70);
	///* Forward/backwards
	wait1Msec(1000);
	drive(300, -70);
	wait1Msec(1000);
	drive(300, 70);
	//*/ //end Forward/backwards
	wait1Msec(1000);
	drive(1000, -70);
	turnRight(560, 70);
	wait1Msec(100);
	driverLoads(375, 4500);
	motor[up] = 0;
	motor[intake] = 0;
	motor[intake2] = 0;
}

void leftBackAuton() {
	driverLoads(460, 6000);
	motor[up] = 100;
	motor[intake] = 100;
	motor[intake2] = 100;
	wait1Msec(7000);
	motor[up] = 0;
	motor[intake] = 0;
	motor[intake2] = 0;
}

void rightRightAuton() {
	//not tested, wrong values (using left values)
	driverLoads(460, 4500);
	drive(1500, 70);
	turnRight(550, 70);
	drive(500, 70);
	motor[up] = 100;
	motor[intake] = 100;
	motor[intake2] = 100;
	drive(500, 70);
	/* Forward/backwards
	wait1Msec(1000);
	drive(500, -70);
	wait1Msec(1000);
	drive(500, 70);
	*/ //end Forward/backwards
	wait1Msec(1000);
	drive(1000, -70);
	turnLeft(490, 70);
	wait1Msec(100);
	driverLoads(380, 4500);
	motor[up] = 0;
	motor[intake] = 0;
	motor[intake2] = 0;
}

void rightBackAuton() {
	driverLoads(460, 6000);
	motor[up] = 100;
	motor[intake] = 100;
	motor[intake2] = 100;
	wait1Msec(7000);
	motor[up] = 0;
	motor[intake] = 0;
	motor[intake2] = 0;
}

void autonSelector() {
	if(SensorValue[DNJ] == 0 && SensorValue[UPJ] == 0) {
		leftLeftAuton();
	}
	else if(SensorValue[DNJ] == 1 && SensorValue[UPJ] == 1) {
		driverLoads(430, 10000);
	}
	else if(SensorValue[DNJ] == 0 && SensorValue[UPJ] == 1) {
		rightRightAuton();
	}
}




#endif
