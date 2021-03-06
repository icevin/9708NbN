#pragma config(Sensor, dgtl8,  sonarSensor,    sensorSONAR_cm)
#pragma config(Motor,  port2,           rightMotor,    tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port3,           leftMotor,     tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port4,           rightMotor2,   tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port5,           leftMotor2,    tmotorServoContinuousRotation, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	wait1Msec(2000);						// Robot waits for 2000 milliseconds before executing program
	while(true);
	{
		while(SensorValue(sonarSensor) == -1);
		{
			motor[rightMotor]  =32;
			motor[leftMotor]   =32;
		}
		else

		wait1Msec(2000);						// Robot waits for 2000 milliseconds before executing program

		motor[rightMotor] = -32;		  // Motor on port2 is run at full (-127) power reverse
		motor[leftMotor]  = 32;			// Motor on port3 is run at full (127) power forward
		wait1Msec(750);					      // Robot runs previous code for 750 milliseconds before moving on
	}

	wait1Msec(750);					      // Robot runs previous code for 750 milliseconds before moving on
}													      // Program ends, and the robot stops
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
