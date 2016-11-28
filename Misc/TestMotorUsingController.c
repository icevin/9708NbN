#pragma config(Motor,  port1,           nonReversed,   tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port10,          reversed,      tmotorServoContinuousRotation, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//LEFT IS PORT 10, RIGHT IS PORT 1 (NON REVERSED)
task main()
{

	int x=100;

	while(true)
	{
		if(vexRT[Btn8L] == 1)
		{
			motor[reversed] = 0;  // stop 'rightMotor'
			motor[nonReversed]  = 0;  // stop 'leftMotor'
		}

		else if(vexRT[Btn8D] == 1)
		{
			wait1Msec(500);
			motor[reversed] = x;  // start 'rightMotor'
			motor[nonReversed]  = x;  // start 'leftMotor'
		}
	}
}