#include "MeMegaPi.h"
#include "Wire.h"
#include "Arm.h"

MeEncoderOnBoard Encoder_B(SLOT1);
MeEncoderOnBoard Encoder_F(SLOT2);

CrabeelArm arm;

PID  pid;
int pwm,zPwm;
int armMotorPwm;
int lMotorPwm;
int rMotorPwm;

double  angleX,goalAngleX;
double  angleZ,goalAngleZ;

double kP = 30;
double kI = 5;
double kD = 0.2;
double target = 0;
bool   start  = false;
bool   fast   = false;

unsigned long time;
unsigned long lastTime;
int     armAngle;
double   dt;
MeGyro gyro;

void setup() {
	Serial.begin(115200);
	Serial3.begin(115200);
	gyro.begin();
	pid.setDebug(true);
	pid.setCoefficients(kP,kI,kD);
	pid.setOutputRange(-255,255,20);
	arm.setup();
	arm.setDebug(false);
	reset();
	printState();
}

void printState() {
	Serial3.print("PID P:");
	Serial3.print(kP);
	Serial3.print(" I:");
	Serial3.print(kI);
	Serial3.print(" D:");
	Serial3.print(kD);
	Serial3.print(" Target:");
	Serial3.print(target);
	Serial3.print(" AngleX:");
	Serial3.print(gyro.getAngleX());
	Serial3.print(" gyroX:");
	Serial3.print(gyro.getGyroX());
	Serial3.print(" AngleZ:");
	Serial3.print(gyro.getAngleZ());
	Serial3.print(" Arm.Angle:");
	Serial3.println(arm.getAngle());
}

int cmd = -1;
int idx = 0;
int val[10];
void processCmd() {
	int value = 0;
	if (idx>0) {
		bool inv = false;
		for (int i=0;i<idx;i++) {
			if (val[i]=='-')
				inv = true;
			else if ('9'>=val[i] && val[i]>='0')
				value = value*10 + (val[i]-'0');
		}
		if (inv) value = -value;
		if (cmd==0)
			kP = value/10.0;
		else if (cmd==1)
			kI = value/10.0;
		else if (cmd==2)
			kD = value/10.0;
		else if (cmd==3)
			target = value/10.0;
		if (cmd>=0 or cmd<=2)
			pid.setCoefficients(kP,kI,kD);
	}
}

void bluetoothLoop() {
	if (Serial3.available()) {
		int v = Serial3.read();
		if (cmd==-1) {
			if (v=='a') {
				armAngle += 2;
				arm.goToAngle(armAngle);
			} else if (v=='e') {
				armAngle -= 2;
				arm.goToAngle(armAngle);
			} else if (v=='p')
				cmd = 0;
			else if (v=='i')
				cmd = 1;
			else if (v=='d')
				cmd = 2;
			else if (v=='t')
				cmd = 3;
			else if (v=='f') {
				fast = not fast;
				Serial3.println(fast?"Fast update":"Std update");
			} else if (v=='s') {
				start = not start;
				Serial3.println(start?"*** Start ***":"*** Stop ***");
				if (not start)
					reset();
			}
		} else {
			if (v == ';') {
				processCmd();
				cmd = -1;
				idx = 0;
			} else {
				val[idx] = v;
				idx += 1;
			}
		}
		if (cmd==-1)
			printState();
	}
}

void reset() {
	lMotorPwm  = 0;
	rMotorPwm  = 0;
	goalAngleX = 0;
	goalAngleZ = gyro.getAngleZ();
	pid.reset();
	armMotorPwm = 0;
	arm.reset();
	Serial3.println("*** Reset ***");
	armAngle = arm.getAngle();
}

void loop()
{
	bluetoothLoop();
	arm.loop();
	if (fast)
		gyro.fast_update();
	else
		gyro.update();
	angleX = gyro.getAngleX();
	angleZ = gyro.getAngleZ();

	lastTime = time;
	time = millis();
	dt = 1;//(time-lastTime)/1000.0;
	
	if (start) {
		if (abs(angleX)>10) {
			// Too much tilted. Stop
			start = false;
			reset();
			Serial3.println("*** Stop ***");
		} else {
			pwm = pid.update(angleX-goalAngleX,dt);
			zPwm = 0;
			//zPwm = (angleZ-goalAngleZ)*10;
			//zPwm = constrain(zPwm,-50,50);
			lMotorPwm = pwm-zPwm;
			rMotorPwm = pwm+zPwm;
		}
	}
		
	rMotorPwm = constrain(rMotorPwm,-255,255);
	lMotorPwm = constrain(lMotorPwm,-255,255);

	Encoder_B.setMotorPwm(lMotorPwm);
	Encoder_F.setMotorPwm(-rMotorPwm);

	delay(10);
}

