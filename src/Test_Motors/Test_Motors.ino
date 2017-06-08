#include <MeMegaPi.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);
MeEncoderOnBoard Encoder_4(SLOT4);
long motorNb = 1;
long motorPwm;

void printState() {
	Serial3.print("Testing Motor ");
	Serial3.print(motorNb);
	Serial3.print(" Target Pwm : ");
	Serial3.print(motorPwm);
	Serial3.print(" Pwm : ");
	if (motorNb==1)
		Serial3.print(Encoder_1.getCurPwm());
	if (motorNb==2)
		Serial3.print(Encoder_2.getCurPwm());
	if (motorNb==3)
		Serial3.print(Encoder_3.getCurPwm());
	if (motorNb==4)
		Serial3.print(Encoder_3.getCurPwm());
	Serial3.println();
}


void reset() {
	motorPwm = 0;
	Encoder_1.setMotorPwm(0);
	Encoder_2.setMotorPwm(0);
	Encoder_3.setMotorPwm(0);
	Encoder_4.setMotorPwm(0);
}

void bluetoothLoop() {
	if (Serial3.available()) {
		int v = Serial3.read();
		if (v=='z')
			motorPwm += 5;
		else if (v=='s')
			motorPwm -= 5;
		else if (v=='d')
			motorPwm = 0;
		else if (v=='1') {
			reset();
			motorNb = 1;
		} else if (v=='2') {
			reset();
			motorNb = 2;
		} else if (v=='3') {
			reset();
			motorNb = 3;
		} else if (v=='4') {
			reset();
			motorNb = 4;
		}
		printState();
	}
}

void setup()
{
	Serial.begin(115200);
	Serial3.begin(115200);
	reset();
	printState();
}

void loop()
{
	bluetoothLoop();
	if (motorNb==1) Encoder_1.setMotorPwm(motorPwm);
	if (motorNb==2) Encoder_2.setMotorPwm(motorPwm);
	if (motorNb==3) Encoder_3.setMotorPwm(motorPwm);
	if (motorNb==4) Encoder_4.setMotorPwm(motorPwm);
	delay(10);
}
