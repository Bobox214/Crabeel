#include <math.h>
#include "MeMegaPi.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PID.h"
#include "BaseLocalizer.h"

#define RAD_TO_DEGREE 57.2957795   // 180/3.141592
#define DEGREE_TO_RAD 0.0174532925 // 3.141592/180

#define BASE_WIDTH 0.173
#define WHEEL_RADIUS 0.064

Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler;
imu::Vector<3> rotation;

BaseLocalizer base = BaseLocalizer(SLOT2,SLOT1);

PID  pid;
int pwm,zPwm;
int lMotorPwm;
int rMotorPwm;

double  pitch,goalPitch,velPitch;
double  goalYaw;
double  goalX,goalY;

double vMax = 0.05;
double wMax = 1;
double w_kP = 1;

double kP = 16;
double kI = 10;
double kD = 0.3;
bool   initialized = false;
unsigned long time;
unsigned long lastTime;
unsigned long lastPrintStateTime;
double   dt;

enum eState                {  IDLE  ,  INITIALIZED  ,  BALANCE  ,  GO_TO_START  ,  TURN_TO_START  };
const char * eStateStr[] = { "IDLE" , "INITIALIZED" , "BALANCE" , "GO_TO_START" , "TURN_TO_START" };
eState state;

void waitBnoCalibration() {
	uint8_t system, gyro, accel, mag;
	Serial3.println("*** Calibration ***");
	while (1) {
		system = gyro = accel = mag = 0;
		bno.getCalibration(&system, &gyro, &accel, &mag);
		if (system>1) break;
		delay(500);
	}
	Serial3.println("*** Calibration Done ***");
	euler    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	rotation = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	goalYaw = euler.x()*DEGREE_TO_RAD;
}

void setup() {
	Serial.begin(115200);
	Serial3.begin(115200);
	pid.setDebug(false);
	pid.setCoefficients(kP,kI,kD);
	pid.setOutputRange(-255,255,0);
	goalPitch = 0;
	reset();
	printState();
	// BNO055
	if(!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	delay(1000);
	bno.setExtCrystalUse(true);
	waitBnoCalibration();
	base.setParameters(BASE_WIDTH,WHEEL_RADIUS,360);
	base.setDebug(true);
}
void printState() {
	Serial3.print("STATE :");
	Serial3.print(eStateStr[state]);
	//Serial3.print(" PID P:");
	//Serial3.print(kP);
	//Serial3.print(" I:");
	//Serial3.print(kI);
	//Serial3.print(" D:");
	//Serial3.print(kD);
	//Serial3.print(" Target:");
	//Serial3.print(goalPitch);
	Serial3.print(" yaw:");
	Serial3.print(euler.x());
	Serial3.print(" roll:");
	Serial3.print(euler.y());
	Serial3.print(" pitch:");
	Serial3.print(euler.z());
	Serial3.print(" X:");
	Serial3.print(base.x());
	Serial3.print(" Y::");
	Serial3.println(base.y());
	lastPrintStateTime = millis();
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
	if (cmd>=0 or cmd<=2)
		pid.setCoefficients(kP,kI,kD);
}
}
void bluetoothLoop() {
	if (Serial3.available()) {
		int v = Serial3.read();
		if (cmd==-1) {
			if (v=='p')
				cmd = 0;
			else if (v=='i')
				cmd = 1;
			else if (v=='d')
				cmd = 2;
			else if (v=='s') {
				if (state == IDLE) {
					state = BALANCE;
				} else {
					reset();
				}
			} else if (v=='t')
				goalPitch = pitch;
			else if (v=='r') {
				goalYaw = euler.x()*DEGREE_TO_RAD;
				goalX   = base.x();
				goalY   = base.y();
			} else if (v=='b') {
				if (state==IDLE) {
					state = GO_TO_START;
				}
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
	pid.reset();
	Serial3.println("*** Reset ***");
	state = IDLE;
}


void loop()
{
	bluetoothLoop();
	euler    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	rotation = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	pitch = euler.z();
	velPitch  = rotation.z()*RAD_TO_DEGREE;
	base.updatePosition(euler.x()*DEGREE_TO_RAD);


	lastTime = time;
	time = millis();
	dt = (time-lastTime)/1000.0;

	switch (state) {
			case IDLE        : break;
			case BALANCE : {
				if (abs(pitch)>15) {
					// Too much tilted. Stop
					Serial3.println("*** Stop ***");
					reset();
					state = IDLE;
				} else {
					pwm = pid.update(pitch-goalPitch,velPitch,dt);
					lMotorPwm = pwm;
					rMotorPwm = pwm;
				}
				break;
			}
			case GO_TO_START : {
				double errX = goalX-base.x();
				double errY = goalY-base.y();
				Serial3.print("     errX:");
				Serial3.print(errX);
				Serial3.print(" errY:");
				Serial3.print(errY);
				if (errX*errX<0.04 and errY*errY<0.04) {
					Serial3.println(" -> TURN_TO_START");
					state = TURN_TO_START;
				} else {
					double v,w;
					double errYaw = atan2(errY,errX)-base.yaw();
					Serial3.print(" errYaw:");
					Serial3.print(errYaw);
					errYaw = atan2(sin(errYaw),cos(errYaw));
					Serial3.print(" errYaw:");
					Serial3.print(errYaw);
					w = constrain(w_kP*errYaw,-wMax,wMax);
					if (abs(errYaw)<0.75) {
						v = -vMax/pow(abs(w)+1,0.5);
					} else {
						v = 0;
					}
					// Convert v,w to motorPwm
					rMotorPwm = 50*(v+w*BASE_WIDTH/2)/vMax;
					lMotorPwm = 50*(v-w*BASE_WIDTH/2)/vMax;
					Serial3.print(" w:");
					Serial3.print(w);
					Serial3.print(" v:");
					Serial3.print(v);
					Serial3.print(" lPwm:");
					Serial3.print(lMotorPwm);
					Serial3.print(" rPwm:");
					Serial3.print(rMotorPwm);
					Serial3.println();
				}
				break;
			}
			case TURN_TO_START : {
				double errYaw = goalYaw-base.yaw();
				errYaw = atan2(sin(errYaw),cos(errYaw));
				Serial3.print(" errYaw:");
				Serial3.print(errYaw);
				if (errYaw*errYaw<0.05) {
					reset();
					Serial3.println(" -> IDLE");
					state = IDLE;
				} else {
					double w = constrain(w_kP*errYaw,-wMax,wMax);
					// Convert v,w to motorPwm
					rMotorPwm = 50*w*(BASE_WIDTH/2)/vMax;
					lMotorPwm = 50*-w*(BASE_WIDTH/2)/vMax;
					Serial3.print(" rPwm:");
					Serial3.print(rMotorPwm);
					Serial3.print(" lPwm:");
					Serial3.print(lMotorPwm);
					Serial3.println();
				}
				break;
			}
		}
		if (time-lastPrintStateTime>1000)
			printState();

		base.setMotorPwm(lMotorPwm,rMotorPwm);

		delay(10);
}

