#include <math.h>
#include "MeMegaPi.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PID.h"
#include "BaseLocalizer.h"

#define abs(x) ((x)>0?(x):-(x)) //math.h overwrite abs
#define RAD_TO_DEGREE 57.2957795   // 180/3.141592
#define DEGREE_TO_RAD 0.0174532925 // 3.141592/180

#define BASE_WIDTH 0.173
#define WHEEL_RADIUS 0.064

Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler;
imu::Vector<3> rotation;

BaseLocalizer base = BaseLocalizer(SLOT2,SLOT1);

PID  pid;

double  pitch,goalPitch,velPitch;
double  goalYaw;
double  goalX,goalY;

double vMax = 0.5;
double wMax = 4;
double w_kP = 4;

double kP = 16;
double kI = 0;
double kD = 0.3;
bool   initialized = false;
unsigned long time;
unsigned long lastTime;
double   dt;

enum eState                {  IDLE  ,  INITIALIZED  ,  BALANCE  ,  GO_TO_START  ,  TURN_TO_START  };
const char * eStateStr[] = { "IDLE" , "INITIALIZED" , "BALANCE" , "GO_TO_START" , "TURN_TO_START" };
eState state;

void waitBnoCalibration() {
	uint8_t system, gyro, accel, mag;
	Serial3.println("*** Calibration ***");
	while (true) {
		system = gyro = accel = mag = 0;
		bno.getCalibration(&system, &gyro, &accel, &mag);
		if (system>1) break;
		delay(500);
	}
	Serial3.println("*** Calibration Done ***");
	euler    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	rotation = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	goalYaw = (euler.x()-180)*DEGREE_TO_RAD;
}

void setup() {
	Serial.begin(115200);
	Serial3.begin(115200);
	pid.setDebug(false);
	pid.setCoefficients(kP,kI,kD);
	pid.setOutputRange(-255,255,0);
	goalPitch = 0;
	reset();
	state = IDLE;
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
	base.setDebug(false);
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
	Serial3.print(" Base x:");
	Serial3.print(base.x());
	Serial3.print(" y:");
	Serial3.print(base.y());
	Serial3.print(" yaw:");
	Serial3.println(base.yaw());
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
					printState();
				} else {
					reset();
					state = IDLE;
					printState();
				}
			} else if (v=='t')
				goalPitch = pitch;
			else if (v=='r') {
				Serial3.println("*** Reset starting position ***");
				goalYaw = (euler.x()-180)*DEGREE_TO_RAD;
				goalX   = base.x();
				goalY   = base.y();
			} else if (v=='b') {
				if (state==IDLE) {
					state = GO_TO_START;
					printState();
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
	pid.reset();
	base.stop();
	Serial3.println("*** Reset ***");
}


void loop()
{
	bluetoothLoop();
	euler    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	rotation = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	pitch = euler.z();
	velPitch  = rotation.z()*RAD_TO_DEGREE;
	base.loop((euler.x()-180)*DEGREE_TO_RAD);


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
					printState();
				} else {
					int pwm = pid.update(pitch-goalPitch,velPitch,dt);
					base.setMotorPwm(pwm,pwm);
				}
				break;
			}
			case GO_TO_START : {
				double errX = goalX-base.x();
				double errY = goalY-base.y();
				//Serial3.print("GO_TO_START from x:");
				//Serial3.print(base.x());
				//Serial3.print(" y:");
				//Serial3.print(base.y());
				//Serial3.print(" to x:");
				//Serial3.print(goalX);
				//Serial3.print(" y:");
				//Serial3.print(goalY);
				//Serial3.print("  -->   errX:");
				//Serial3.print(errX,6);
				//Serial3.print(" errY:");
				//Serial3.print(errY,6);
				if (errX*errX<0.0025 && errY*errY<0.0025) {
					//Serial3.println(" --> TURN_TO_START");
					state = TURN_TO_START;
					printState();
				} else {
					double v,w;
					double lclGoalYaw = atan2(errY,errX);
					double errYaw = lclGoalYaw-base.yaw();
					errYaw = atan2(sin(errYaw),cos(errYaw)); // Normalize
					//Serial3.print(" yaw:");
					//Serial3.print(base.yaw());
					//Serial3.print(" goalYaw:");
					//Serial3.print(lclGoalYaw);
					//Serial3.print(" --> errYaw:");
					//Serial3.print(errYaw);
					w = constrain(w_kP*errYaw,-wMax,wMax);
					if (abs(errYaw)<0.25) {
						v = vMax/pow(abs(w)+1,0.5);
					} else {
						v = 0;
					}
					// Convert v,w to motorPwm
					base.setSpeed(v,w);
					//Serial3.print(" --> w:");
					//Serial3.print(w);
					//Serial3.print(" v:");
					//Serial3.print(v);
					//Serial3.println();
				}
				break;
			}
			case TURN_TO_START : {
				double errYaw = goalYaw-base.yaw();
				errYaw = atan2(sin(errYaw),cos(errYaw));
				//Serial3.print("TURN_TO_START from yaw:");
				//Serial3.print(base.yaw(),6);
				//Serial3.print(" to yaw:");
				//Serial3.print(goalYaw,6);
				//Serial3.print(" --> errYaw:");
				//Serial3.print(errYaw,6);
				if (errYaw*errYaw<0.0025) {
					//Serial3.println(" --> IDLE");
					reset();
					state = IDLE;
					printState();
				} else {
					double w = constrain(w_kP*errYaw,-wMax,wMax);
					// Convert v,w to motorPwm
					//Serial3.print(" --> w:");
					//Serial3.print(w,6);
					//Serial3.println();
					base.setSpeed(0,w);
				}
				break;
			}
		}
		delay(20);
}

