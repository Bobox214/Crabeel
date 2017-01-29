#include <math.h>
#include "MeMegaPi.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PID.h"
#include "Base.h"
#include "Generation.h"

#define abs(x) ((x)>0?(x):-(x)) //math.h overwrite abs
#define RAD_TO_DEGREE 57.2957795   // 180/3.141592
#define DEGREE_TO_RAD 0.0174532925 // 3.141592/180
#define TIP_PITCH 17

#define BASE_WIDTH 0.173
#define WHEEL_RADIUS 0.064

Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler;

Base base = Base(SLOT2,SLOT1);

PID  pid;

double  pitch;
double  goalYaw;
double  goalX,goalY;

double vMax = 0.5;
double wMax = 4;
double w_kP = 4;


Generation    *currentGeneration;
uint8_t        currentConfIdx;
Configuration *currentConf;

unsigned long autoStart_timeOut  = 1000;
uint8_t       autoStart_count;

unsigned long curTime;
unsigned long lastTime;
unsigned long stateEnterTime; // Time in millis when current state was entered
unsigned long balanceTime; // Time in millis when current state was entered

enum eState                {  IDLE  ,  CALIBRATION  ,  AUTO_START  ,  BALANCE  ,  SCORE_CONF  ,  GO_ORIGIN  };
const char * eStateStr[] = { "IDLE" , "CALIBRATION" , "AUTO_START" , "BALANCE" , "SCORE_CONF" , "GO_ORIGIN" };
eState state;
uint8_t nbScore;

void setCurrentConf(uint8_t idx) {
	currentConfIdx = idx;
	currentConf = currentGeneration->configuration[currentConfIdx];
}

void setup() {
	Serial.begin(115200);
	Serial3.begin(115200);
	Serial3.println("***************************");
	currentGeneration = new Generation();
	setCurrentConf(0);
	currentConf->setCoefficients(
		16.0,0.0,0.3
	,	0.0
	,	250,220
	,	0
	);
	pid.setDebug(false);
	reset();
	// BNO055
	if(!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	delay(1000);
	bno.setExtCrystalUse(true);
	base.setParameters(BASE_WIDTH,WHEEL_RADIUS,360);
	base.setDebug(false);
	setState(CALIBRATION);
}

void setState(eState _state) {
	if (_state==AUTO_START && state!=AUTO_START) {
		autoStart_count = 0;
	} else {
		autoStart_count += 1;
	}
	state = _state;
	if (state==IDLE) reset();
	if (state==BALANCE) {
		pid.setCoefficients(currentConf->kP,currentConf->kI,currentConf->kD);
		pid.setOutputRange(-255,255,currentConf->neutralZone);
	}
	stateEnterTime = millis();
	printState();
}

void printOrigin() {
	Serial3.print("# Origin - x:");
	Serial3.print(goalX,3);
	Serial3.print(" y:");
	Serial3.print(goalY,3);
	Serial3.print(" yaw:");
	Serial3.println(goalYaw);
}

void printState() {
	Serial3.print(millis()/1000.0,2);
	Serial3.print(" ");
	Serial3.print(eStateStr[state]);
	if (state==SCORE_CONF) {
		Serial3.print(" ");
		Serial3.print(nbScore);
	}
	if (state==AUTO_START) {
		Serial3.print(" ");
		Serial3.print(autoStart_count);
	}
	Serial3.println();
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
		if (cmd==0) {
			currentConf->kP = value/10.0;
			currentConf->print();
		} else if (cmd==1) {
			currentConf->kI = value/10.0;
			currentConf->print();
		} else if (cmd==2) {
			currentConf->kD = value/10.0;
			currentConf->print();
		} else if (cmd==3) {
			currentGeneration->saveToEEPROM(value,true);
		} else if (cmd==4) {
			currentGeneration->loadFromEEPROM(value,true);
		} else if (cmd==5) {
			setCurrentConf(value);
		}
		if (cmd>=0 or cmd<=2)
			pid.setCoefficients(currentConf->kP,currentConf->kI,currentConf->kD);
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
			else if (v=='e')
				cmd = 3;
			else if (v=='l')
				cmd = 4;
			else if (v=='f')
				cmd = 5;
			else if (v=='a') {
				if (state == IDLE) {
					setState(AUTO_START);
				}
			} else if (v=='q') {
				setState(SCORE_CONF);
			} else if (v=='s') {
				if (state == IDLE) {
					setState(BALANCE);
				} else {
					setState(IDLE);
				}
			} else if (v=='c') {
				currentGeneration->saveToEEPROM(0);
				currentGeneration->print();
			} else if (v=='v') {
				currentGeneration->loadFromEEPROM(0);
				currentGeneration->print();
			} else if (v==' ') {
				printState();
				Serial3.print(" ");
				Serial3.print(currentConfIdx);
				currentConf->print();
				currentGeneration->print();
			} else if (v=='t') {
				currentConf->balancePitch = pitch;
				currentConf->print();
			}
			else if (v=='r') {
				currentGeneration->randomize();
				currentGeneration->print();
			} else if (v=='o') {
				if (state==IDLE) {
					setState(GO_ORIGIN);
				}
			} else if (v=='h') {
				currentGeneration->debugCreateScores();
				currentGeneration->print();
			} else if (v=='g') {
				currentGeneration->breed();
				currentGeneration->print();
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
	}
}

void reset() {
	pid.reset();
	base.stop();
	nbScore = 0;
}


void loop()
{

	bluetoothLoop();
	euler    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	pitch = euler.z();
	base.loop(euler.x()*DEGREE_TO_RAD);

	lastTime = curTime;
	curTime = millis();


	switch (state) {
			case IDLE        : break;
			case CALIBRATION : {
				uint8_t system, gyro, accel, mag;
				system = gyro = accel = mag = 0;
				bno.getCalibration(&system, &gyro, &accel, &mag);
				if (system>1) {
					euler    = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
					goalYaw = euler.x()*DEGREE_TO_RAD;
					setState(IDLE);
				}
				break;
			}
			case AUTO_START : {
				if ( curTime-stateEnterTime < currentConf->autoStartDuration ) {
					int motorPwm = currentConf->autoStartPwm*(((double)(curTime-stateEnterTime))/currentConf->autoStartDuration);
					if (pitch>0) motorPwm = -motorPwm;
					base.setMotorPwm(motorPwm,motorPwm);
				} else {
					base.setMotorPwm(0,0);
					if (abs(pitch)<TIP_PITCH) {
						setState(BALANCE);
					}
					if (curTime-stateEnterTime > currentConf->autoStartDuration+autoStart_timeOut ) {
						if (autoStart_count<3) 
							setState(AUTO_START);
						else
							setState(BALANCE); // Force moving on
					}
				}
				break;
			}
			case BALANCE : {
				if (abs(pitch)>TIP_PITCH) {
					// Too much tilted
					balanceTime = curTime-stateEnterTime;
					if (nbScore==0) {
						setState(IDLE);
					} else {
						setState(SCORE_CONF);
					}
				} else {
					double dt = (curTime-lastTime)/1000.0;
					imu::Vector<3> rotation = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
					double velPitch  = rotation.z()*RAD_TO_DEGREE;
					int pwm = pid.update(pitch-currentConf->balancePitch,velPitch,dt);
					base.setMotorPwm(pwm,pwm);
				}
				break;
			}
			case GO_ORIGIN : {
				double errX = goalX-base.x();
				double errY = goalY-base.y();
				//Serial3.print("GO_ORIGIN from x:");
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
					double errYaw = goalYaw-base.yaw();
					errYaw = atan2(sin(errYaw),cos(errYaw));
					//Serial3.print("from yaw:");
					//Serial3.print(base.yaw(),6);
					//Serial3.print(" to yaw:");
					//Serial3.print(goalYaw,6);
					//Serial3.print(" --> errYaw:");
					//Serial3.print(errYaw,6);
					//Serial3.println();
					if (errYaw*errYaw<0.0025) {
						if (nbScore==0 || nbScore>=NBSCORES) {
							setState(IDLE);
						} else {
							setState(AUTO_START);
						}
					} else {
						double w = constrain(w_kP*errYaw,-wMax,wMax);
						// Convert v,w to motorPwm
						//Serial3.print(" --> w:");
						//Serial3.print(w,6);
						//Serial3.println();
						base.setSpeed(0,w);
					}
				} else {
					int8_t vM;
					double v,w;
					double lclGoalYaw = atan2(errY,errX);
					double errYaw = lclGoalYaw-base.yaw();
					errYaw = atan2(sin(errYaw),cos(errYaw)); // Normalize
					if (abs(errYaw)>PI/2) {
						errYaw = atan2(sin(errYaw+PI),cos(errYaw+PI)); // Normalize
						vM = -1;
					} else {
						vM = 1;
					}
					//Serial3.print(" yaw:");
					//Serial3.print(base.yaw());
					//Serial3.print(" goalYaw:");
					//Serial3.print(lclGoalYaw);
					//Serial3.print(" --> errYaw:");
					//Serial3.print(errYaw);
					w = constrain(w_kP*errYaw,-wMax,wMax);
					if (abs(errYaw)<0.25) {
						v = vM*vMax/pow(abs(w)+1,0.5);
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
			case SCORE_CONF : {
				// Delay scoring to let times between auto starts
				base.stop();
				if (curTime-stateEnterTime>=500) {
					if (nbScore==0) {
						currentConf->resetScore();
						nbScore = 1;
						setState(AUTO_START);
					} else {
						currentConf->addScore(balanceTime);
						if (nbScore<NBSCORES) {
							nbScore += 1;
							if (abs(base.x()-goalX)>1 or abs(base.y()-goalY) > 1)
								setState(GO_ORIGIN);
							else
								setState(AUTO_START);
						} else {
							currentConf->compileScore();
							currentConf->print();
							if (currentConfIdx!=NBCONFS-1)
								setCurrentConf(currentConfIdx+1);
							setState(GO_ORIGIN);
						}
					}
				}
				break;
			}
		}
		delay(0);
}

