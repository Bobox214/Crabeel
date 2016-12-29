#include "MeMegaPi.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PID.h"

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> euler;
imu::Vector<3> rotation;


MeEncoderOnBoard Encoder_B(SLOT1);
MeEncoderOnBoard Encoder_F(SLOT2);

PID  pid;
int pwm,zPwm;
int lMotorPwm;
int rMotorPwm;

double  angleX,goalAngleX,gyroX;

double kP = 16;
double kI = 10;
double kD = 0.3;
bool   initialized = false;
unsigned long time;
unsigned long lastTime;
unsigned long lastPrintStateTime;
double   dt;

enum eState                {  IDLE  ,  INITIALIZED  ,  BALANCE  ,  RESET  };
const char * eStateStr[] = { "IDLE" , "INITIALIZED" , "BALANCE" , "RESET" };
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

}

void setup() {
Serial.begin(115200);
Serial3.begin(115200);
pid.setDebug(true);
pid.setCoefficients(kP,kI,kD);
pid.setOutputRange(-255,255,0);
goalAngleX = 0;
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
}

void printState() {
Serial3.print("STATE :");
Serial3.print(eStateStr[state]);
Serial3.print(" PID P:");
Serial3.print(kP);
Serial3.print(" I:");
Serial3.print(kI);
Serial3.print(" D:");
Serial3.print(kD);
Serial3.print(" Target:");
Serial3.print(goalAngleX);
Serial3.print(" BNO X :");
Serial3.print(euler.x());
Serial3.print(" BNO Y :");
Serial3.print(euler.y());
Serial3.print(" BNO z :");
Serial3.print(euler.z());
Serial3.print(" BNO roll:");
Serial3.print(rotation.x());
Serial3.print(" BNO pitch:");
Serial3.println(rotation.y());
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
		else if (v=='m')
			cmd = 3;
		else if (v=='s') {
			if (state == IDLE) {
				state = BALANCE;
			} else {
				reset();
			}
		} else if (v=='t')
			goalAngleX = angleX;
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
angleX = euler.z();
gyroX  = rotation.z()*180/3.141592;


lastTime = time;
time = millis();
dt = (time-lastTime)/1000.0;

switch (state) {
	case IDLE        : break;
	case BALANCE : {
		if (abs(angleX)>15) {
			// Too much tilted. Stop
			reset();
			Serial3.println("*** Stop ***");
		} else {
			pwm = pid.update(angleX-goalAngleX,gyroX,dt);
				lMotorPwm = pwm;
				rMotorPwm = pwm;
			}
			break;
		}
		case RESET: { reset(); break; }
	}
	if (time-lastPrintStateTime>1000)
		printState();
		
	rMotorPwm = constrain(rMotorPwm,-255,255);
	lMotorPwm = constrain(lMotorPwm,-255,255);

	Encoder_B.setMotorPwm(lMotorPwm);
	Encoder_F.setMotorPwm(-rMotorPwm);

	delay(10);
}

