#include "MeMegaPi.h"

#define PI 3.141592

MeEncoderOnBoard *EncoderL;
MeEncoderOnBoard *EncoderR;

void processEncoderLInt(void) {
	if(digitalRead(EncoderL->getPortB()) == 0)
		EncoderL->pulsePosMinus();
	else
		EncoderL->pulsePosPlus();;
}
void processEncoderRInt(void) {
	if(digitalRead(EncoderR->getPortB()) == 0)
		EncoderR->pulsePosMinus();
	else
		EncoderR->pulsePosPlus();;
}

class Base {
	public:
		Base(uint8_t slotL,uint8_t slotR)
			:	debug(false)
			,	_x(0)
			,	_y(0)
			,	_yaw(0)
			,	kP(100)
			,	kI(1000)
			,	kD(0)
		{	EncoderL = new MeEncoderOnBoard(slotL);
			EncoderR = new MeEncoderOnBoard(slotR);
			attachInterrupt(EncoderL->getIntNum() , processEncoderLInt , RISING);
			attachInterrupt(EncoderR->getIntNum() , processEncoderRInt , RISING);
			pidL.setCoefficients(kP,kI,kD);
			pidL.setOutputRange(-255,255,20);
			pidL.setDebug(false);
			pidR.setCoefficients(kP,kI,kD);
			pidR.setOutputRange(-255,255,20);
			pidR.setDebug(false);
		}
		void setParameters(double baseWidth, double wheelRadius, int ticksPerRotation) {
			this->baseWidth        = baseWidth;
			this->wheelRadius      = wheelRadius;
			this->ticksPerRotation = ticksPerRotation;
		}
		void setPosition(double x, double y, double yaw) {
			this->_x   = x;
			this->_y   = y;
			this->_yaw = yaw;
		}
		void setDebug(bool debug) {
			this->debug = debug;
		}
		void loop(double yaw) {
			double dt = (millis()-loopTime)/1000.0;
			long newTicksL = -EncoderL->getPulsePos();
			long newTicksR =  EncoderR->getPulsePos();
			double dl = (2*PI*wheelRadius*(newTicksL-ticksL))/ticksPerRotation;
			double dr = (2*PI*wheelRadius*(newTicksR-ticksR))/ticksPerRotation;
			double df = (dr+dl)/2;
			double dx = df*cos(yaw);
			double dy = df*sin(yaw);
			double dyaw = atan2(sin(yaw-_yaw),cos(yaw-_yaw));
			curVR = dr/dt;
			curVL = dl/dt;
			curW  = dyaw/dt;
			if (debug && (newTicksL!=ticksL) && (newTicksR!=ticksR)) {
				Serial3.print("Base tl: ");
				Serial3.print(newTicksL);
				Serial3.print(" tr: ");
				Serial3.print(newTicksR);
				Serial3.print(" yaw: ");
				Serial3.print(yaw);
				Serial3.print(" dl: ");
				Serial3.print(dl,4);
				Serial3.print(" dr: ");
				Serial3.print(dr,4);
				Serial3.print(" -> dx: ");
				Serial3.print(dx,4);
				Serial3.print(" dy: ");
				Serial3.println(dy,4);
				Serial3.print("   Speed vL: ");
				Serial3.print(curVL,6);
				Serial3.print(" vR: ");
				Serial3.print(curVR);
				Serial3.print(" w: ");
				Serial3.println(curW);
			}
			_x += dx;
			_y += dy;
			_yaw = yaw;
			if (debug && (newTicksL!=ticksL) && (newTicksR!=ticksR)) {
				Serial3.print("Base position x:");
				Serial3.print(_x,6);
				Serial3.print(" y:");
				Serial3.print(_y,6);
				Serial3.print(" yaw:");
				Serial3.print(_yaw,6);
				Serial3.println();
			}
			if (speedMode) {
				pwmL = pidL.update(curVL-goalVL,dt);
				pwmR = pidR.update(curVR-goalVR,dt);
				//Serial3.print("SpeedMode dt ");
				//Serial3.print(dt);
				//Serial3.print(" curVL:");
				//Serial3.print(curVL,6);
				//Serial3.print(" goalVL:");
				//Serial3.print(goalVL,6);
				//Serial3.print(" pwmL:");
				//Serial3.println(pwmL);
				//Serial3.print("SpeedMode dt ");
				//Serial3.print(dt);
				//Serial3.print(" curVR:");
				//Serial3.print(curVR,6);
				//Serial3.print(" goalVR:");
				//Serial3.print(goalVR,6);
				//Serial3.print(" pwmR:");
				//Serial3.println(pwmR);
				setMotorPwm(pwmL,pwmR);
			}
			ticksL    = newTicksL;
			ticksR    = newTicksR;
			loopTime  = millis();
		}
		void setMotorPwm(int pwmL,int pwmR) {
			pwmL = constrain(pwmL,-255,255);
			pwmR = constrain(pwmR,-255,255);
			EncoderL->setMotorPwm(-pwmL);
			EncoderR->setMotorPwm( pwmR);
		}
		void setSpeed(double v, double w) {
			speedMode = true;
			goalVR = v-w*baseWidth/2;
			goalVL = v+w*baseWidth/2;
		}
		void print() {
			Serial3.print("# Base - x:");
			Serial3.print(x(),3);
			Serial3.print(" y:");
			Serial3.print(y(),3);
			Serial3.print(" yaw:");
			Serial3.println(yaw());
		}
		void stop() {
			EncoderL->setMotorPwm(0);
			EncoderR->setMotorPwm(0);
			pidL.reset();
			pidR.reset();
			speedMode = false;
		}
			
		double x()   { return this->_x;   }
		double y()   { return this->_y;   }
		double yaw() { return this->_yaw; }
	private:
		double baseWidth,wheelRadius;
		int    ticksPerRotation;
		double _x,_y,_yaw;
		bool   debug;
		PID    pidL,pidR;
		bool   speedMode;
		double kP,kI,kD;
		double goalVR,goalVL;
		double curVR,curVL,curW;
		unsigned long loopTime;
		long    pwmL,pwmR;
		long    ticksL,ticksR;

};


