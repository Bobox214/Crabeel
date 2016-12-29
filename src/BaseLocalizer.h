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

class BaseLocalizer {
	public:
		BaseLocalizer(uint8_t slotL,uint8_t slotR)
			:	debug(false)
			,	_x(0)
			,	_y(0)
			,	_yaw(0)
		{	EncoderL = new MeEncoderOnBoard(slotL);
			EncoderR = new MeEncoderOnBoard(slotR);
			attachInterrupt(EncoderL->getIntNum() , processEncoderLInt , RISING);
			attachInterrupt(EncoderR->getIntNum() , processEncoderRInt , RISING);
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
		void updatePosition(double yaw) {
			int newTicksL = -EncoderL->getPulsePos();
			int newTicksR =  EncoderR->getPulsePos();
			double dl = (2*PI*wheelRadius*(newTicksL-ticksL))/ticksPerRotation;
			double dr = (2*PI*wheelRadius*(newTicksR-ticksR))/ticksPerRotation;
			double df = (dr+dl)/2;
			double dx = df*cos(yaw);
			double dy = df*sin(yaw);
			if (debug && (newTicksL!=ticksL) && (newTicksR!=ticksR)) {
				Serial3.print("BaseLocalizer tl: ");
				Serial3.print(newTicksL-ticksL);
				Serial3.print(" tr: ");
				Serial3.print(newTicksR-ticksR);
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
			}
			_x += dx;
			_y += dy;
			_yaw = yaw;
			ticksL = newTicksL;
			ticksR = newTicksR;
		}
		void setMotorPwm(int pwmL,int pwmR) {
			pwmL = constrain(pwmL,-255,255);
			pwmR = constrain(pwmR,-255,255);
			EncoderL->setMotorPwm(-pwmL);
			EncoderR->setMotorPwm( pwmR);
		}
		double x()   { return this->_x;   }
		double y()   { return this->_y;   }
		double yaw() { return this->_yaw; }
	private:
		double baseWidth,wheelRadius;
		int    ticksPerRotation;
		double _x,_y,_yaw;
		int    ticksL,ticksR;
		bool   debug;

};


