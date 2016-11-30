#include "PID.h"

MeEncoderOnBoard Encoder_Arm(SLOT3);

#define CRABEEL_ARM_GEAR_RATIO 7
class CrabeelArm {
	public:
		CrabeelArm()
			:   debug(false)
			,	timeStartMotorPwm(0)
		{
			anglePID.setCoefficients(1,0.1,0);
		}
		void setDebug(bool debug) {
			this->debug = debug;
			this->anglePID.setDebug(debug);
		}
		int getAngle() {
			return (Encoder_Arm.getPulsePos()/CRABEEL_ARM_GEAR_RATIO)%360;
		}
		void goToAngle(int angle,int pwmMax=255) {
			int curPos = Encoder_Arm.getPulsePos();
			targetPos = CRABEEL_ARM_GEAR_RATIO*(angle+360*(curPos/(CRABEEL_ARM_GEAR_RATIO*360)));
			if (curPos-targetPos>180*CRABEEL_ARM_GEAR_RATIO) targetPos=targetPos+360*CRABEEL_ARM_GEAR_RATIO;
			if (targetPos-curPos>180*CRABEEL_ARM_GEAR_RATIO) targetPos=targetPos-360*CRABEEL_ARM_GEAR_RATIO;
			if (debug) {
				Serial3.print("Crabeel Arm : Start goToAngle ");
				Serial3.print(angle);
				Serial3.print(" ArmAngle ");
				Serial3.print((curPos/CRABEEL_ARM_GEAR_RATIO)%360);
				Serial3.print(" CurPos ");
				Serial3.println(curPos);
				Serial3.print(" TargetPos ");
				Serial3.print(targetPos);
			}
			anglePID.reset();
			anglePID.setOutputRange(-pwmMax,pwmMax,0);
			Encoder_Arm.setMotionMode(DIRECT_MODE);
			activeGoToAngle = true;
		}
		void setMotorPwm(int pwm) {
			if (activeGoToAngle)
				Serial3.println("*** Error *** : CrabeelArm : Cannot setMotorPwm when goToAngle is still active");
			timeStartMotorPwm = millis();
			Encoder_Arm.setMotorPwm(pwm);
			if (debug) {
				Serial3.print("CrabeelArm : Start MotorPwm ");
				Serial3.println(pwm);
			}
		}
		void initialize() {
			double minAngle,maxAngle;
			Encoder_Arm.setMotorPwm(80);
			Encoder_Arm.loop();
			delay(100);
			while (true) {
				delay(20);
				Encoder_Arm.loop();
				if (abs(Encoder_Arm.getCurrentSpeed())<3)
					break;
			}
			maxAngle = getAngle();
			Encoder_Arm.setMotorPwm(-80);
			Encoder_Arm.loop();
			delay(100);
			while (true) {
				delay(20);
				Encoder_Arm.loop();
				if (abs(Encoder_Arm.getCurrentSpeed())<3)
					break;
			}
			minAngle = getAngle();
			Encoder_Arm.setMotorPwm(0);
			Serial3.print("Found min :");
			Serial3.print(minAngle);
			Serial3.print(" max :");
			Serial3.println(maxAngle);

			goToAngle((maxAngle+minAngle)/2);
			while (activeGoToAngle) {
				loop();
				delay(10);
			}
			reset();
		}

		void reset() {
			activeGoToAngle = false;
			Encoder_Arm.setPulsePos(0);
			Encoder_Arm.setMotionMode(DIRECT_MODE);
			Encoder_Arm.setMotorPwm(0);
		}
		void setup() {
			attachInterrupt(Encoder_Arm.getIntNum() , processEncoderInt , RISING);
			Encoder_Arm.setPulse(8);
			Encoder_Arm.setRatio(46.67);
			Encoder_Arm.setPosPid(1.8,0,1.2);
			Encoder_Arm.setSpeedPid(0.18,0,0);
		}
		void loop() {
			Encoder_Arm.loop();
			if (activeGoToAngle) {
				int error = targetPos-Encoder_Arm.getPulsePos();
				if (abs(error)>=2) {
					Encoder_Arm.setMotorPwm( anglePID.update(error) );
				} else {
					activeGoToAngle = false;
					Encoder_Arm.setMotorPwm(0);
					if (debug)
						Serial3.println("CrabeelArm : End goToAngle ");
				}
			} else if (timeStartMotorPwm) {
				if (millis()-timeStartMotorPwm>200) {
					Encoder_Arm.setMotorPwm(0);
					timeStartMotorPwm = 0;
					if (debug) {
						Serial3.print("CrabeelArm : End MotorPwm ");
					}
				}
			}
		}

	private:
		bool debug;
		PID anglePID;
		int targetPos;
		unsigned long timeStartMotorPwm;
		bool activeGoToAngle = false;

		static void processEncoderInt(void) {
			if(digitalRead(Encoder_Arm.getPortB()) == 0)
				Encoder_Arm.pulsePosMinus();
			else
				Encoder_Arm.pulsePosPlus();;
		}
};

