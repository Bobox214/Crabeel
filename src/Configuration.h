#include <EEPROM.h>

#define NBSCORES 10
#define MAXKP 100
#define MAXKI 100
#define MAXKD  10
#define MINPITCH -2
#define MAXPITCH  2
#define MAXAUTOSTART 1000
#define MINPWMSTART  100
#define MAXPWMSTART  255
#define MAXNEUTRAL   255

class Configuration {
	public:
		Configuration() {
			resetScore();
		}
		void setCoefficients(
			double kP,double kI,double kD
		,	double balancePitch
		,	uint16_t autoStartDuration,uint8_t autoStartPwm
		,	uint8_t neutralZone
		) {
			this->kP = constrain(kP,0,MAXKP);
			this->kI = constrain(kI,0,MAXKI);
			this->kD = constrain(kD,0,MAXKD);
			this->balancePitch      = constrain(balancePitch,MINPITCH,MAXPITCH);
			this->autoStartDuration = constrain(autoStartDuration,0,MAXAUTOSTART);
			this->autoStartPwm      = constrain(autoStartPwm,MINPWMSTART,MAXPWMSTART);
			this->neutralZone       = constrain(neutralZone,0,MAXNEUTRAL);
			resetScore();
		}
		double   kP,kI,kD;
		double   balancePitch;
		uint16_t autoStartDuration;
		uint8_t  autoStartPwm;
		uint8_t  neutralZone;
		void print() {
			Serial3.print("# Configuration - PID:(");
			Serial3.print(kP,2);
			Serial3.print(",");
			Serial3.print(kI,2);
			Serial3.print(",");
			Serial3.print(kD,2);
			Serial3.print(") - pitch:"); 
			Serial3.print(balancePitch,2); 
			Serial3.print(" - autoStart:"); 
			Serial3.print(autoStartPwm); 
			Serial3.print(" for "); 
			Serial3.print(autoStartDuration); 
			Serial3.print("ms - neutral:"); 
			Serial3.print(neutralZone); 
			if (score!=0) {
				Serial3.print(" -> score:");
				Serial3.print(score);
			}
			Serial3.println();
		}
		void randomize() {
			kP = random(0,10*MAXKP+1)/10.0;
			kI = random(0,10*MAXKI+1)/10.0;
			kD = random(0,100*MAXKD+1)/100.0;
			balancePitch = random(10*MINPITCH,10*MAXPITCH)/10.0;
			autoStartPwm = random(MINPWMSTART,MAXPWMSTART+1);
			autoStartDuration = random(0,MAXAUTOSTART+1);
			neutralZone = random(0,MAXNEUTRAL+1);
			resetScore();
		}
		void saveToEEPROM(int idx) {
			EEPROM.put(idx   ,kP);
			EEPROM.put(idx+4 ,kI);
			EEPROM.put(idx+8 ,kD);
			EEPROM.put(idx+12,balancePitch);
			EEPROM.put(idx+16,autoStartDuration);
			EEPROM.put(idx+18,autoStartPwm);
			EEPROM.put(idx+19,neutralZone);
		}
		void loadFromEEPROM(int idx) {
			EEPROM.get(idx   ,kP);
			EEPROM.get(idx+4 ,kI);
			EEPROM.get(idx+8 ,kD);
			EEPROM.get(idx+12,balancePitch);
			EEPROM.get(idx+16,autoStartDuration);
			EEPROM.get(idx+18,autoStartPwm);
			EEPROM.get(idx+19,neutralZone);
		}
		uint16_t debugCreateScore() {
			resetScore();
			uint16_t score = 0;
			score += 100*abs(kP-kI);
			score -= 100*kD;
			score -= abs(balancePitch);
			score += abs(250-autoStartDuration);
			score += autoStartPwm;
			score -= abs(neutralZone);
			for (int j=0;j<NBSCORES;j++) {
				addScore(score);
			}
			compileScore();
		}
			
		void resetScore() {
			runIdx = 0;
			score  = 0;
		}
		void addScore(uint16_t score) {
			if (runIdx>NBSCORES-1) {
				Serial3.println("#### ERROR #### Cannot add score. Too much runs");
				return;
			}
			runScores[runIdx] = score;
			runIdx++;
		}
		void compileScore() {
			if (runIdx!=NBSCORES) {
				Serial3.print("#### ERROR #### Cannot compile score. Not enough runs; runIdx=");
				Serial3.print(runIdx);
				Serial3.println();
				score = 0;
				return;
			}
			// Sort the runScores
			for(uint8_t i=0; i<(NBSCORES-1); i++) {
				for(uint8_t o=0; o<(NBSCORES-(i+1)); o++) {
					if(runScores[o] > runScores[o+1]) {
						uint16_t t = runScores[o];
						runScores[o] = runScores[o+1];
						runScores[o+1] = t;
					}
				}
			}
			uint32_t sum=0;
			// score is the mean of runScores[2:8];	
			for (uint8_t i=2;i<NBSCORES-2;i++)
				sum += runScores[i];
			score = sum/6;
		}
		void setChildOf(Configuration* father, Configuration* mother) {
			setCoefficients(
				random(90,110)/100.0*(random(2)==0 ? father->kP                : mother->kP                )
			,	random(90,110)/100.0*(random(2)==0 ? father->kI                : mother->kI                )
			,	random(90,110)/100.0*(random(2)==0 ? father->kD                : mother->kD                )
			,	random(90,110)/100.0*(random(2)==0 ? father->balancePitch      : mother->balancePitch      )
			,	random(90,110)/100.0*(random(2)==0 ? father->autoStartDuration : mother->autoStartDuration )
			,	random(90,110)/100.0*(random(2)==0 ? father->autoStartPwm      : mother->autoStartPwm      )
			,	random(90,110)/100.0*(random(2)==0 ? father->neutralZone       : mother->neutralZone       )
			);
		}
		void mutate() {
			uint8_t i = random(0,7);
			setCoefficients(
				i==0 ? (random(70,130)/100.0)*kP                : kP
			,	i==1 ? (random(70,130)/100.0)*kI                : kI
			,	i==2 ? (random(70,130)/100.0)*kD                : kD
			,	i==3 ? (random(70,130)/100.0)*balancePitch      : balancePitch
			,	i==4 ? (random(70,130)/100.0)*autoStartDuration : autoStartDuration
			,	i==5 ? (random(70,130)/100.0)*autoStartPwm      : autoStartPwm
			,	i==6 ? (random(70,130)/100.0)*neutralZone       : neutralZone
			);
		}
	public:
		uint16_t score;
	private:
		uint8_t  runIdx;
		uint16_t runScores[NBSCORES];
};
