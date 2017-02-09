#include <EEPROM.h>

#define NBSCORES 10
#define MINKP 0
#define MAXKP 100
#define MINKI 0
#define MAXKI 100
#define MINKD 0
#define MAXKD 10
#define MINPITCH -2
#define MAXPITCH  2
#define MINAUTOSTART 150
#define MAXAUTOSTART 400
#define MINPWMSTART  170
#define MAXPWMSTART  255
#define MINNEUTRAL   0
#define MAXNEUTRAL   150

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
			this->kP                = constrain(kP,MINKP,MAXKP);
			this->kI                = constrain(kI,MINKI,MAXKI);
			this->kD                = constrain(kD,MINKD,MAXKD);
			this->balancePitch      = constrain(balancePitch,MINPITCH,MAXPITCH);
			this->autoStartDuration = constrain(autoStartDuration,MINAUTOSTART,MAXAUTOSTART);
			this->autoStartPwm      = constrain(autoStartPwm,MINPWMSTART,MAXPWMSTART);
			this->neutralZone       = constrain(neutralZone,MINNEUTRAL,MAXNEUTRAL);
			resetScore();
		}
		double  kP,kI,kD;
		double  balancePitch;
		int16_t autoStartDuration;
		int16_t autoStartPwm;
		int16_t neutralZone;
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
		void mutate_kP(uint16_t percent) {
			double var = ((MAXKP-MINKP)*random(percent*10))/1000.0;
			if (random(2)==0) var = -var;
			kP = constrain( kP+var , MINKP , MAXKP);
		}
		void mutate_kI(uint16_t percent) {
			double var = ((MAXKI-MINKI)*random(percent*10))/1000.0;
			if (random(2)==0) var = -var;
			kI = constrain( kI+var , MINKI , MAXKI);
		}
		void mutate_kD(uint16_t percent) {
			double var = ((MAXKD-MINKD)*random(percent*10))/1000.0;
			if (random(2)==0) var = -var;
			kD = constrain( kD+var , MINKD , MAXKD);
		}
		void mutate_balancePitch(uint16_t percent) {
			double var = ((MAXPITCH-MINPITCH)*random(percent*10))/1000.0;
			if (random(2)==0) var = -var;
			balancePitch = constrain( balancePitch+var , MINPITCH , MAXPITCH );
		}
		void mutate_autoStartDuration(uint16_t percent) {
			double var = ((MAXAUTOSTART-MINAUTOSTART)*random(percent*10))/1000.0;
			if (random(2)==0) var = -var;
			autoStartDuration = constrain( autoStartDuration+var , MINAUTOSTART , MAXAUTOSTART );
		}
		void mutate_autoStartPwm(uint16_t percent) {
			double var = ((MAXPWMSTART-MINPWMSTART)*random(percent*10))/1000.0;
			if (random(2)==0) var = -var;
			autoStartPwm = constrain( autoStartPwm+var , MINPWMSTART , MAXPWMSTART );
		}
		void mutate_neutralZone(uint16_t percent) {
			double var = ((MAXNEUTRAL-MINNEUTRAL)*random(percent*10))/1000.0;
			if (random(2)==0) var = -var;
			neutralZone = constrain( neutralZone+var , MINNEUTRAL , MAXNEUTRAL );
		}

		void randomize() {
			kP = random(0,10*MAXKP+1)/10.0;
			kI = random(0,10*MAXKI+1)/10.0;
			kD = random(0,100*MAXKD+1)/100.0;
			balancePitch = random(10*MINPITCH,10*MAXPITCH)/10.0;
			autoStartPwm = random(MINPWMSTART,MAXPWMSTART+1);
			autoStartDuration = random(MINAUTOSTART,MAXAUTOSTART+1);
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
			EEPROM.put(idx+20,neutralZone);
			EEPROM.put(idx+22,score);
		}
		void loadFromEEPROM(int idx) {
			EEPROM.get(idx   ,kP);
			EEPROM.get(idx+4 ,kI);
			EEPROM.get(idx+8 ,kD);
			EEPROM.get(idx+12,balancePitch);
			EEPROM.get(idx+16,autoStartDuration);
			EEPROM.get(idx+18,autoStartPwm);
			EEPROM.get(idx+20,neutralZone);
			EEPROM.get(idx+22,score);
			runIdx = 0;
		}
		uint16_t debugCreateScore() {
			resetScore();
			uint16_t score = 0;
			score += 10*(100-abs(kP-kI));
			score -= 10*kD;
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
				Serial3.println("[ERROR] Cannot add score. Too much runs");
				return;
			}
			runScores[runIdx] = score;
			runIdx++;
		}
		void compileScore() {
			if (runIdx!=NBSCORES) {
				Serial3.print("[ERROR] Cannot compile score. Not enough runs; runIdx=");
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
			// score is the mean of runScores[3:9];	
			for (uint8_t i=3;i<NBSCORES-1;i++)
				sum += runScores[i];
			score = sum/(NBSCORES-4);
		}
		void setChildOf(Configuration* father, Configuration* mother) {
			kP                = random(2)==0 ? father->kP                : mother->kP                ;
			kI                = random(2)==0 ? father->kI                : mother->kI                ;
			kD                = random(2)==0 ? father->kD                : mother->kD                ;
			balancePitch      = random(2)==0 ? father->balancePitch      : mother->balancePitch      ;
			autoStartDuration = random(2)==0 ? father->autoStartDuration : mother->autoStartDuration ;
			autoStartPwm      = random(2)==0 ? father->autoStartPwm      : mother->autoStartPwm      ;
			neutralZone       = random(2)==0 ? father->neutralZone       : mother->neutralZone       ;
			mutate_kP(5);
			mutate_kI(5);
			mutate_kD(5);
			mutate_balancePitch(5);
			mutate_autoStartDuration(5);
			mutate_autoStartPwm(5);
			mutate_neutralZone(5);
		}
		void mutate(uint8_t nbMutation) {
			for (uint8_t j=0;j<nbMutation;j++) {
				uint8_t i = random(0,7);
				if	(i==0) mutate_kP(15);
				if	(i==1) mutate_kI(15);
				if	(i==2) mutate_kD(15);
				if	(i==3) mutate_balancePitch(15);
				if	(i==4) mutate_autoStartDuration(15);
				if	(i==5) mutate_autoStartPwm(15);
				if	(i==6) mutate_neutralZone(15);
			}
		}
	public:
		uint16_t score;
	private:
		uint8_t  runIdx;
		uint16_t runScores[NBSCORES];
};
