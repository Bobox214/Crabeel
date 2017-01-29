#include "Configuration.h"

#define NBCONFS  10
#define NBCONFKEEP    3
#define NBCONFRAND    1
#define NBCONFPARENTS 3
#define NBCONFCHILDS  2
#define NBCONFMUTATE  4
#define NBITEMMUTATE  2
#define SYNC  (uint32_t)(0xDEADBEEF)

class Generation {
	public:
		Generation() {
			for (int i=0;i<NBCONFS;i++) {
				configuration[i] = new Configuration();
			}
			randomize();
		}
		void randomize() {
			level = 0;
			for (int i=0;i<NBCONFS;i++) {
				configuration[i]->randomize();
			}
		}
		void debugCreateScores() {
			for (int i=0;i<NBCONFS;i++) {
				configuration[i]->debugCreateScore();
			}
		}

		void print() {
			Serial3.print("# Generation : ");
			Serial3.println(level);
			for (int i=0;i<NBCONFS;i++) {
				Serial3.print("    ");
				Serial3.print(i);
				configuration[i]->print();
			}
		}
		void breed() {
			// Create nextGeneration;
			level += 1;
			// Sort configurations by score 0 is worst, NBCONFS is best
			for(uint8_t i=0; i<(NBCONFS-1); i++) {
				for(uint8_t o=0; o<(NBCONFS-(i+1)); o++) {
					if (configuration[o]->score > configuration[o+1]->score) {
						Configuration *t = configuration[o];
						configuration[o] = configuration[o+1];
						configuration[o+1] = t;
					}
				}
			}
			uint8_t curIdx = 0;
			// Randomize the first configuration
			for (;curIdx<NBCONFRAND;curIdx++){
				configuration[curIdx]->randomize();
				configuration[curIdx]->setChildOf(
					configuration[NBCONFS-1]
				,	configuration[curIdx]
				);
			}
			// Create child from best and less best
			for (uint8_t i=0;i<NBCONFPARENTS;i++) {
				for (uint8_t j=0;j<NBCONFCHILDS;j++) {
					configuration[curIdx]->setChildOf(
						configuration[NBCONFS-1]
					,	configuration[NBCONFS-1-i]
					);
					curIdx += 1;
				}
			}
			// Keep the best ones
			// Mutate, never the best configuration
			for (uint8_t i=0;i<NBCONFMUTATE;i++) {
				configuration[random(0,NBCONFS-1)]->mutate(NBITEMMUTATE);
			}
			// Reset scores
			for (uint8_t i=0;i<NBCONFS;i++)
				configuration[i]->resetScore();
		}
		void saveToEEPROM(uint16_t addr,bool verbose=false) {
			if ((addr%256) != 0 ) {
				Serial3.print("[ERROR] Cannot save to address ");
				Serial3.print(addr);
				Serial3.println(", must a multiple of 256.");
				return;
			}
			EEPROM.put(addr,SYNC);
			EEPROM.put(addr+4,level);
			for (uint8_t i;i<NBCONFS;i++) {
				configuration[i]->saveToEEPROM(addr+6+22*i);
			}
			if (verbose) {
				Serial3.print("[INFO] Generation saved to address ");
				Serial3.print(addr);
				Serial3.println();
				print();
			}
		}
		void loadFromEEPROM(uint16_t addr,bool verbose=false) {
			if ((addr%256) != 0 ) {
				Serial3.print("[ERROR] Cannot load from address ");
				Serial3.print(addr);
				Serial3.println(", must a multiple of 256.");
				return;
			}
			uint32_t sync;
			EEPROM.get(addr,sync);
			if (sync!=SYNC) {
				Serial3.print("[ERROR] Cannot load from address ");
				Serial3.print(addr);
				Serial3.println(", no generation saved at this address before.");
				return;
			}
			EEPROM.get(addr+4,level);
			for (uint8_t i;i<NBCONFS;i++) {
				configuration[i]->loadFromEEPROM(addr+6+22*i);
			}
			if (verbose) {
				Serial3.print("[INFO] Generation loaded from address ");
				Serial3.print(addr);
				Serial3.println();
				print();
			}
		}
		uint16_t level;
		Configuration *configuration[NBCONFS];
};
