#include "Configuration.h"

#define NBCONFS  10
#define NBCONFKEEP    3
#define NBCONFRAND    1
#define NBCONFPARENTS 3
#define NBCONFCHILDS  2
#define NBCONFMUTATE  4
#define SYNC  0xFACE

class Generation {
	public:
		Generation() {
			level = 0;
			for (int i=0;i<NBCONFS;i++) {
				configuration[i] = new Configuration();
			}
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
			// Mutate
			for (uint8_t i=0;i<NBCONFMUTATE;i++) {
				configuration[random(0,NBCONFS)]->mutate();
			}
			// Reset scores
			for (uint8_t i=0;i<NBCONFS;i++)
				configuration[i]->resetScore();
		}
		uint16_t level;
		Configuration *configuration[NBCONFS];
};
