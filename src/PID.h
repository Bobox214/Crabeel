
class PID {
	public:
		PID() {
			reset();
		}
		void setCoefficients(double kP, double kI, double kD) {
			this->kP = kP;
			this->kI = kI;
			this->kD = kD;
		}
		void setDebug(bool debug) {
			this->debug = debug;
		}
		void setOutputRange(double minOutput,double maxOutput,double neutralZone) {
			this->minOutput   = minOutput;
			this->maxOutput   = maxOutput;
			this->neutralZone = neutralZone;
		}
		double update(double error) {
			return update(error,error-lastError,1);
		}
		double update(double error,double dt) {
			return update(error,(error-lastError)/dt,dt);
		}
		double update(double error,double diffError,double dt) {
			output = kP*error+kI*cumError+kD*diffError;
			if (output<0)
				output -= neutralZone;
			else
				output += neutralZone;
				
			output = constrain(output,minOutput,maxOutput);
			if (output!=minOutput and output!=maxOutput) {
				cumError += error*dt;
			}
			lastError = error;
			if (debug) {
				Serial3.print("PID : P ");
				Serial3.print(error);
				Serial3.print("->");
				Serial3.print(kP*error);
				Serial3.print(" I ");
				Serial3.print(cumError);
				Serial3.print("->");
				Serial3.print(kI*cumError);
				Serial3.print(" D ");
				Serial3.print(diffError);
				Serial3.print("->");
				Serial3.print(kD*diffError);
				Serial3.print(" : output ");
				Serial3.println(output);
			}
			return output;
		}
		void reset() {
			error     = 0;
			cumError  = 0;
			diffError = 0;
		}
	private:
		bool debug;
		double neutralZone;
		double minOutput;
		double maxOutput;
		double kI, kP, kD;
		double error, lastError, diffError, cumError;
		double output;
};
