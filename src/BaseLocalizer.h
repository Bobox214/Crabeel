#define PI 3.141592

class BaseLocalizer {
	public:
		BaseLocalizer()
			:	debug(false)
			,	_x(0)
			,	_y(0)
			,	_yaw(0)
		{}
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
		void update(int newTicksL, int newTicksR, double yaw) {
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
