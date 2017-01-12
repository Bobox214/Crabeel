
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define WIDTH  128
#define HEIGHT 32
class Display {
	public:
		Display(uint8_t i2cAddr) {
			display = new Adafruit_SSD1306();
			display->begin(SSD1306_SWITCHCAPVCC, i2cAddr);
		}
		void printState(char stateName[]) {
			uint8_t len = strlen(stateName); 
			if (len>10) {
				stateName = "dead";
				len = 4;
			}
			uint8_t x = (128-len*12)/2;
			Serial.println(stateName);
			Serial.println(x);
			Serial.print("len");
			Serial.println(len);
			display->setFont();
			display->setTextSize(2);
			display->setCursor(x,0);
			display->setTextColor(WHITE,BLACK);
			display->println(stateName);
			display->display();
		}
		void fillScreen(uint8_t color) {
			display->fillScreen(color);
			display->display();
		}
		void setup() {
			display->clearDisplay();
			display->display();
		}
	private:
		Adafruit_SSD1306 *display;
};
			
Display *display;

void setup()   {                
	Serial.begin(9600);
	Serial.println("Setup");
	display = new Display(0x3c);
	display->setup();
	display->printState("IDLE");
}

void loop() {
	delay(1000);
	display->printState("BALANCE");
	delay(1000);
	display->printState("Calibration");
	delay(1000);
	display->printState("AutoStart");
}
