
#include <MeMegaPi.h>

void setup() {
	Serial.begin(115200);
	Serial3.begin(115200);
}

void loop() {
	// Write back data received from Bluetooth to Serial monitor
	if (Serial3.available()) {
		Serial.write(Serial3.read());
	}
	// Send data push to Serial monitor to send through Bluetooth
	if (Serial.available()) {
		Serial3.write(Serial.read());
	}
}
