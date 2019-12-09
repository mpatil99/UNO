
#include "AS5048A.h"

AS5048A angleSensor(10, true);

void setup()
{
	Serial.begin(19200);
	angleSensor.begin();
  pinMode(12, INPUT);
}

void loop()
{
	delay(1000);

	float val = angleSensor.getRotationInDegrees();
	Serial.print("\nGot rotation of: ");
	Serial.println(val);
	Serial.println("\nState: ");
	angleSensor.printState();
	Serial.println("\nErrors: ");
	Serial.println(angleSensor.getErrors());
	Serial.println("\nDiagnostics: ");
	Serial.println(angleSensor.getDiagnostic());
}
