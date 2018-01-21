/*
 Name:		Serial_Clock.ino
 Created:	21.01.2018 20:52:04
 Author:	Allester Fox
*/

#include <RTClib.h>
#include "RTC_Buildin.h"

RTC_Buildin rtc;

void setup() {
	Serial.begin(115200);
	Serial.println(F("RTC Init..."));
	pinMode(LED_BUILTIN, OUTPUT);	// Internal led for seconds indication
	rtc.begin(DateTime(F(__DATE__), F(__TIME__)));	// Set date to Build Date and Time
	Serial.println(F("Done!"));
}

void loop() {
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));	// Switch led state
	
	DateTime dt = rtc.now();	// get current date and time

	Serial.print(F("Current: "));
	Serial.print(dt.day());
	Serial.print(F("."));
	Serial.print(dt.month());
	Serial.print(F("."));
	Serial.print(dt.year());
	Serial.print(F(", "));
	Serial.print(dt.hour());
	Serial.print(F(":"));
	Serial.print(dt.minute());
	Serial.print(F(":"));
	Serial.print(dt.second());
	Serial.println();
	
	delay(1000);
}