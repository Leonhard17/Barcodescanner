/*
This Project implements a simple barcode scanner using a led and a phtotransistor
*/

// Inlcudes
#include <Arduino.h>
#include "Barcodescanner.h"

// Defines
#define SENSOR_PIN A0
#define SAMPLINGRATE 1000 // in us

Barcodescanner scanner = Barcodescanner(SENSOR_PIN, SAMPLINGRATE);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {
    // put your main code here, to run repeatedly:
    scanner.Advanced_Measure();
}

