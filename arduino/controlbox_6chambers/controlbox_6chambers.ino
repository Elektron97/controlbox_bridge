#include <SPI.h>
// PLEASE USE IT WITH 6 VALVES. THE 7th IS NECESSARY JUST TO MAKE THE CODE WORK!
// DAC channel pins
const int dACa = 0;
const int dACb = 2;
const int dACc = 4;
const int dACd = 6;
const int dACe = 8;
const int dACf = 10;
const int dACg = 12;
const int dACh = 14;
const int dRNG = 1;

#define DAC dACh
const int loadPin = 53;
const int syncByte = 106;

void setup() {
  pinMode(loadPin, OUTPUT);
  digitalWrite(loadPin, LOW);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE1);

  Serial.begin(115200);
}

void DAC1Write(int channel, int level) {
  digitalWrite(loadPin, HIGH);
  SPI.transfer(channel);
  SPI.transfer(level);
  digitalWrite(loadPin, LOW);
}

void loop() {
  getData();
}

void getData() {
  const int numChannels = 7;
  int values[numChannels];

  if (Serial.available() >= (numChannels + 1)) {
    if (Serial.read() == syncByte) {
      for (int i = 0; i < numChannels; i++) {
        values[i] = Serial.read();
      }

      // Map each value to its DAC channel
      DAC1Write(dACg, values[0]);
      DAC1Write(dACh, values[2]);
      DAC1Write(dACe, values[3]);
      DAC1Write(dACb, values[4]);
      DAC1Write(dACc, values[5]);
      DAC1Write(dACa, values[7]);
      DAC1Write(dACd, values[6]);
      DAC1Write(dACf, values[1]);
      // DAC1Write(dACh, values[0]);
      // DAC1Write(dACg, values[1]);
      // DAC1Write(dACc, values[2]);
      // DAC1Write(dACd, values[3]);
      // DAC1Write(dACa, values[4]);
      // DAC1Write(dACb, values[5]);
      // DAC1Write(dACe, values[6]);
    }
  }
}