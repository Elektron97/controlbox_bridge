#include <SPI.h>

#define DAC dACh
const int loadPin = 53;

const int dACa=0, 
          dACb=2,
          dACc=4,
          dACd=6,
          dACe=8,
          dACf=10,
          dACg=12,
          dACh=14,
          dRNG=1;
          

void setup() {
  pinMode (loadPin, OUTPUT);  // DEFINE PIN OUTPUT

  // initialize SPI:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);  // Most Significant bit first.
  SPI.setClockDivider(SPI_CLOCK_DIV16);  //16MHz divided by 16 = 1MHz
  SPI.setDataMode(SPI_MODE1);  // zero based clock, data on falling edge, seems like the correct setting
  digitalWrite(loadPin, LOW);   //PUT LOW PIN
  // zeroDACs();
  Serial.begin(115200);

}



void DAC1Write(int channel, int level)
{
 
  digitalWrite(loadPin, HIGH);
  //  send in the address and value via SPI:
  SPI.transfer(channel);
  SPI.transfer(level);

  digitalWrite(loadPin, LOW);
}

void loop() {

  getData();
  
}


void getData()
{
  int syncbyte = 106;

  int array [] = {0, 0, 0, 0, 0, 0, 0 };
  
  if (Serial.available() > 0)   // verify to serial is available
  { //
    if (Serial.read() == syncbyte)      // verify that the first value reads is 106
    { 
     while (Serial.available() < 7)
     {

      }

    
      
      array[0] = Serial.read();//valve1  read value from serial
      array[1] = Serial.read();//valve2      
      array[2] = Serial.read();//valve3
      array[3] = Serial.read();//valve4
      array[4] = Serial.read();//valve5
      array[5] = Serial.read();//valve6
      array[6] = Serial.read();//valve7
      
   
      DAC1Write(dACh, array[0]);
      DAC1Write(dACg, array[1]);
      DAC1Write(dACc, array[2]);
      DAC1Write(dACd, array[3]);
      DAC1Write(dACa, array[4]);
      DAC1Write(dACb, array[5]);
      DAC1Write(dACe, array[6]);
      
    }
}
}