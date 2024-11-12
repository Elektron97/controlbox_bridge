#include <SPI.h>

#define DAC dACh
const int loadPin = 53;
int valve9=0;
const int dACa=0, 
          dACb=2,
          dACc=4,
          dACd=6,
          dACe=8,
          dACf=10,
          dACg=12,
          dACh=14,
          dRNG=1;
          
void setup()
{


  analogWriteResolution(12); 
  // set the slaveSelectPin as an output:
  pinMode (loadPin, OUTPUT);
  // initialize SPI:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);  // Most Significant bit first.
  SPI.setClockDivider(SPI_CLOCK_DIV16);  //16MHz divided by 16 = 1MHz
  SPI.setDataMode(SPI_MODE1);  // zero based clock, data on falling edge, seems like the correct setting
  digitalWrite(loadPin, LOW);
  Serial.begin(115200);
  analogWrite(DAC0,0);  
}

void loop()
{
  getData();
}

void DAC1Write(int channel, int level)
{
  // take the SS pin low to select the chip:
  digitalWrite(loadPin, HIGH);
  //  send in the address and value via SPI:
  SPI.transfer(channel);
  SPI.transfer(level);
  // take the SS pin high to de-select the chip:
  digitalWrite(loadPin, LOW);
}


void getData()
{
  int syncbyte = 106;

  long array [] = {0, 0, 0, 0, 0, 0, 0, 0,0,0}; //9
  if (Serial.available() > 0)
  { //
    if (Serial.read() == syncbyte)
    { // got a sync byte?
      while (Serial.available() < 10)
      {
        
      }
      array[0] = Serial.read();//valvola1  /// metto valvola 9 in 0 
      array[1] = Serial.read();//valvola2
      array[2] = Serial.read();//valvola3
      array[3] = Serial.read();//valvola4
      array[4] = Serial.read();//valvola5
      array[5] = Serial.read();//valvola6
      array[6] = Serial.read();//valvola7
      array[7] = Serial.read();//valvola8
      array[8] = Serial.read();//valvola9 Low
     array[9] = Serial.read();//valvola9  High

      valve9=array[8] + 256 * array[9];
      
         
      DAC1Write(dACg, array[0]);
      DAC1Write(dACf, array[1]);
      DAC1Write(dACh, array[2]);
      DAC1Write(dACe, array[3]);
      DAC1Write(dACb, array[4]);
      DAC1Write(dACc, array[5]);
      DAC1Write(dACa, array[6]);
      DAC1Write(dACd, array[7]);
      analogWrite(DAC0,valve9); 
     // analogWrite(DAC0,array[8]);

    
    }
  }
}