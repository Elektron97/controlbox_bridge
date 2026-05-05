
#include <SPI.h>
#define DAC dACh

#ifndef ARDPRINTF
#define ARDPRINTF
#define ARDBUFFER 16
#include <stdarg.h>
#include <Arduino.h>

int ardprintf(char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

  va_list argv;
  va_start(argv, count);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j=0;
      temp[0] = '\0';

      switch(str[++i])
      {
        case 'd': Serial.print(va_arg(argv, int));
                  break;
        case 'l': Serial.print(va_arg(argv, long));
                  break;
        case 'f': Serial.print(va_arg(argv, double));
                  break;
        case 'c': Serial.print((char)va_arg(argv, int));
                  break;
        case 's': Serial.print(va_arg(argv, char *));
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%ARDBUFFER;
      if(j==0) 
      {
        temp[ARDBUFFER] = '\0';
        Serial.print(temp);
        temp[0]='\0';
      }
    }
  };
  Serial.println();
  return count + 1;
}
#undef ARDBUFFER
#endif
#include <SPI.h>

#define DAC dACh
const int loadPin = 52;
const int trigger1 = 49;
const int trigger2 = 51;
const int dACa=0, 
          dACb=2,
          dACc=4,
          dACd=6,
          dACe=8,
          dACf=10,
          dACg=12,
          dACh=14,
          dRNG=1;
int TR1=0;
int TR2=0;    
float p0,p1,p2,p3,p4,p5=0;    //feedback pneumatic valves camozzi     
void setup()
{

  pinMode(trigger1,OUTPUT);   // trigger to Instron from Arduino
  digitalWrite(trigger1,LOW);
  pinMode(trigger2,INPUT);    // trigger from Instron to Arduino
  
  analogWriteResolution(12);
  analogReadResolution(12); 
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

  long array [] = {0, 0 ,0 ,0 ,0 ,0, 0}; //2
  if (Serial.available() > 0)
  { //
    if (Serial.read() == syncbyte)
    { // got a sync byte?
      while (Serial.available() < 7)
      {
        
      }
      array[0] = Serial.read();//valve1    
      array[1] = Serial.read();//valve2
      array[2] = Serial.read();//valve3
      array[3] = Serial.read();//valve4
      array[4] = Serial.read();//valve5
      array[5] = Serial.read();//valve6
      array[6] = Serial.read(); // trigger1
       
      DAC1Write(dACg, array[0]);
      DAC1Write(dACf, array[1]);
      DAC1Write(dACh, array[2]);
      DAC1Write(dACe, array[3]);
      DAC1Write(dACb, array[4]);
      DAC1Write(dACa, array[5]);
      TR1 = array[6];
      digitalWrite(trigger1,TR1);

      TR2=digitalRead(trigger2);
      //Serial.println(TR2);

      p0=analogRead(A0);
      p1=analogRead(A1);
      p2=analogRead(A2);
      p3=analogRead(A3);
      p4=analogRead(A4);
      p5=analogRead(A5);
      
      ardprintf("%d %f %f %f %f %f %f ",TR2, p0*0.0008, p1*0.0008, p2*0.0008, p3*0.0008, p4*0.0008, p5*0.0008);
    
    
    }
 
  }
}
