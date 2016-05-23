
////////////////////////////////////////////////
//            IMPORTING LIBARY               //
//////////////////////////////////////////////

#include <SoftwareSerial.h>       //bluetooth
#include <NewPing.h>              //ultra
#include <i2cmaster.h>
#include <Wire.h>
#include <ZumoMotors.h>

////////////////////////////////////////////////
//        DEFINE THE ROBOT I/O PINS          //
//////////////////////////////////////////////

double Input;
ZumoMotors motor;

// Ultra Sensor
#define ultraL 4
#define ultraF 6
#define ultraR 5
#define MAX_DISTANCE 99 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing uL(ultraL, ultraL, MAX_DISTANCE);
NewPing uF(ultraF, ultraF, MAX_DISTANCE);
NewPing uR(ultraR, ultraR, MAX_DISTANCE);

//TIR Sensor Addresses
#define tirL 0x1A<<1
#define tirF 0x1B<<1
#define tirR 0x1C<<1

int sensordata[9] {0, 0, 0, 0, 0, 0, 0, 0, 0};
//0 -Batterylevel
//1 -tirL
//2 -tirF
//3 -tirR
//4 -tirLOGIC
//5
//6
//7
//8

void lampa(int villog)
{
    for(int i; i < 5; i++)
    {
     
      digitalWrite(13, HIGH);
      delay(850);
      digitalWrite(13, LOW);
       delay(850);
    }
}

void UltraUPDATE ()
{
  sensordata[4] = uL.ping_cm();
  sensordata[5] = uF.ping_cm();
  sensordata[6] = uR.ping_cm();
}

void bal()
{
  motor.setLeftSpeed(-110);
  motor.setRightSpeed(110);
  delay(2100);
  motor.setLeftSpeed(0);
  motor.setRightSpeed(0);
}

void jobb()
{
  motor.setLeftSpeed(110);
  motor.setRightSpeed(-110);
  delay(2100);
  motor.setLeftSpeed(0);
  motor.setRightSpeed(0);
}

void elore()
{
  for (short int i = 0; i < 170; i++) {
    homerseklet_figyeles();
    feny_erzekeles();
    Input = uR.ping_cm();
    if (Input < 20)
    {

      motor.setLeftSpeed(160);
      motor.setRightSpeed(160);
    }
    else
    {
      motor.setLeftSpeed(130);
      motor.setRightSpeed(130);
    }
    delay(5);
    short int front = uF.ping_cm();
    if (front < 21)
    {
      motor.setLeftSpeed(120);
      motor.setRightSpeed(120);
      for (short int i = 0; i < 10; i++)
      {
        homerseklet_figyeles();
        delay(195);
      }

      motor.setLeftSpeed(0);
      motor.setRightSpeed(0);
      delay(30);
      motor.setLeftSpeed(-100);
      motor.setRightSpeed(-100);
      delay(1350);
      motor.setLeftSpeed(0);
      motor.setRightSpeed(0);
      delay(300);
      break;
    }
  }
  motor.setLeftSpeed(0);
  motor.setRightSpeed(0);
}

void fordul()
{
  motor.setLeftSpeed(150);
  motor.setRightSpeed(-150);
  delay(1900);
  motor.setLeftSpeed(0);
  motor.setRightSpeed(0);
}

void homerseklet_figyeles()
{
  tirUPDATE();
  if (sensordata[3] > 25 || sensordata[1] > 25)
  {

    motor.setLeftSpeed(0);
    motor.setRightSpeed(0);
    delay(80);
    digitalWrite(3, LOW);
    delay(85);
    digitalWrite(3, HIGH);

    motor.setLeftSpeed(100);
    motor.setRightSpeed(100);
    delay(2000);
    motor.setLeftSpeed(0);
    motor.setRightSpeed(0);
    delay(2000);
  }
}

void feny_erzekeles()
{
  if (analogRead(1) > 500)
  {
    motor.setLeftSpeed(-150);
    motor.setRightSpeed(-150);
    delay(20);
    fordul();
  }
}

void tirUPDATE()
{
  const byte dev[3] = {0x1B << 1, 0x1C << 1, 0x1C << 1}; //szenzorok címe
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

  for (int i = 0; i < 3; i++)
  {
    data_low = 0;
    data_high = 0;
    pec = 0;

    i2c_start_wait(dev[i] + I2C_WRITE);
    i2c_write(0x07);

    // read
    i2c_rep_start(dev[i] + I2C_READ);
    data_low = i2c_readAck(); //Read 1 byte and then send ack
    data_high = i2c_readAck(); //Read 1 byte and then send ack
    pec = i2c_readNak();
    i2c_stop();

    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point

    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor) - 0.01;


    sensordata[i + 1] = tempData - 273.15;
  }
}

void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT);
  //#START I2C For TIR INTIALIZE
  i2c_init();                                 //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);      //enable pullups
  //#END

  delay(3000);
}

void loop()
{
  lampa(6);
  UltraUPDATE();
  Serial.print(sensordata[4]);
  Serial.print(" - ");
  Serial.print(sensordata[5]);
  Serial.print(" - ");
  Serial.print(sensordata[6]);
  Serial.println();

  if (sensordata[6] > 25)
  {
    jobb();
    elore();
  }
  else if (sensordata[5] > 25)
  {
    elore();
  }
  else if (sensordata[4] > 25)
  {
    bal();
    elore();
  }
  else {
    fordul();
  }
  UltraUPDATE();
  delay(2000);
}
