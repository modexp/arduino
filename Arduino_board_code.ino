//Arduino 1.0+ Only

#include <Wire.h>
#include <math.h>


#define BMP085_ADDRESS 0x77  // I2C address of BMP085

#include <FreqCounter.h>

int freq, offset, sens;

const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
int time;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

void setup(){
  Serial.begin(9600);
  Wire.begin();

  bmp085Calibration();
  
  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(byte(0x00)); //continuous measurement mode
  Wire.endTransmission();
  
  Wire.beginTransmission(0x1E);
  Wire.write(0x01);
  Wire.write(byte(0x00)); //for gain see therefore HMC5883L.cpp and HMC5883L.h
  Wire.endTransmission();
  
 sens    =  i2cRead2bytes(81, 10); //Read sensitivity from EEPROM
 offset =  i2cRead2bytes(81, 12); //Same for offset
  
  
  
}

void loop()
{
  // check for the command to write data to serial port
  if ((!Serial.available()) || (Serial.read() != 'r')) {
    return;
  }
  
  //
  //pressure sensor and temperature read out 
  //
  float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pressure = bmp085GetPressure(bmp085ReadUP());

  //
  //magnetic field sensor read out 
  //
  int x,y,z; //triple axis data
  float x1,y1,z1,r;

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(0x1E);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(0x1E, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  //x1=x; y1=y; z1=z; //for gain see therefore HMC5883L.cpp and HMC5883L.h
  x1=((float)x*.73);
  y1=((float)y*.73);
  z1=((float)z*.73);
  r= (sqrt((square(x1))+(square(y1))+(square(z1))));
  
  //
  //humidity sensor readout 
  //
  //Get Frequency
  FreqCounter::f_comp= 8;             // Set compensation to 12
  FreqCounter::start(1000);            // Start counting with gatetime of 1000ms
  while (FreqCounter::f_ready == 0)         // wait until counter ready 
  freq=FreqCounter::f_freq;            // read result
 
 //Calculate RH
 //float RH =  (offset-freq)*sens/4096; //Sure, you can use int - depending on what do you need
 //float RH = ((float)(offset -freq))*(float)sens/4096.0;
 float RH = ((float)(offset -freq)*sens)/4096.0;
 
 //delay(1000); //wait a second and get values again.
 
 //print out the values as a final step
 
  Serial.print("temperature: ");
  Serial.print(temperature, 2); //display 2 decimal places in degrees celcius
  //Serial.println("deg C");
  Serial.print("/");

  Serial.print("pressure: ");
  Serial.print(pressure, 0); //whole number only. in Pascal
  //Serial.println(" Pa");
  Serial.print("/");
 // Serial.println();//line break
 
  //Print out values of each axis
  Serial.print("magneticfieldx: ");
  Serial.print(x1);
  Serial.print("/");
  Serial.print("magneticfieldy: ");
  Serial.print(y1);
  Serial.print("/");
  Serial.print("magneticfieldz: ");
  Serial.print(z1);
  Serial.print("/");
  Serial.print("magneticfieldtotal: "); // possibly in milli gauss (depending on the gain0)
  Serial.print(r);
  Serial.print("/");
  //Serial.println("mG");
  //Serial.println(); line break
  
 Serial.print("humidity: ");
 Serial.print(RH);
 Serial.print("/");
 Serial.print("seconds: ");
 time= time+1;
 Serial.print(time); //print the value for humidity
 Serial.println("//");
 //Serial.println();//line break
 
}

int i2cRead2bytes(int deviceaddress, byte address)  
{
 // SET ADDRESS
 Wire.beginTransmission(deviceaddress);
 Wire.write(address); // address for sensitivity
 Wire.endTransmission();

 // REQUEST RETURN VALUE
 Wire.requestFrom(deviceaddress, 2);
 // COLLECT RETURN VALUE
 int rv = 0;
 for (int c = 0; c < 2; c++ )
 if (Wire.available()) rv = rv * 256 + Wire.read();
 return rv;
  
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

