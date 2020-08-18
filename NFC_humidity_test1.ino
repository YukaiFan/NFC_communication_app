
#include<Wire.h>

// HTS221 I2C address is 0x5F
#define Addr 0x5F
#define Address 0x55
   unsigned int val[4];
   unsigned int data[2];
   unsigned int value[16];
   unsigned int  T0, T1, raw;
   float H0,H1;
   int H2, H3, T2, T3;
void setup() 
{
  // Initialise I2C communication as MASTER 
  Wire.begin();
  // Initialise serial communication, set baud rate = 9600
  Serial.begin(9600);
  
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select average configuration register
  Wire.write(0x10);
  // Temperature average samples = 256, Humidity average samples = 512
  Wire.write(0x1B);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select control register1
  Wire.write(0x20);
  // Power ON, Continuous update, Data output rate = 1 Hz
  Wire.write(0x85);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(300);

 // Humidity calliberation values
  for(int i = 0; i < 2; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Send data register(30&31)
    Wire.write((0x30 + i));
    // Stop I2C Transmission
    Wire.endTransmission();
    
    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);
    
    // Read 1 byte of data
    if(Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }
  
  // Convert Humidity data
  H0 = (float)data[0] / 2;
  H1 = (float)data[1] / 2;
 // Serial.println(H0);
 // Serial.println(H1);
  for(int i = 0; i < 2; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Send data register(36&37)
    Wire.write((0x36 + i));
    // Stop I2C Transmission
    Wire.endTransmission();
    
    // Request 1 byte of data
    Wire.requestFrom(Addr,1);
    
    // Read 1 byte of data
    if(Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }
  // Convert Humidity data
  H2 = (int)data[1] * 256 + (int)data[0];
  //H2 = abs(data[1] <<8 | data[0]);
  //Serial.println(H2);
  for(int i = 0; i < 2; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Send data register(3A&3B)
    Wire.write((0x3A + i));
    // Stop I2C Transmission
    Wire.endTransmission();
    
    // Request 1 byte of data
    Wire.requestFrom(Addr,1);
    
    // Read 1 byte of data
    if(Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }
  // Convert Humidity data
 //H3 = abs(data[1] <<8 | data[0]);
 H3 = (int)data[1] * 256 + (int)data[0];
 //Serial.println(data[0]);
 //Serial.println(data[1]);
 // Serial.println(H3);

   // Temperature calliberation values
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Send data register
  Wire.write(0x32);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr,1);

  // Read 1 byte of data
  if(Wire.available() == 1)
  {
    T0 = Wire.read();
  }

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Send data register
  Wire.write(0x33);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr,1);

  // Read 1 byte of data
  if(Wire.available() == 1)
  {
    T1 = Wire.read();
  }
  
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Send data register
  Wire.write(0x35);
  // Stop I2C Transmission
  Wire.endTransmission();
  
  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);

  // Read 1 byte of data
  if(Wire.available() == 1)
  {
    raw = Wire.read();
  }

  raw = raw & 0x0F;
  
  // Convert the temperature calliberation values to 10-bits
  T0 = ((raw & 0x03) * 256) + T0;
  T1 = ((raw & 0x0C) * 64) + T1;

  for(int i = 0; i < 2; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Send data register(3C&3D)
    Wire.write((60 + i));
    // Stop I2C Transmission
    Wire.endTransmission();
    
    // Request 1 byte of data
    Wire.requestFrom(Addr,1);
    
    // Read 1 byte of data
    if(Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }
  // Convert the data
  T2 = (data[1] * 256.0) + data[0];
    
  for(int i = 0; i < 2; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Send data register(3E&3F)
    Wire.write((62 + i));
    // Stop I2C Transmission
    Wire.endTransmission();
    
    // Request 1 byte of data
    Wire.requestFrom(Addr,1);
    
    // Read 1 byte of data
    if(Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }
  // Convert the data
  T3 = (data[1] * 256.0) + data[0];
  
}

void loop() 
{
//  int humidity_float[4];
//  int humidity[4];
//  int cTemp[4];
//  int cTemp_float[4];
  
  //for (int j = 0; j<2;j++){
    // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Send data register
  Wire.write(0x28 | 0x80);
  // Stop I2C Transmission
  Wire.endTransmission();  
  // Request 4 bytes of data
  Wire.requestFrom(Addr,4);
  // Read 4 bytes of data
  // humidity msb, humidity lsb, temp msb, temp lsb
  if(Wire.available() == 4)
  {
    val[0] = Wire.read();
    val[1] = Wire.read();
    val[2] = Wire.read();
    val[3] = Wire.read();
  }
  int number = abs (val[1] <<8 | val[0]);
  int number1 = val[1] <<8 | val[0];
  float humidity = ((1.0 * H1) - (1.0 * H0)) * (1.0 * number - 1.0 * H2) / (1.0 * H3 - 1.0 * H2) + (1.0 * H0);
  byte *floathumidity = (byte*) &humidity;
  int temp =val[3] <<8 | val[2];
  float temperature = (((T1 - T0) / 8.0) * (temp - T2)) / (T3 - T2) + (T0 / 8.0);
  byte *floattemperature = (byte*) &temperature;


  Serial.print("Relative humidity : ");
  Serial.print(humidity);
  Serial.println(" % RH");

  //Serial.println("Relative humidity in HEX: ");
  //Serial.println(floathumidity[0], HEX);
  //Serial.println(floathumidity[1], HEX);
  //Serial.println(floathumidity[2], HEX);
  //Serial.println(floathumidity[3], HEX);

  Serial.print("Temperature in Celsius : ");
  Serial.print(temperature);
  Serial.println(" C");

//  Serial.println("Temperature in Celsius in HEX: ");
//  Serial.println(floattemperature[0], HEX);
//  Serial.println(floattemperature[1], HEX);
//  Serial.println(floattemperature[2], HEX);
//  Serial.println(floattemperature[3], HEX);
  delay(500);
  


  Wire.begin();
  // Initialise serial communication, set baud rate = 9600
  Serial.begin(9600);
  // Start I2C Transmission
  Wire.beginTransmission(Address);
  Wire.write(0x01);
  
  for(int k = 0;k<4;k++){
    Wire.write(floathumidity[k]);
  }
  for(int k = 0;k<4;k++){
   Wire.write(floattemperature[k]);
  }
  for(int k = 0;k<8;k++){
     Wire.write(0x00);
  }
  Wire.endTransmission();
  delay(500);

//  Wire.beginTransmission(Address);
//  Wire.write(0x01);
//  Wire.endTransmission();

//  Wire.requestFrom(Address, 16);
//  if(Wire.available() == 16)
//   {
//    for (int i = 0; i<16; i++)
//    {
//     value[i] = Wire.read();
//    }  
//   }
//
//   for (int i = 0 ; i < 15; i++)
//   {
//    Serial.print(value[i] < 16 ? "0" : "");// if data[i]<16, output 0 or not ""
//    Serial.print(value[i],HEX);
//    Serial.print("-");
//   }
//   Serial.print(value[15] < 16 ? "0" : "");
//   Serial.println(value[15],HEX);
}
