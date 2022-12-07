// # include <SPI.h>
// char str[ ]="Hello Slave, I'm Arduino Family\n";

// void setup() 
// {
// Serial.begin(115200); // set baud rate to 115200 for usart
// SPI.begin();
// SPI.setClockDivider(SPI_CLOCK_DIV8); //divide the clock by 8
// Serial.println("Hello I'm SPI Mega_Master");
// } 

// void loop (void)
//  {
//  digitalWrite(SS, LOW); // enable Slave Select
//  // send test string
//  for(int i=0; i< sizeof(str); i++) 
//  SPI.transfer( str [i] );
//  digitalWrite(SS, HIGH); // disable Slave Select
//  delay(2000);
// }

# include <SPI.h>
char str[ ]="Hello Slave, I'm Arduino Family\n";
int CK = 2; 
int SI = 3; // TX (in terms of master) 
int SO = 4; // RX
int CS = 5; 

void setup() 
{
//bool setRX(SO); 
//bool setCS(CS); 
//bool setSCK(CK); 
//bool setTX(SI); 

SPI.setRX(SO); 
SPI.setCS(CS); 
SPI.setSCK(CK); 
SPI.setTX(SI);


Serial.begin(115200); // set baud rate to 115200 for usart
SPI.begin(true); // Adding true IS IMPORTANT. 

//SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
// SPI.setClockDivider(SPI_CLOCK_DIV8); //divide the clock by 8
delay(10000);
Serial.println("Hello I'm SPI Mega_Master");
} 

void loop (void)
 {
 SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
 digitalWrite(SS, LOW); // enable Slave Select
 // send test string
 for(int i=0; i< sizeof(str); i++) 
 SPI.transfer( str [i] );
 SPI.endTransaction();
 digitalWrite(SS, HIGH); // disable Slave Select
 delay(2000);
}
