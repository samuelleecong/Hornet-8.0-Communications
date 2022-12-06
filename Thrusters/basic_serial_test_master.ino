// SENDING FROM NANO TO PICO (MASTER -> SLAVE)

char Mymessage[5] = "Hello"; //String data

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.write(Mymessage,5); //Write the serial data
  delay(1000);
}
