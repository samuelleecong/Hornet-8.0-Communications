char Mymessage[5];
void setup() {
  // put your setup code here, to run once:
  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial.begin(9600);
  Serial2.begin(9600);
  pinMode(25,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//    while(!Serial1.available()){
//      Serial.println("save me");
//    }
    
    Serial2.readBytes(Mymessage,5); //Read the serial data and store in var
    Serial.println(Mymessage); //Print data on Serial Monitor
    Serial.println("help");
    delay(1000);
    digitalWrite(25,HIGH);
}
