void setup(){
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  Serial.println("Starting up");
}

void loop(){
  digitalWrite(13,HIGH);
}//end of code
