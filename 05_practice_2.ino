#define PIN_LED 7
bool on,off;

void setup(){
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); //Initialize serial port
  while (!Serial){
    ;//wait for serial port to connect.
  }
  on=false;
  off=true;
  digitalWrite(PIN_LED,on);
  delay(1000);
  for(int i=1;i<=5;i++){
    digitalWrite(PIN_LED, off);
    delay(100);
    digitalWrite(PIN_LED, on);
    delay(100);
  }
}

void loop(){
  while(1){
    digitalWrite(PIN_LED, off);
    }
 }
 

  
  
