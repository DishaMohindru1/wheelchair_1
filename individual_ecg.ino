
const int ecgPin = A1;  //defined the sensor input pin


int ecg_value; // declaration of variable to store ECG sensor value

void setup() {
  Serial.begin(9600);
}

void loop() {
 
  ecg_value = analogRead(ecgPin);
  
 
  Serial.println(ecg_value);   //print value to the serial monitor
  
  
  delay(2000);
}
