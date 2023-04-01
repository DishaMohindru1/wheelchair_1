const int led_pin = 2;  // output pin for LED
const int sensor_pin = A1; // input pin for pulse oximeter sensor


int sensor_value; // Variable to store value of pulse oximeter sensor 
int threshold = 500; // Threshold value to detect pulse

void setup() {
  
  pinMode(led_pin, OUTPUT);   //led pin is initialized as output pin
  Serial.begin(9600);   
}

void loop() {
 
  sensor_value = analogRead(sensor_pin);   //reading of the values by the sensor
  Serial.println(sensor_value);
  
  // If sensor value is above threshold, turn on LED
  if (sensor_value > threshold) {
    digitalWrite(led_pin, HIGH);
  }
  else {
    digitalWrite(led_pin, LOW);
  }
  
  delay(2000);
}
