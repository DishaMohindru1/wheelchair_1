#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();  // Create a new instance of the MLX90614 object

void setup() {
  Serial.begin(9600);    
  Wire.begin();  // Initialize I2C communication  
  mlx.begin();  // Initialize the MLX90614 sensor
}

void loop() {
  
  float temp_ambient = mlx.readAmbientTempC();   // ambient temperature from the sensor
    
  float temp_object = mlx.readObjectTempC();     // object temperature from the sensor
  
  Serial.print("Ambient temperature: ");
  Serial.print(temp_ambient);
  Serial.println(" degrees C");
  
  Serial.print("Object temperature: ");
  Serial.print(temp_object);
  Serial.println(" degrees C");
  
  delay(2000);
}
