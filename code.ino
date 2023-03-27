#include <Wire.h>

#define m1 5
#define m2 6
#define m3 11
#define m4 10

#define Speed 1

void setup()
{
  Serial.begin(9600);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
}

void loop()
{
  if (Serial.available() == 1)
  {
    voicecontrol();
  }
}

void voicecontrol()
{
  String val = Serial.readString();
  Serial.println(val);
  if (val == "forward")
  {
    Stop();
    Forward();
  }
  if (val == "backward")
  {
    Stop();
    Backward();
  }
  if (val == "left")
  {
    Stop();
    Left();
  }
  if (val == "right")
  {
    Stop();
    Right();
  }
  if (val == "stop")
  {
    Stop();
  }
}

void Forward()
{
  digitalWrite(m1, Speed);
  digitalWrite(m3, Speed);
}

void Backward()
{
  digitalWrite(m2, Speed);
  digitalWrite(m4, Speed);
}

void Left()
{
  digitalWrite(m3, Speed);
}

void Right()
{
  digitalWrite(m1, Speed);
}

void Stop()
{
  digitalWrite(m1, 0);
  digitalWrite(m2, 0);
  digitalWrite(m3, 0);
  digitalWrite(m4, 0);
}
