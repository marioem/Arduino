/*
 LED breathing bar project.
 5-LED bar, each LED is gradually switched on untill all LEDs are lit.
 Then each LED is gradually switched off.
 The whole process is toggled by a button press
 LEDs are connected to PWM outputs 11, 10, 9, 6, 5.
 Potentomenter's slider is conencted to analog input A0.
 */

#define BUTTON 7
#define UP 1
#define DOWN 0
#define POT 0

const int segment = 1023 / 5;
const float scaling = (255.0 / ((float)segment - 1.0));

int d = 5;
int pins[5] = {
  11, 10, 9, 6, 5};
int analogVal = 0;

void setup()
{
  for (int i = 0; i < 5; i++)
  {
    pinMode(pins[i], OUTPUT);
    analogWrite(pins[i],0);
  }
}

void loop() {
  int led;
  float brightness;

  analogVal = analogRead(POT);

  led = analogVal / segment;  // determining highest LED that should be lit

  if(led <= 4)
    brightness = ((float)analogVal - (float)led * (float)segment) * scaling;
  else  // due to some rounding error we could end up outside of our five-led range, so clamping is required
  {
    led = 4;
    brightness = 255.0;
  }

  for(int i = 0; i < 5; i++)
  {
    if(i < led)
      analogWrite(pins[i],255);
    if(i == led)
      analogWrite(pins[i],(int)brightness);
    if(i > led)
      analogWrite(pins[i],0);
  }
}

