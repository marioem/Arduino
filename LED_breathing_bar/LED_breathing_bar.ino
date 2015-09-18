/*
LED breathing bar project.
5-LED bar, each LED is gradually switched on untill all LEDs are lit.
Then each LED is gradually switched off.
*/

int d = 5;
int pins[5] = {
  11, 10, 9, 6, 5};

void setup()
{
  for (int i = 0; i < 5; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
}
void loop() {
  for (int i = 0; i < 5; i++)
  {
    for ( int a = 0 ; a < 256 ; a++ )
    {
      analogWrite(pins[i], a);
      delay(d); 
    }
  }
  for (int i = 0; i < 5; i++)
  {
    for ( int a = 255 ; a >= 0 ; a-- )
    {
      analogWrite(pins[i], a);
      delay(d); 
    }
  }
  //delay(200);
}





