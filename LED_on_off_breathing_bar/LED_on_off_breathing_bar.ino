/*
 LED breathing bar project.
 5-LED bar, each LED is gradually switched on untill all LEDs are lit.
 Then each LED is gradually switched off.
 The whole process is toggled by a button press
*/
 
#define BUTTON 7
#define UP 1
#define DOWN 0

int d = 5;
int pins[5] = {
  11, 10, 9, 6, 5};
int onOff = 0;
boolean buttonState = 0;
int i = 0, j = 0, a = 0, b = 255;
int phase = UP;

void setup()
{
  pinMode(BUTTON, INPUT);
  for (int i = 0; i < 5; i++)
  {
    pinMode(pins[i], OUTPUT);
  }
}

void loop() {
  if(digitalRead(BUTTON) == HIGH)
  {
    while(digitalRead(BUTTON) == HIGH)
      ;
    buttonState = ~buttonState;
  }
  if (buttonState)
  {
    if(phase == UP)
    {
      for (; i < 5; i++)
      {
        for (; a < 256 ; a++ )
        {
          analogWrite(pins[i], a);
          delay(d);
          if(digitalRead(BUTTON) == HIGH)
          {
            while(digitalRead(BUTTON) == HIGH)
              ;
            buttonState = ~buttonState;
            goto skip;
          }      
        }
        if(a >= 256)
          a = 0;
      }
      if(i >= 5)
        i = 0;
      phase = DOWN;
      delay(10*d);
    }
    else
    {
      for (; j < 5; j++)
      {
        for (; b >= 0 ; b-- )
        {
          analogWrite(pins[j], b);
          delay(d);
          if(digitalRead(BUTTON) == HIGH)
          {
            while(digitalRead(BUTTON) == HIGH)
              ;
            buttonState = ~buttonState;
            goto skip;
          }
        }
        if(b < 0)
          b = 255;  
      }
      if(j >= 5)
        j = 0;
      phase = UP;
    }
  }
skip:
  delay(10*d);
}

