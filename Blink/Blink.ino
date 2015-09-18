/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
int dashLen = 600;
int dotLen = 200;
int intraPause = 200;
int interPause = 400;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
}

void dash() {
  digitalWrite(led, HIGH);
  delay(dashLen);
  digitalWrite(led, LOW);
  delay(intraPause);
}

void dot() {
  digitalWrite(led, HIGH);
  delay(dotLen);
  digitalWrite(led, LOW);
  delay(intraPause);
}

void pause() {
  delay(interPause);
}

void morseC() {
  dash();
  dot();
  dash();
  dot();
}

void morseQ() {
  dash();
  dash();
  dot();
  dash();
}
  
// the loop routine runs over and over again forever:
void loop() {
  morseC();
  pause();
  morseQ();
  pause();
  pause();
}
