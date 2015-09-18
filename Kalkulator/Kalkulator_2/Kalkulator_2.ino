#include <Wire.h>
#include <LiquidCrystal_SR.h>
#include <Keypad.h>

#define SIDLE 0
#define SOPA 1
#define SOPB 2
#define SWAITC 3

//pin driving the buzzer
#define BEEP 5


LiquidCrystal_SR lcd(4,3,2);
//                   | | - Enable Pin
//                   | \-- Clock Pin
//                   \---- Data Pin



const byte ROWS = 5; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
  {
    '1','2','3', '+'                              }
  ,
  {
    '4','5','6', '-'                              }
  ,
  {
    '7','8','9', 'C'                              }
  ,
  {
    '*','0','/', '='                              }
  ,
  {
    '.', 'z', 'e', 'p'                              }
};
byte rowPins[ROWS] = {
  13, 12, 11, 10, 5}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {
  9, 8, 7, 6}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

int state = SOPA, cnt = 0, divisor = 10, deccnt = 0;
double a = 0, b = 0;
byte operacja, errorflag = 0, decpoint = 0;
boolean positive = true;

void setup(){
  // pinMode(BEEP, OUTPUT);
  lcd.begin(16,2);               // initialize the lcd

  lcd.home ();                   // go home
  lcd.clear();
  lcd.cursor();
}

void loop(){

  char key = keypad.waitForKey();
  // beep();

  switch(state){
  case SOPA:
    if((key >= '0') && (key <= '9')) {
      constructOperand(&a, key);
    }
    else if (key == 'z' && cnt > 0){
      changeSign(&a, SOPA);
    }
    else if (key == '.' && decpoint == 0){
      decpoint = 1;
      lcd.write(key);
    }
    else if(((key == '+') || (key == '-') || (key == '*') || (key == '/')) && cnt > 0){
      state = SOPB; 
      clearcddAndPrint(key);
    }
    else if(key == 'C')
      clearcalc();
    break;
  case SOPB:
    if((key >= '0') && (key <= '9')) {
      constructOperand(&b, key);
    }
    else if (key == 'z' && cnt > 0){
      changeSign(&b, SOPB);
      if (positive){
        lcd.write(' ');
        int pos = cnt + decpoint + 1;
        lcd.setCursor(pos, 1);
      }
    }
    else if (key == '.' && decpoint == 0){
      decpoint = 1;
      lcd.write(key);
    }
    else if(((key == '+') || (key == '-') || (key == '*') || (key == '/')) && cnt > 0) {
      calcanddisp();
      b = 0;
      clearcddAndPrint(key);
    }
    else if(key == '=' && cnt > 0) {
      state = SWAITC;
      calcanddisp();
    }
    else if(key == 'C')
      clearcalc();
    break;
  case SWAITC:
    if(key == 'C')
      clearcalc();
    else if(((key == '+') || (key == '-') || (key == '*') || (key == '/')) && cnt > 0){
      state = SOPB; 
      b = 0;
      clearcddAndPrint(key);
    }
    else if (key == 'z' && cnt > 0){
      changeSign(&a, SWAITC);
    }
    break;
  default:
    break;
  }
}

void clearcalc() {
  state = SOPA;
  a = 0;
  b = 0;
  errorflag = 0;
  cnt = 0;
  divisor = 10;
  decpoint = 0;
  deccnt = 0;
  positive = true;
  lcd.clear();
  lcd.cursor();
}

void beep(){
  digitalWrite(BEEP, HIGH);
  delay(100);
  digitalWrite(BEEP, LOW);
}


void calcanddisp(){
  switch(operacja){
  case '+':
    a = a + b;
    break;
  case '-':
    a = a - b;
    break;
  case '*':
    a = a * b;
    break;
  case '/':
    if( b != 0)
      a = a / b;
    else
    {
      lcd.clear();
      lcd.noCursor();
      lcd.print("ERROR");
      errorflag = 1;
    }
    break;
  default:
    break;
  }
  if(errorflag == 0){
    lcd.clear();
    lcd.noCursor();
    lcd.print(a);
  }
}

void clearcddAndPrint(byte _key) {
  cnt = 0;
  divisor = 10;
  decpoint = 0;
  deccnt = 0;
  positive = true;
  operacja = _key;
  lcd.setCursor(0,1);
  lcd.write(_key);
}

void constructOperand(double *op, byte _key) {
  if(cnt < 7 && deccnt < 2){
    cnt++;
    *op = *op * (decpoint == 0 ? 10 : 1);
    *op = *op + (_key - '0') * (positive ? 1.0 : -1.0) / (decpoint == 0 ? 1.0 : (double) divisor);
    if (decpoint == 1){ 
      divisor *= 10;
      deccnt++;
    }
    lcd.write(_key);
  }
}

void changeSign(double *op, int _state) {
  *op *= -1.0;
  positive = !positive; 
  if(_state == SOPB)
    lcd.setCursor(1,1);
  else
    lcd.clear();
  lcd.print(*op,deccnt);
  if(decpoint && !deccnt)
    lcd.write('.');
}

