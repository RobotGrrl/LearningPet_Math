/*
 
 Learning Pet - Math Demo
 -------------------------
 RobotGrrl.com/LearningPet
 
 Licensed under the BSD 3-Clause License
 
 */

#include <Wire.h>
#include <Servo.h>
#include <Streaming.h>

#define  LED3_RED       2
#define  LED3_GREEN     4
#define  LED3_BLUE      3

#define  LED2_RED       5
#define  LED2_GREEN     7
#define  LED2_BLUE      6

#define  LED1_RED       8       // eyes
#define  LED1_GREEN     10      // eyes
#define  LED1_BLUE      9       // eyes

#define  SERVO1         11      // right wing
#define  SERVO2         12      // left wing
#define  SERVO3         13      // beak
#define  SERVO4         27      // rotation

#define  TOUCH_RECV     14
#define  TOUCH_SEND     15

#define  RELAY1         A0
#define  RELAY2         A1

#define  LIGHT_SENSOR   A2
#define  TEMP_SENSOR    A3

#define  BUTTON1        A6
#define  BUTTON2        A7
#define  BUTTON3        A8

#define  JOY_SWITCH     A9      // pulls line down when pressed
#define  JOY_nINT       A10     // active low interrupt input
#define  JOY_nRESET     A11     // active low reset output

#define  ULTRASONIC     A14     // ultrasonic sensor (plug 2)
#define  ANSWER_SWITCH  A13      // answer switch (plug 2)

#define  WING_R_UPPER   30;
#define  WING_R_LOWER   90;

#define  WING_L_UPPER   110
#define  WING_L_LOWER   70      // accounts for the ultrasonic sensor height

#define  BEAK_OPEN      140
#define  BEAK_CLOSED    10

Servo servos[4];

#define PACKET_LENGTH 8
#define LEN (PACKET_LENGTH*2)
char msg[LEN];

int R_start = 0;
int G_start = 0;
int B_start = 0;
int R_pre = 0;
int G_pre = 0;
int B_pre = 0;

void setup();
void loop();

void init_buttons()
{
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
  pinMode(JOY_SWITCH, INPUT);

  // enable the internal pullups
  digitalWrite(BUTTON1, HIGH);
  digitalWrite(BUTTON2, HIGH);
  digitalWrite(BUTTON3, HIGH);
  digitalWrite(JOY_SWITCH, HIGH);
}

void init_relays()
{
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
}

void init_leds()
{
  digitalWrite(LED1_RED, 1);
  digitalWrite(LED1_GREEN, 1);
  digitalWrite(LED1_BLUE, 1);

  pinMode(LED1_RED, OUTPUT);
  pinMode(LED1_GREEN, OUTPUT);
  pinMode(LED1_BLUE, OUTPUT);

  analogWrite(LED1_RED, 128);
  analogWrite(LED1_GREEN, 128);
  analogWrite(LED1_BLUE, 128);

  digitalWrite(LED2_RED, 1);
  digitalWrite(LED2_GREEN, 1);
  digitalWrite(LED2_BLUE, 1);

  pinMode(LED2_RED, OUTPUT);
  pinMode(LED2_GREEN, OUTPUT);
  pinMode(LED2_BLUE, OUTPUT);

  digitalWrite(LED3_RED, 1);
  digitalWrite(LED3_GREEN, 1);
  digitalWrite(LED3_BLUE, 1);

  pinMode(LED3_RED, OUTPUT);
  pinMode(LED3_GREEN, OUTPUT);
  pinMode(LED3_BLUE, OUTPUT);
}

void init_joystick(int threshold);
byte b1, b2, b3, b4, c;
int sendingOrder = 0;

void setup()
{
  Serial.begin(115200);
  Serial.print("\r\nStart");

  init_leds();
  Serial.println("LEDs initialized");

  init_relays();
  Serial.println("relays initialized");

  init_buttons();
  Serial.println("buttons initialized");

  int p = WING_R_UPPER;

  servos[0].attach(SERVO1);
  servos[0].write(p);
  servos[1].attach(SERVO2);
  servos[1].write(WING_L_UPPER);
  servos[3].attach(SERVO4);
  servos[3].write(90);
  Serial.println("servos initialized");

  b1 = digitalRead(BUTTON1);
  b2 = digitalRead(BUTTON2);
  b3 = digitalRead(BUTTON3);
  b4 = digitalRead(JOY_SWITCH);
  c = 0;

  fade2(128, 128, 128,
  128, 128, 128,
  1);

}

void loop()
{
  mathProgram();
  delay(10);
}

void mathProgram() {

  refresh();

  int ultra = analogRead(ULTRASONIC);

  if(ultra >= 1000) {
    Serial << "~U" << ultra << "!";
  } 
  else if(ultra >= 100) {
    Serial << "~U0" << ultra << "!";
  } 
  else if(ultra >= 10) {
    Serial << "~U00" << ultra << "!";
  } 
  else if(ultra >= 0) {
    Serial << "~U000" << ultra << "!";
  }

  int switchVal = analogRead(ANSWER_SWITCH);

  if(switchVal >= 1000) {
    Serial << "~S" << switchVal << "!";
  } 
  else if(switchVal >= 100) {
    Serial << "~S0" << switchVal << "!";
  } 
  else if(switchVal >= 10) {
    Serial << "~S00" << switchVal << "!";
  } 
  else if(switchVal >= 0) {
    Serial << "~S000" << switchVal << "!";
  }

  Serial.flush();

}

void refresh() {

  int p = 0;

  while(Serial.available() > 0) {
    msg[p] = Serial.read();
    p++;
    if(p == LEN) break;
  }

  if(p == LEN) {
    p = 0;

    for(int i=0; i<LEN; i++) {
      char c = msg[i];

      switch(c) {
      case 'A':
        if(i < PACKET_LENGTH) parseAnswerMsg(i);
        break;
      case 'L':
        if(i < PACKET_LENGTH) parseLevelMsg(i);
        break;
      case 'D':
        if(i < PACKET_LENGTH) parseDoneMsg(i);
        break;
      }

    }

    Serial.flush();

  }

}

void parseAnswerMsg(int i) {

  int correct = ((int)msg[i+1]-48);

  if(correct == 1) {
    bothWings(2, 100);
  } 
  else {
    openBeak(10, 10);
    delay(50);
    closeBeak(10, 10);
  }

}

void parseLevelMsg(int i) {

  int level = ((int)msg[i+1]-48);

  updateLights();

  switch (level) {
  case 1:
    bothWings(5, 50);
    shake(3);
    break;
  case 2:
    bothWings(10, 50);
    shake(3);
    bothWings(10, 50);
    break;
  case 3:
    leftWing(3, 50);
    shake(3);
    rightWing(3, 50);
    shake(3);
    break;
  case 4:
    bothWings(10, 50);
    shake(5);
    rightWing(3, 50);
    break;
  case 5:
    shake(5);
    rightWing(3, 50);
    shake(5);
    leftWing(3, 50);
    shake(5);
    bothWings(10, 50);
    shake(5);
    break;
  default:
    break;
  }

  fade2(R_pre, G_pre, B_pre,
  128, 128, 128,
  1);

}

void parseDoneMsg(int i) {

  leftWing(5, 50);

}

void shake(int repeat) {

  for(int j=0; j<repeat; j++) {

    for(int i=90; i>60; i--) {
      servos[3].write(i);
    }
    delay(100);

    for(int i=60; i<120; i++) {
      servos[3].write(i);
    }
    delay(100);

    for(int i=120; i>90; i--) {
      servos[3].write(i);
    }
    delay(10);

  }


}

void leftWing(int repeat, int speed) {

  for(int j=0; j<repeat; j++) {

    for(int i=WING_L_LOWER; i<WING_L_UPPER; i++) {
      servos[1].write(i);
    }
    delay(speed);

    for(int i=WING_L_UPPER; i>WING_L_LOWER; i--) {
      servos[1].write(i);
    }
    delay(speed);

  }
}

void rightWing(int repeat, int speed) {

  int l = WING_R_LOWER;
  int u = WING_R_UPPER;

  for(int j=0; j<repeat; j++) {

    for(int i=u; i<l; i++) {
      servos[0].write(i);
    }
    delay(speed);

    for(int i=l; i>u; i--) {
      servos[0].write(i);
    }
    delay(speed);

  }
}

void bothWings(int repeat, int speed) {

  int rl = WING_R_LOWER;
  int ll = WING_L_LOWER;

  for(int j=0; j<repeat; j++) {

    for(int i=0; i<40; i++) {
      servos[0].write(rl-20-i);
      servos[1].write(ll+i);
    }
    delay(speed);

    for(int i=40; i>0; i--) {
      servos[0].write(rl-20-i);
      servos[1].write(ll+i);
    }
    delay(speed);

  }
}

void openBeak(int speed, int step) {

  int b = BEAK_OPEN;
  int currentPos = servos[2].read();

  servos[2].attach(SERVO3);

  if(currentPos > b) {
    for(int i=currentPos; i>b; i-=step) {
      servos[2].write(i);
      delay(speed);
    }
  } 
  else {
    for(int i=currentPos; i<b; i+=step) {
      servos[2].write(i);
      delay(speed);
    }
  }

  servos[2].detach();

}

void closeBeak(int speed, int step) {

  int b = BEAK_CLOSED;
  int currentPos = servos[2].read();

  servos[2].attach(SERVO3);

  if(currentPos > b) {
    for(int i=currentPos; i>b; i-=step) {
      servos[2].write(i);
      delay(speed);
    }
  } 
  else {
    for(int i=currentPos; i<b; i+=step) {
      servos[2].write(i);
      delay(speed);
    }
  }

  servos[2].detach();

}

void resetValues() {

  // reset outputs to default values on disconnect
  analogWrite(LED1_RED, 255);
  analogWrite(LED1_GREEN, 255);
  analogWrite(LED1_BLUE, 255);
  analogWrite(LED2_RED, 255);
  analogWrite(LED2_GREEN, 255);
  analogWrite(LED2_BLUE, 255);
  analogWrite(LED3_RED, 255);
  analogWrite(LED3_GREEN, 255);
  analogWrite(LED3_BLUE, 255);
  servos[0].write(90);
  servos[1].write(90);
  servos[2].write(90);
  servos[3].write(90);
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);

}

void updateLights() {

  R_start = int(random(50, 255));
  G_start = int(random(50, 255));
  B_start = int(random(50, 255));

  fade2( R_pre,    G_pre,      B_pre, 
  R_start,  G_start,    B_start, 
  1);

  R_pre = R_start;
  G_pre = G_start;
  B_pre = B_start;

}

void fade2 ( int start_R,  int start_G,  int start_B, 
int finish_R, int finish_G, int finish_B,
int stepTime ) {

  int skipEvery_R = 256/abs(start_R-finish_R); 
  int skipEvery_G = 256/abs(start_G-finish_G);
  int skipEvery_B = 256/abs(start_B-finish_B);

  for(int i=0; i<256; i++) {

    if(start_R<finish_R) {
      if(i<=finish_R) {
        if(i%skipEvery_R == 0) {
          analogWrite(LED1_RED, i);
        } 
      }
    } 
    else if(start_R>finish_R) {
      if(i>=(256-start_R)) {
        if(i%skipEvery_R == 0) {
          analogWrite(LED1_RED, 256-i); 
        }
      } 
    }

    if(start_G<finish_G) {
      if(i<=finish_G) {
        if(i%skipEvery_G == 0) {
          analogWrite(LED1_GREEN, i);
        } 
      }
    } 
    else if(start_G>finish_G) {
      if(i>=(256-start_G)) {
        if(i%skipEvery_G == 0) {
          analogWrite(LED1_GREEN, 256-i); 
        }
      } 
    }

    if(start_B<finish_B) {
      if(i<=finish_B) {
        if(i%skipEvery_B == 0) {
          analogWrite(LED1_BLUE, i);
        } 
      }
    } 
    else if(start_B>finish_B) {
      if(i>=(256-start_B)) {
        if(i%skipEvery_B == 0) {
          analogWrite(LED1_BLUE, 256-i); 
        }
      } 
    }

    delay(stepTime);

  }

}

