#define potRul A0
#define potStop A1
#define potKpp A2
#define rullNotAuto A4

#define rulA 7
#define rulB 8
#define rulPWM 6

#define stopA 2
#define stopB 3
#define stopPWM 9

#define drive 11

#define kppA 4
#define kppB 5
#define kppPWM 10


#define pauseBt 36
#define startBt 38
#define stopBt 40
#define autoNotAuto 42
#define reversBt 46
#define driveBt 48
#define ksBt 12


#include <Servo.h>

Servo servoDrive;

int needRul, needKpp, needStop, needPwmRul, needDrive, potStopVal, potKppVal;
int driveKpp = 65, reversKpp = 171, nKpp = 115, pKpp = 240, stopOn = 180, stopOff = 135, servoDriveOn = 50, servoDriveOff = 30;
int potRulVal;
int rulValues[9] = {200, 270, 340, 410, 480, 550, 620, 690, 760};
int targetVelocity[2] = {0, 0};
int rullNotAutoVal;

String to = "N";
String from = "";
String stopValue;
unsigned long timing;
unsigned long timingRul;
String kpp;
int n;
int pwm = 255;
String inputString = "";
bool stringComplete = false;
String mainString = "";
int val;

volatile int pauseVal, startVal, stopVal, ksVal, autoNotAutoVal, driveBtVal, reversBtVal;

void setup() {

  pinMode(pauseBt, INPUT_PULLUP);
  pinMode(startBt, INPUT_PULLUP);
  pinMode(stopBt, INPUT_PULLUP);
  pinMode(autoNotAuto, INPUT_PULLUP);
  pinMode(driveBt, INPUT_PULLUP);
  pinMode(reversBt, INPUT_PULLUP);
  pinMode(ksBt, INPUT_PULLUP);

  servoDrive.attach(drive);

  pinMode(potRul, INPUT);
  pinMode(rulA, OUTPUT);
  pinMode(rulB, OUTPUT);
  pinMode(rulPWM, OUTPUT);

  pinMode(potStop, INPUT);
  pinMode(stopA, OUTPUT);
  pinMode(stopB, OUTPUT);
  pinMode(stopPWM, OUTPUT);

  pinMode(potKpp, INPUT);
  pinMode(kppA, OUTPUT);
  pinMode(kppB, OUTPUT);
  pinMode(kppPWM, OUTPUT);

  Serial.begin(115200);
  needKpp = nKpp;
  needStop = stopOff;
  needRul = rulValues[4];
  needDrive = servoDriveOff;
  timingRul = millis();


}

void loop() {
  
  
  pauseVal = !digitalRead(pauseBt);
  stopVal = digitalRead(stopBt);
  startVal = !digitalRead(startBt);
  ksVal = !digitalRead(ksBt);
  autoNotAutoVal = !digitalRead(autoNotAuto);
  reversBtVal = !digitalRead(reversBt);
  driveBtVal = !digitalRead(driveBt);
  potRulVal = analogRead(potRul);
  rullNotAutoVal = analogRead(rullNotAuto);
  potStopVal = analogRead(potStop);
  potKppVal = analogRead(potKpp);

  //Serial.println(autoNotAutoVal);

  if (potStopVal > stopOff) {
    stopValue = "1";
  }
  if (potStopVal <= stopOff + 10) {
    stopValue = "0";
  }

  //ksVal = 1;
  //autoNotAutoVal = 0;

  if (ksVal == 1 && autoNotAutoVal == 0) {
    servoDrive.write(needDrive);

    if (stringComplete) {
      stringComplete = false;
      
      //Serial.println(mainString);
      testParsing();
      needRul = map(targetVelocity[1], -80, 80, rulValues[0], rulValues[8]);

      if(targetVelocity[0] > 0.0){
        needKpp = driveKpp;
        from = to;
        to = "F";
      }
      else if (targetVelocity[0] < 0.0) {
        needKpp = reversKpp;
        from = to;
        to = "R";
      }

      else{
        needKpp = nKpp;
        from = to;
        to = "N";
      }
      
      mainString = "";
    }
    //Serial.print(to);
      //Serial.print(" ");
      //Serial.println(kpp);
      if((kpp == to || kpp == to+to) && kpp != "N") {
          
          needStop = stopOff;
          needDrive = servoDriveOn;
        }
        
      else{
          needStop = stopOn;
          needDrive = servoDriveOff;
        }
  }



  else if (ksVal == 0 && autoNotAutoVal == 0) {
    needStop = stopOn;
    needDrive = servoDriveOff;
    needKpp = nKpp;
  }

  else if (autoNotAutoVal == 1) {
    //Serial.println("Not auto");
    notAuto();
  }

  moveRulType2();
  moveStop();
  fromToKpp();

  if (millis() - timing > 250) {
    timing = millis();
    Serial.print("CVL");
    Serial.print(":");
    Serial.print(analogRead(potRul));
    Serial.print(":");
    Serial.print(kpp);
    Serial.print(":");
    Serial.print(stopValue);
    Serial.print(":");
    Serial.print(stopVal);
    Serial.print(":");
    Serial.print(pauseVal);
    Serial.print(":");
    Serial.print(startVal);
    Serial.print(":");
    Serial.println(ksVal);    
    //Serial.print("#");
    //Serial.print('\n');
    //stop
  }
}
void notAuto() {
  //digitalWrite(drive, LOW);
  needRul = map(rullNotAutoVal, 0, 1023, 760, 200);
  needStop = stopOff;
  if (reversBtVal == 1) {
    needKpp = reversKpp;
  }

  else if (driveBtVal == 1) {
    needKpp = driveKpp;
  }

  else {
    needKpp = nKpp;
  }
}

void fromToKpp() {

  if (potKppVal < needKpp - 3) {
    digitalWrite(kppA, HIGH);//вперёд
    digitalWrite(kppB, LOW);
    analogWrite(kppPWM, pwm);
    kpp = from + to;
  }

  else if (potKppVal > needKpp + 3) {
    digitalWrite(kppA, LOW);//вперёд
    digitalWrite(kppB, HIGH);
    analogWrite(kppPWM, pwm);
    kpp = from + to;
  }

  else {
    digitalWrite(kppA, LOW);//стоп
    digitalWrite(kppB, LOW);
    analogWrite(kppPWM, 0);
    kpp = to;
  }

}

void moveStop() {
  if (analogRead(potStop) < needStop - 3) {
    digitalWrite(stopA, LOW);//вперёд
    digitalWrite(stopB, HIGH);
    analogWrite(stopPWM, pwm);
  }

  else if (analogRead(potStop) > needStop + 3) {
    digitalWrite(stopA, HIGH);//вперёд
    digitalWrite(stopB, LOW);
    analogWrite(stopPWM, pwm);
  }

  else {
    digitalWrite(stopA, LOW);
    digitalWrite(stopB, LOW);
    analogWrite(stopPWM, 0);
  }
}

void moveRul() {
  int diff = abs(potRulVal - needRul);
  if (potRulVal > needRul + 2) {

    if (diff < 40) {
      needPwmRul = map(diff, 40, 0, 255, 40);
    }
    else {
      needPwmRul = 255;
    }
    digitalWrite(rulA, LOW);//вправо
    digitalWrite(rulB, HIGH);
    analogWrite(rulPWM, needPwmRul);
  }

  else if (potRulVal < needRul - 2) {

    if (diff < 40) {
      needPwmRul = map(diff, 40, 0, 255, 40);
    }
    else {
      needPwmRul = 255;
    }

    digitalWrite(rulA, HIGH);//влево
    digitalWrite(rulB, LOW);
    analogWrite(rulPWM, needPwmRul);
  }

  else {
    timingRul = millis();
    digitalWrite(rulA, LOW);
    digitalWrite(rulB, LOW);
    analogWrite(rulPWM, 0);
  }
  //}
}

void moveRulType2() {
  int diff = abs(potRulVal - needRul);
  if (potRulVal > needRul + 2) {

    if (diff < 40) {
      needPwmRul = map(diff, 40, 0, 255, 40);
    }
    else {
      needPwmRul = 255;
    }
    analogWrite(rulA, 0);//вправо
    analogWrite(rulB, needPwmRul);
  }

  else if (potRulVal < needRul - 2) {

    if (diff < 40) {
      needPwmRul = map(diff, 40, 0, 255, 40);
    }
    else {
      needPwmRul = 255;
    }

    analogWrite(rulA, needPwmRul);//влево
    analogWrite(rulB, 0);
  }

  else {
    timingRul = millis();
    digitalWrite(rulA, 0);
    digitalWrite(rulB, 0);
  }
}

void moveKppType2() {

  if (potKppVal < needKpp - 6) {

    analogWrite(kppA, 0);//вправо
    analogWrite(kppB, 255);
    kpp = from + to;
  }

  else if (potKppVal > needKpp + 6) {

    analogWrite(kppA, 255);//влево
    analogWrite(kppB, 0);
    kpp = from + to;
  }

  else {
    digitalWrite(kppA, 0);
    digitalWrite(kppB, 0);
    kpp = to;
  }
}

void serialEvent() {

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    mainString += inChar;
    if (inChar == '\n'){ 
      stringComplete = true;
      break;
    }
  }
  for (int i = 0; i < 2; i++) { //ПРОВЕРИТЬ
    targetVelocity[i] = 0;
  }
}





void testParsing() {
  byte dividerIndex = mainString.indexOf(';');
  String buf1 = mainString.substring(0, dividerIndex);
  parsingLocal(buf1);
  mainString = mainString.substring(dividerIndex + 1);
  if (mainString != NULL) testParsing();
}

void parsingLocal(String tempString) {

  byte dividerIndex = tempString.indexOf(':');
  String buf1 = tempString.substring(0, dividerIndex);
  uint32_t ind = 0;
  dividerIndex++;


  if (buf1.equals("tv")) {
    for (uint8_t index = 0; index < 2; index++) {
      ind = tempString.indexOf(',', dividerIndex + 1);
      targetVelocity[index] = ((tempString.substring(dividerIndex, ind).toFloat()) * 100);
      dividerIndex = ind + 1;
    }
  }
}

int analogReadStab(byte analogPin) {

  int sum = 0;

  for (int p = 0; p <= 15; p++) {
 
    sum += analogRead(analogPin);
 
  }

  sum = sum>>4;

  return sum;

}

//вправо руль-значения уменшаются
// 450 - вправо до упора
// 550 влево до упора
//digitalWrite(rulA, HIGH);//влево
//digitalWrite(rulB, LOW);
