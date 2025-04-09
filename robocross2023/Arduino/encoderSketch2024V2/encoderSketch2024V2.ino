// алгоритм с "таблицей", позволяющий увеличить точность энкодера
// в 4 раза, работает максимально чётко даже с плохими энкодерами.
// Для увеличения скорости опроса используйте PCINT и чтение из PINn
#define CLK1 2
#define DT1 4

#define CLK2 3
#define DT2 5

byte lastState1 = 0;
byte lastState2 = 0;

const int8_t increment[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

volatile int encCounter1;
volatile int encCounter2;
uint32_t timer = 0; 

void setup() {
  Serial.begin(115200);
}
void loop() {
  byte state1 = digitalRead(CLK1) | (digitalRead(DT1) << 1);
  if (state1 != lastState1) {
    encCounter1 += increment[state1 | (lastState1 << 2)];
    lastState1 = state1;
  }

  byte state2 = digitalRead(CLK2) | (digitalRead(DT2) << 1);
  if (state2 != lastState2) {
    encCounter2 += increment[state2 | (lastState2 << 2)];
    lastState2 = state2;
  }

  if (micros() - timer > 100000) { // таймер на micros()
    timer = micros(); // сброс
    Serial.print(encCounter1);
    Serial.print(";");
    Serial.println(encCounter2);
    encCounter1 = 0;
    encCounter2 = 0;
    
  }
}
