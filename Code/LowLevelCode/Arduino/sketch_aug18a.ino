S#include <Arduino.h>

const float COUNTS_PER_REV_ALPHA = 4338.8;
const float COUNTS_PER_REV_BRAVO = 4267.2;
const float COUNTS_PER_REV_CHARLIE = 4345.0;
const float COUNTS_PER_REV_DELTA = 4324.4;

const unsigned long TARGET_INTERVAL = 100; // измеряем каждые 100 ms

// --- Пины (под Arduino Nano) ---
const uint8_t ALPHA_A_PIN   = 2;  // PD2  PCINT18
const uint8_t ALPHA_B_PIN   = 4;  // PD4  PCINT20
const uint8_t BRAVO_A_PIN   = 3;  // PD3  PCINT19
const uint8_t BRAVO_B_PIN   = 5;  // PD5  PCINT21
const uint8_t CHARLIE_A_PIN = 6;  // PD6  PCINT22
const uint8_t CHARLIE_B_PIN = 8;  // PB0  PCINT0
const uint8_t DELTA_A_PIN   = 7;  // PD7  PCINT23
const uint8_t DELTA_B_PIN   = 9;  // PB1  PCINT1

// --- Глобальные счётчики (в прерываниях изменяются) ---
volatile long alpha_encoder_value   = 0;
volatile long bravo_encoder_value   = 0;
volatile long charlie_encoder_value = 0;
volatile long delta_encoder_value   = 0;

// предыдущие 2-битные состояния (0..3)
volatile uint8_t prevAlphaState   = 0;
volatile uint8_t prevBravoState   = 0;
volatile uint8_t prevCharlieState = 0;
volatile uint8_t prevDeltaState   = 0;

// Таблица переходов (prev<<2 | curr) -> delta
const int8_t transTable[16] = {
  0,  1, -1,  0,
 -1,  0,  1,  0,
  1,  0,  0, -1,
  0, -1,  1,  0
};

// маски для чтения портов
const uint8_t MASK_PORTD = (1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7); // PD2..PD7
const uint8_t MASK_PORTB = (1<<PB0)|(1<<PB1); // PB0..PB1 (D8,D9)

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("PCINT 4 encoders - corrected"));

  // входы энкодеров — используем внутренние подтяжки (если провода короткие)
  pinMode(ALPHA_A_PIN,   INPUT_PULLUP);
  pinMode(ALPHA_B_PIN,   INPUT_PULLUP);
  pinMode(BRAVO_A_PIN,   INPUT_PULLUP);
  pinMode(BRAVO_B_PIN,   INPUT_PULLUP);
  pinMode(CHARLIE_A_PIN, INPUT_PULLUP);
  pinMode(CHARLIE_B_PIN, INPUT_PULLUP);
  pinMode(DELTA_A_PIN,   INPUT_PULLUP);
  pinMode(DELTA_B_PIN,   INPUT_PULLUP);

  // инициализируем prev-state по текущим уровням (чтобы не получить скачок при старте)
  uint8_t portd = PIND;
  uint8_t portb = PINB;
  prevAlphaState   = ((portd & (1<<PD2)) ? 1 : 0) | ((portd & (1<<PD4)) ? 2 : 0);
  prevBravoState   = ((portd & (1<<PD3)) ? 1 : 0) | ((portd & (1<<PD5)) ? 2 : 0);
  prevCharlieState = ((portd & (1<<PD6)) ? 1 : 0) | ((portb & (1<<PB0)) ? 2 : 0);
  prevDeltaState   = ((portd & (1<<PD7)) ? 1 : 0) | ((portb & (1<<PB1)) ? 2 : 0);

  // включаем PCINT для портов D и B
  cli();
  // порт D: PCINT18..23 => PD2..PD7
  PCMSK2 = 0;
  PCMSK2 |= (1<<PCINT18) | (1<<PCINT19) | (1<<PCINT20) | (1<<PCINT21) | (1<<PCINT22) | (1<<PCINT23);
  // порт B: PCINT0..1 => PB0..PB1
  PCMSK0 = 0;
  PCMSK0 |= (1<<PCINT0) | (1<<PCINT1);
  // разрешить прерывания по группам
  PCICR |= (1<<PCIE2); // порт D
  PCICR |= (1<<PCIE0); // порт B
  sei();
}

float miliseconds_counter = 0;
int tics_counter = 0;
float integratorA = 0.0;
float integratorB = 0.0;
float integratorC = 0.0;
float integratorD = 0.0;

//static unsigned long last = 0;
static long last_alpha = 0, last_bravo = 0, last_charlie = 0, last_delta = 0;

void loop() {
    static unsigned long last_time = micros();
    unsigned long now = micros();
    unsigned long dt_us = now - last_time;
    float T = dt_us / 1e6f;  // Реальное время в секундах
    miliseconds_counter += dt_us / 1e3f;
    tics_counter++;
    last_time = now;
    
    //Serial.println(integratorA);
    //Serial.println(tics_counter);
    //Serial.println(miliseconds_counter);

//    static unsigned long last = 0;
//    static long last_alpha = 0, last_bravo = 0, last_charlie = 0, last_delta = 0;

     // атомарно читаем volatile счётчики
    noInterrupts();
    long a = alpha_encoder_value;
    long b = bravo_encoder_value;
    long c = charlie_encoder_value;
    long d = delta_encoder_value;
    interrupts();

    long da = a - last_alpha;
    long db = b - last_bravo;
    long dc = c - last_charlie;
    long dd = d - last_delta;

    last_alpha = a; last_bravo = b; last_charlie = c; last_delta = d;

    //float T = INTERVAL_MS / 1000.0f; // сек
    float rpmA = (da / COUNTS_PER_REV_ALPHA) * (60.0f / T);
    float rpmB = (db / COUNTS_PER_REV_BRAVO) * (60.0f / T);
    float rpmC = (dc / COUNTS_PER_REV_CHARLIE) * (60.0f / T);
    float rpmD = (dd / COUNTS_PER_REV_DELTA) * (60.0f / T);

    integratorA += rpmA;
    integratorB += rpmB;
    integratorC += rpmC;
    integratorD += rpmD;
/*
    Serial.print("A:"); Serial.print(rpmA, 2);
    Serial.print(",B:"); Serial.print(rpmB, 2);
    Serial.print(",C:"); Serial.print(rpmC, 2);
    Serial.print(",D:"); Serial.println(rpmD, 2);
*/
    //Serial.println(miliseconds_counter);
    if(miliseconds_counter >= TARGET_INTERVAL) {

      if (tics_counter == 0) tics_counter = 1;

      float outA = integratorA / tics_counter;
      float outB = integratorB / tics_counter;
      float outC = integratorC / tics_counter;
      float outD = integratorD / tics_counter;

      if (!isfinite(outA)) outA = 0.0f;
      if (!isfinite(outB)) outB = 0.0f;
      if (!isfinite(outC)) outC = 0.0f;
      if (!isfinite(outD)) outD = 0.0f;
      
      Serial.print("A:"); Serial.print(outA, 2);
      Serial.print(",B:"); Serial.print(outB, 2);
      Serial.print(",C:"); Serial.print(outC, 2);
      Serial.print(",D:"); Serial.println(outD, 2);

      miliseconds_counter = 0;
      tics_counter = 0;
      integratorA = 0;
      integratorB = 0;
      integratorC = 0;
      integratorD = 0;

    }
    // Отправляем CSV строку. Формат: A:xxx.x,B:yyy.y,C:...
    // Можно уменьшить количество знаков через форматирование.
    /*char buf[128];
    int n = snprintf(buf, sizeof(buf), "A:%.2f,B:%.2f,C:%.2f,D:%.2f\n", rpmA, rpmB, rpmC, rpmD);
    Serial.write((uint8_t*)buf, n);*/
}

// ISR для изменений на порту D (PD0..PD7) — мы используем PD2..PD7
ISR(PCINT2_vect) {
  uint8_t curD = PIND & MASK_PORTD;
  uint8_t curB = PINB & MASK_PORTB;

  // Alpha (PD2, PD4)
  {
    uint8_t curState = ((curD & (1<<PD2)) ? 1 : 0) | ((curD & (1<<PD4)) ? 2 : 0);
    uint8_t idx = (prevAlphaState << 2) | curState;
    alpha_encoder_value += transTable[idx];
    prevAlphaState = curState;
  }

  // Bravo (PD3, PD5)
  {
    uint8_t curState = ((curD & (1<<PD3)) ? 1 : 0) | ((curD & (1<<PD5)) ? 2 : 0);
    uint8_t idx = (prevBravoState << 2) | curState;
    bravo_encoder_value += transTable[idx];
    prevBravoState = curState;
  }

  // Charlie (PD6, PB0)
  {
    uint8_t curState = ((curD & (1<<PD6)) ? 1 : 0) | ((curB & (1<<PB0)) ? 2 : 0);
    uint8_t idx = (prevCharlieState << 2) | curState;
    charlie_encoder_value += transTable[idx];
    prevCharlieState = curState;
  }

  // Delta (PD7, PB1)
  {
    uint8_t curState = ((curD & (1<<PD7)) ? 1 : 0) | ((curB & (1<<PB1)) ? 2 : 0);
    uint8_t idx = (prevDeltaState << 2) | curState;
    delta_encoder_value += transTable[idx];
    prevDeltaState = curState;
  }
}

// ISR для изменений на порту B (PB0..PB7) — мы используем PB0..PB1
ISR(PCINT0_vect) {
  uint8_t curD = PIND & MASK_PORTD;
  uint8_t curB = PINB & MASK_PORTB;

  // Аналогично: пересчитываем все энкодеры — это важно, потому что при изменении PB0/PB1
  // мы хотим корректно обновить Charlie/Delta. Обработка всех энкодеров здесь безопасна.
  {
    uint8_t curState = ((curD & (1<<PD2)) ? 1 : 0) | ((curD & (1<<PD4)) ? 2 : 0);
    uint8_t idx = (prevAlphaState << 2) | curState;
    alpha_encoder_value += transTable[idx];
    prevAlphaState = curState;
  }
  {
    uint8_t curState = ((curD & (1<<PD3)) ? 1 : 0) | ((curD & (1<<PD5)) ? 2 : 0);
    uint8_t idx = (prevBravoState << 2) | curState;
    bravo_encoder_value += transTable[idx];
    prevBravoState = curState;
  }
  {
    uint8_t curState = ((curD & (1<<PD6)) ? 1 : 0) | ((curB & (1<<PB0)) ? 2 : 0);
    uint8_t idx = (prevCharlieState << 2) | curState;
    charlie_encoder_value += transTable[idx];
    prevCharlieState = curState;
  }
  {
    uint8_t curState = ((curD & (1<<PD7)) ? 1 : 0) | ((curB & (1<<PB1)) ? 2 : 0);
    uint8_t idx = (prevDeltaState << 2) | curState;
    delta_encoder_value += transTable[idx];
    prevDeltaState = curState;
  }
}
