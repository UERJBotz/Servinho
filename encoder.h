// encoder.h - Complete single-file solution for Arduino IDE
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// ================= CONFIGURATION =================
#define PPR 12             // Pulses per revolution (6 teeth × 2 edges)
#define WHEEL_DIAMETER_MM 50  // Wheel diameter in millimeters
#define WHEEL_CIRCUM_MM (WHEEL_DIAMETER_MM * PI)
#define DEBOUNCE_US 200        // 50μs debounce window

// ================= ENCODER STATE =================
typedef struct {
  volatile long count;
  volatile bool dir;
  volatile uint32_t last_time;
  uint8_t pinA, pinB;
} Encoder;

Encoder left = {0, true, 0, 2, 4};
Encoder right = {0, true, 0, 3, 5};

// ================= FORWARD DECLARATIONS =================
void encoder_isr(Encoder* e);
void left_isr();
void right_isr();

// ================= INTERRUPT SERVICE ROUTINES =================
void left_isr() { encoder_isr(&left); }
void right_isr() { encoder_isr(&right); }

void encoder_isr(Encoder* e) {
  uint32_t now = micros();
  if (now - e->last_time < DEBOUNCE_US) return;

  uint8_t new_state = (digitalRead(e->pinB) << 1) | digitalRead(e->pinA);
  static uint8_t old_state[2] = {0,0}; // Separate for each encoder

  if (new_state != old_state[e == &right]) {
    if (old_state[e == &right] == 0) {
      // Standard decoding for left, reversed for right
      if (new_state == 2) e->dir = (e == &left);
      if (new_state == 1) e->dir = !(e == &left);
    }
    e->count += e->dir ? 1 : -1;
    old_state[e == &right] = new_state;
  }
  e->last_time = now;
}

// ================= PUBLIC FUNCTIONS =================
void setup_encoder() {
  // Left encoder (pins 2,3)
  pinMode(left.pinA, INPUT_PULLUP);
  pinMode(left.pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(left.pinA), left_isr, CHANGE);

  // Right encoder (pins 18,19)
  pinMode(right.pinA, INPUT_PULLUP);
  pinMode(right.pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(right.pinA), right_isr, CHANGE);
}

void get_wheel_speeds(unsigned long dt_ms, float* left_speed, float* right_speed) {
  if (dt_ms == 0) {
    *left_speed = *right_speed = 0;
    return;
  }

  // Atomic read and reset of counters
  noInterrupts();
  long left_counts = left.count; left.count = 0;
  long right_counts = right.count; right.count = 0;
  interrupts();

  // Calculate speeds in m/s
  *left_speed = ((float)left_counts / PPR) * (WHEEL_CIRCUM_MM / dt_ms);
  *right_speed = ((float)right_counts / PPR) * (WHEEL_CIRCUM_MM / dt_ms);

}

#endif



/*
const int PINO_CH2 = 2;
const int PINO_CH1 = 3;
const int PINO_CH3 = 19;
const int PINO_CH4 = 18;

int estado,  ultimo_estado;
int estado2, ultimo_estado2;
bool sentido;
bool sentido2;
float vel_e;
float vel_d;

unsigned long contador1;
unsigned long contador2;
unsigned long contador3;
unsigned long contador4;
const int NUMERO_CONTADORES = 2;
const int NUMERO_LEITURAS   = 4;
const int NUMERO_DENTES     = 6;

const double diam = 50;
const double circ = diam*PI;

template <unsigned long& contador>
void conta_pulso1(){

    static unsigned long last_time = 0;
    unsigned long now = micro();
    if (now - last_time >= 500) {
        contador++;
        last_time = now;
    }

}

template <int PINO_CH1, int PINO_CH2,
          unsigned long& contador, int& ultimo_estado, bool& sentido>
void conta_pulso2(void){
  contador++;

  //Verifica o sentido de rotacao do motor
  bool estado = digitalRead(PINO_CH2);
  if (ultimo_estado == LOW && estado == HIGH) {
    sentido = digitalRead(PINO_CH1);
  }
  ultimo_estado = estado;
}


void setup_encoder() {
  //Configuracao dos pinos conectados aos canais do encoder como entrada
  pinMode(PINO_CH2, INPUT);
  pinMode(PINO_CH1, INPUT);
  pinMode(PINO_CH4, INPUT);
  pinMode(PINO_CH3, INPUT);

  //Inicializa as interrupcoes com os pinos configurados para chamar as funcoes  
  attachInterrupt(digitalPinToInterrupt(PINO_CH1), conta_pulso1<contador1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_CH2), conta_pulso2<PINO_CH1, PINO_CH2, contador2, ultimo_estado,  sentido>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_CH3), conta_pulso1<contador3>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_CH4), conta_pulso2<PINO_CH3, PINO_CH4, contador4, ultimo_estado2, sentido2>, CHANGE);
}

float vel_atual(unsigned long dt, unsigned long* contador1, unsigned long* contador2, bool sentido) {
    //Calcula a velocidade e exibe no monitor
    unsigned long media = (*contador1 + *contador2);
    float pct = (float)media / (float)(NUMERO_DENTES*NUMERO_LEITURAS); //Calcula a % de um giro completo de acordo com o numero de dentes do disco
    float frequencia = pct/(dt/ 1000.0f); // r/ms
    float velocidade = (frequencia * circ)/1000.0f; // mm/ms = m/s

    //Zera os contadores e reinicia a contagem de tempo.
    *contador1 = 0;
    *contador2 = 0;
    return sentido ? velocidade : -velocidade;
}
*/



/*
// encoder.h - Uno-compatible (x2 decoding on channel A)

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// ================= CONFIGURATION =================
// For 6 teeth and x2 decoding (edges only on channel A), you get 12 counts/rev.
#define PPR 12

#define WHEEL_DIAMETER_MM 50
#define WHEEL_CIRCUM_MM (WHEEL_DIAMETER_MM * PI)

// Debounce window for ISR (microseconds)
#define DEBOUNCE_US 200

// ================= ENCODER STATE =================
typedef struct {
  volatile long count;
  volatile bool dir;
  volatile uint32_t last_time;
  uint8_t pinA, pinB;
} Encoder;

// On Uno, only pins 2 and 3 have external interrupts.
// Route each encoder's channel A to 2 and 3 respectively.
Encoder left  = {0, true, 0, 2, 4}; // A=2 (INT0),  B=4 (any digital)
Encoder right = {0, true, 0, 3, 5}; // A=3 (INT1),  B=5 (any digital)

// ================= FORWARD DECLARATIONS =================
void encoder_isr(Encoder* e);
void left_isr();
void right_isr();

// ================= INTERRUPT SERVICE ROUTINES =================
void left_isr()  { encoder_isr(&left); }
void right_isr() { encoder_isr(&right); }

// Simple x2 decoding: trigger on channel A, read channel B for direction
void encoder_isr(Encoder* e) {
  uint32_t now = micros();
  if ((uint32_t)(now - e->last_time) < DEBOUNCE_US) return;

  // Direction: if A changed, B state determines direction
  bool a = digitalRead(e->pinA);
  bool b = digitalRead(e->pinB);

  // For quadrature: when A changes, dir = (A XOR B)
  // Choose polarity so that forward is positive for left and right as needed.
  bool dir = (a ^ b);

  // If your wiring causes reversed sign on one wheel, invert here:
  // Example: make left positive when dir==true, right positive when dir==true.
  e->dir = dir;

  e->count += e->dir ? 1 : -1;
  e->last_time = now;
}

// ================= PUBLIC FUNCTIONS =================
void setup_encoder() {
  // Left encoder
  pinMode(left.pinA, INPUT_PULLUP);
  pinMode(left.pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(left.pinA), left_isr, CHANGE);

  // Right encoder
  pinMode(right.pinA, INPUT_PULLUP);
  pinMode(right.pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(right.pinA), right_isr, CHANGE);
}

// dt_ms: elapsed time since last call in milliseconds
// Outputs: speeds in m/s (mm/ms numerically equals m/s)
void get_wheel_speeds(unsigned long dt_ms, float* left_speed, float* right_speed) {
  if (dt_ms == 0) { *left_speed = *right_speed = 0; return; }

  // Atomic read and reset of counters
  noInterrupts();
  long left_counts  = left.count;  left.count  = 0;
  long right_counts = right.count; right.count = 0;
  interrupts();

  // counts -> revolutions -> distance per dt
  // Using mm and ms: mm/ms == m/s numerically
  *left_speed  = ((float)left_counts  / PPR) * (WHEEL_CIRCUM_MM / dt_ms);
  *right_speed = ((float)right_counts / PPR) * (WHEEL_CIRCUM_MM / dt_ms);
}

#endif
*/
