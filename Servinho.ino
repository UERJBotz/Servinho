// Inclusão das bibliotecas
//#include <MPU6050_tockn.h> // Giroscópio e Acelerômetro
//#include <Wire.h> // Necessária para o Giroscópio e Acelerômetro
#include <SoftwareSerial.h> // Para o módulo Bluetooth

#include "encoder.h"

// Declaração de objetos
//MPU6050 mpu6050(Wire);
SoftwareSerial BT(6, 9);

// Definição dos pinos dos motores
#define motorEsq1 12
#define motorEsq2 13
#define motorEsqPWM 10
#define motorDir1 8
#define motorDir2 7
#define motorDirPWM 11

// Definição dos pinos dos sensores
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define SCURVA A5

#define ADC_MAX 1023
#define THRESHOLD 50

// Variáveis para sensores e PID
#define SENSOR_COUNT (sizeof(sensores)/sizeof(*sensores))

int   sensores[]           = { S1, S2, S3, S4, S5}; // Vetor para sensores
float sensor_para_angulo[] = {-25, -13, 0, 13, 25};
bool  sensor_dig[SENSOR_COUNT] /*inicializado no loop*/;
bool  running = false;
int vel_base = 240;

bool last_signal = HIGH;
int  curve_number = 0;
const int stop_count = 22;
const unsigned long debounce_delay = 50;
unsigned long last_debounce_time   = 0;
bool show_vel = 0;
unsigned long tempo_antes = 0;
float soma_vel = 0;
float contM = 0;
float vel_media = 0;
float velM = 0;

// Programas principais
void setup() {
    Serial.begin(9600);
    BT.begin(9600);
    BT.println("Espere alguns segundos...");
    //Wire.begin();
    setup_encoder();

    for (int i = 0; i < 5; i++) {
        pinMode(sensores[i], INPUT);
    }
    pinMode(SCURVA, INPUT);

    pinMode(motorEsq1, OUTPUT);
    pinMode(motorEsq2, OUTPUT);
    pinMode(motorDir1, OUTPUT);
    pinMode(motorDir2, OUTPUT);
    pinMode(motorDirPWM, OUTPUT);
    pinMode(motorEsqPWM, OUTPUT);

    BT.println("### MENU ###");
    BT.println("'kp, ki ou kd' para mostrar valores atuais do PID");
    BT.println("'kp, Ki ou kd' seguido de um valor x altera valores.");
    BT.println(" 'start' para acionar os motores.");
    BT.println(" 'stop' para parar os motores.");
    BT.println(" 'line.read' indica quais sensores estão ativos naquele momento.");
    BT.println("SEGUIDOR PRONTO!");
}

bool debounce_count(bool novo, bool velho, unsigned long delay = debounce_delay) {
    if ((millis() - last_debounce_time) >= delay) return novo;
    else                                          return velho;
}
bool parar() { // fazer: dividir em duas funções: passou_por_marcador() e parar()
    int16_t analog = analogRead(SCURVA);
    bool    signal = debounce_count(analog > THRESHOLD, last_signal);  // no pino analógico: valores menores -> mais claro

    if (signal != last_signal) last_debounce_time = millis();

    if (last_signal && !signal) curve_number++;
    last_signal = signal;

    if (curve_number >= stop_count) return true;
    else                            return false;
}

void loop() {
    if (parar()) {
        BT.println("Parado");
        //mover(240,220);
        delay(400);
        stop();
    } else {
        //BT.println(curve_number);
    }

    unsigned long dt = (millis() - tempo_antes);
    if ((dt >= 100) && (show_vel)) {
        float left, right;
        get_wheel_speeds(dt, &left, &right);
        velM = ((left + right)/2 );
        soma_vel += velM;
        contM ++;
        BT.print("Vel: ");
        BT.print(velM);
        BT.println(" m/s");
        Serial.println((velM));

        tempo_antes = millis();
    }

    if (running) mover_motores();

    if (BT.available() > 0){
        String CMD = BT.readString(); // Read the command
        Serial.println("command: " + CMD);
        BT.println("command: " + CMD);
        String resposta = terminal(CMD.c_str());
        Serial.println(resposta);
        BT.println(resposta);
    }

}


// Leitura do Angulo e dos sensores ativos
float read_angle() {
    int qtd_sens_ativos = 0, soma = 0;
    for (size_t i = 0; i<SENSOR_COUNT; i++) {
        sensor_dig[i] = digitalRead(sensores[i]);
        if (sensor_dig[i]) {
            soma += sensor_para_angulo[i];
            qtd_sens_ativos++;
        }
    }

    return (float)soma/(float)(qtd_sens_ativos ? qtd_sens_ativos : 1);
}

// Controlador PID
class control_pid {
  public:
    double kp = 0;
    double ki = 0;
    double kd = 0;
    uint32_t last_ms = 0;
    float last_erro = 0;
    float P = 0;
    float I = 0;
    float D = 0;
    float I_MAX = 1000;

  public:
    void init() {
        I = 0;
    }
    float loop(float erro) {
        uint32_t ms = millis();
        uint32_t dt = ms - last_ms; dt = dt ? : 0;
        P = erro;
        D = 1000.0*( erro - last_erro )/(float)dt;
        I = constrain( I+0.001*erro*dt, -I_MAX, I_MAX );
        last_erro = erro;
        last_ms = ms;
        return kp*P + I*ki + kd*D;
    }
};

control_pid pid_ang;
control_pid pid_motor;


// Função para parar os motores
void stop() {
    mover(0, 0);
    running = false;
}

void start(){
    pid_ang.init();
    pid_motor.kp = 4.65;
    pid_motor.kd = 0.4;
    pid_motor.ki = 0.35;
    pid_motor.init();
    running = true;
}

//Terminal via Bluetooth
String terminal(const char *const cmd) {
    //Tratamento dos dados
    String input = String(cmd);
    input.trim(); 
    input.replace(',', '.'); 

    Serial.println("Raw command: " + input);

    int spaceIndex = input.indexOf(' ');
    String key;
    double x = 0;
    bool hasValue = (spaceIndex != -1); 
    if (hasValue) {
        key = input.substring(0, spaceIndex); 
        String valueStr = input.substring(spaceIndex + 1); 
        x = valueStr.toDouble(); 
    } else {
        key = input;
    }

    String resposta = "";

    //Comandos
    if (key == "kp") {
        if (hasValue) {
            pid_ang.kp = x; // Se um valor de kp é dado
        }
        resposta = "KP: " + String(pid_ang.kp, 5) + "\n";
    } else if (key == "ki") {
        if (hasValue) {
            pid_ang.ki = x; // Se um valor de ki é dado
        }
        resposta = "KI: " + String(pid_ang.ki, 5) + "\n";
    } else if (key == "kd") {
        if (hasValue) {
            pid_ang.kd = x; 
        }
        resposta = "KD: " + String(pid_ang.kd, 5) + "\n";
    } else if (key == "vel") {
    if (hasValue) {
        vel_base = x; }
        resposta = "Vel: " + String(vel_base) + "  (Max: 255 pwm)" + "\n";
    }else if (key == "show_vel") {
        show_vel = !show_vel;
    } else if (key == "start") {
        start();
        show_vel = 1;
        curve_number = 0;
        soma_vel = 0;
        contM = 0;
        resposta = "System started\n";
    } else if (key == "stop") {
        stop(); 
        show_vel = 0;
        vel_media = soma_vel/contM;
        resposta = "Velocidade Média: " + String(vel_media) + " System stopped\n";
    } else if (key == "line.read") {
        float angle = read_angle(); 
        resposta = "line: " + String(angle) + " Graus\n";
        resposta += str_array("", sensor_dig, SENSOR_COUNT) + "\n"; 
    } else if (key == "reset") {
        curve_number = 0;
    } else {
        resposta = "Unknown command: " + key + "\n"; 
    }
    Serial.println("Response: " + resposta); 
    return resposta;
}

template<typename T> String str_array(const char *nome, T* arr, size_t sz){
    String str = String(nome) + " = [ ";
    for (size_t i=0; i<sz-1; i++) str += (String(arr[i]) + " ");
    str += String(arr[sz-1]) + " ]";
    return str;
}

template<typename T> void print_array(String prefix, T* arr, size_t sz){
    Serial.print(prefix);
    Serial.print("[");
    for (size_t i=0; i<sz-1; i++){
        Serial.print(arr[i]);
    }
    Serial.print(arr[sz-1]);
    Serial.print("]\n");
}

// Função para controlar os motores
void mover_motores() {
    float rot = 0;
    float vel = 0.3;
    float angle = read_angle();
    float erro  = 0 - angle;
    float erroVel = vel - velM;
    float dif   = pid_ang.loop(erro);
    float difVel = pid_motor.loop(erroVel);

    /*
    float wl = vel/r + (rot*l)/r;
    float wr = vel/r - (rot*l)/r;
    */

    int velocidade_esq = (vel_base - erroVel) - dif;
    int velocidade_dir = (vel_base + erroVel) + dif;


    // Constranger as velocidades para não ultrapassar os limites
    velocidade_esq = constrain(velocidade_esq, -255, 255);
    velocidade_dir = constrain(velocidade_dir, -255, 255);
    mover(velocidade_esq, velocidade_dir);

    delay(10);

    //BT.println(velocidade_esq);

}

void mover(int16_t vel_esq, int16_t vel_dir) {
    motor3pin(motorEsq1, motorEsq2, motorEsqPWM, vel_esq);
    motor3pin(motorDir1, motorDir2, motorDirPWM, vel_dir);
}

void motor(uint8_t p1, uint8_t p2, int16_t vel) {
    if (vel < 0) {
        analogWrite(p1, 0);
        analogWrite(p2, vel);
    } else {
        analogWrite(p1, abs(vel));
        analogWrite(p2, 0);
    }
}

void motor3pin(uint8_t p1, uint8_t p2, uint8_t pwm, int16_t vel) {
    if (vel < 0) {
        digitalWrite(p1, LOW);
        digitalWrite(p2, HIGH);

        analogWrite(pwm, vel);
    } else {
        digitalWrite(p1, HIGH);
        digitalWrite(p2, LOW);

        analogWrite(pwm, abs(vel));
    }

}
