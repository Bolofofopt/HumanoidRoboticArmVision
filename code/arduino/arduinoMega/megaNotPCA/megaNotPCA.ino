#include <Wire.h>
#include <Servo.h>

const int PINOS_SERVOS[] = {2, 3, 4, 5, 6, 7};
#define ID_COTOVELO 0
#define ID_DEDO1    1 
#define ID_DEDO2    2 
#define ID_DEDO3    3 
#define ID_DEDO4    4 
#define ID_DEDO5    5

Servo servos[6];

const int ANGULO_ABERTO = 180;  // Equivalente ao seu antigo pulso de 90 graus
const int ANGULO_FECHADO = 0;  // Equivalente ao seu antigo pulso de 0 graus

// --- COMMS ---
char g_bufferPC[100];    
bool g_mensagemPronta = false; 
int g_bufferIndex = 0;


void processarMensagemRapida() {
  // 1. Stepper (Uno)
  char* token = strtok(g_bufferPC, ",");
  if (token == NULL) return;
  Serial3.write(token[0]); 

  moverRapido(ID_COTOVELO);
  moverRapido(ID_DEDO1);
  moverRapido(ID_DEDO2);
  moverRapido(ID_DEDO3);
  moverRapido(ID_DEDO4);
  moverRapido(ID_DEDO5);
}

inline void moverRapido(int idServo) {
  char* token = strtok(NULL, ",");
  if (token != NULL) {
    // Se '1' vai para 90 graus, se não '0' vai para 0 graus
    int angulo = (token[0] == '1') ? ANGULO_ABERTO : ANGULO_FECHADO;
    
    // Comando direto para a biblioteca Servo
    servos[idServo].write(angulo);
  }
}

void moverTodos(int ang) {
  for (int i = 0; i < 6; i++) {
    servos[i].write(ang);
  }
}


// Ajuste de calibração baseado no feedback (0 graus estava dando 10 graus)
// Reduzimos o tempo do pulso para corrigir o offset e evitar o "ticking" no final
#define PULSO_MIN_US 400   // Antes era ~732 (muito alto)
#define PULSO_MAX_US 2300  // Antes era ~2588 (muito alto)

void setup() {
  Serial.begin(115200); // Debug via USB
  Serial2.begin(115200); // Velocidade máxima na serial que comunica com o RaspberryPi
  Serial3.begin(9600); // O Uno mantém-se a 9600

  for(int i = 0; i < 6; i++) {
    servos[i].attach(PINOS_SERVOS[i], PULSO_MIN_US, PULSO_MAX_US);
  }
  delay(10);
  
  // Posição inicial rápida
  moverTodos(0);
  Serial.println("MEGA PRONTO (MODO TURBO 115200)");
}

void loop() {
  while (Serial2.available() > 0 && !g_mensagemPronta) {
    char inChar = Serial2.read();
    if (inChar == '$') {
      g_bufferIndex = 0;
    } 
    else if (inChar == '\n') {
      g_bufferPC[g_bufferIndex] = '\0';
      g_mensagemPronta = true;
    } 
    else if (g_bufferIndex < 99) {
      g_bufferPC[g_bufferIndex] = inChar;
      g_bufferIndex++;
    }
  }

  if (g_mensagemPronta) {
    processarMensagemRapida();
    g_mensagemPronta = false;
    g_bufferIndex = 0;
  }
}

