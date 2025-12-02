#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- CALIBRAÇÃO (Ajusta aqui se necessário) ---
#define SERVOMIN  150 
#define SERVOMAX  530

// Variáveis para guardar os valores de pulso já calculados
// Isto evita fazer contas matemáticas durante o movimento
int PULSO_ABERTO;  // Valor para 90 graus
int PULSO_FECHADO; // Valor para 0 graus

// --- PINOS PCA ---
#define PORTA_COTOVELO 0
#define PORTA_DEDO1    1 
#define PORTA_DEDO2    2 
#define PORTA_DEDO3    3 
#define PORTA_DEDO4    4 
#define PORTA_DEDO5    5 

// --- COMMS ---
char g_bufferPC[100];    
bool g_mensagemPronta = false; 
int g_bufferIndex = 0;

void setup() {
  // 1. Velocidade máxima na serial que comunica com o RaspberryPi
  Serial2.begin(115200); 
  Serial3.begin(9600); // O Uno mantém-se a 9600

  // 2. Velocidade máxima no I2C (PCA9685)
  pwm.begin();
  pwm.setPWMFreq(50);
  Wire.setClock(400000); // Aumenta de 100kHz para 400kHz (Muito mais rápido!)

  // 3. PRÉ-CÁLCULO (Matemática feita apenas uma vez)
  PULSO_ABERTO = map(0, 0, 180, SERVOMIN, SERVOMAX);
  PULSO_FECHADO  = map(180, 0, 180, SERVOMIN, SERVOMAX);

  delay(10);
  
  // Posição inicial rápida
  moverTodos(0);
  Serial.println("MEGA PRONTO (MODO TURBO 115200)");
}

void loop() {
  // Leitura otimizada da Serial
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

void processarMensagemRapida() {
  // 1. Stepper (Uno)
  char* token = strtok(g_bufferPC, ",");
  if (token == NULL) return;
  Serial3.write(token[0]); 

  // 2. Servos PCA - Execução direta sem prints
  // Chamamos a função diretamente para evitar overhead
  moverRapido(PORTA_COTOVELO);
  moverRapido(PORTA_DEDO1);
  moverRapido(PORTA_DEDO2);
  moverRapido(PORTA_DEDO3);
  moverRapido(PORTA_DEDO4);
  moverRapido(PORTA_DEDO5);
}

// Função Otimizada: Usa valores pré-calculados e remove IFs desnecessários
inline void moverRapido(int porta) {
  char* token = strtok(NULL, ",");
  if (token != NULL) {
    // Se o caracter for '1', usa PULSO_ABERTO, senão usa PULSO_FECHADO
    // Esta operação ternária é mais rápida que if/else tradicional
    int pulso = (token[0] == '1') ? PULSO_FECHADO : PULSO_ABERTO;
    
    pwm.setPWM(porta, 0, pulso);
  }
}

void moverTodos(int ang) {
  int pulso = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  for (int i = 0; i <= 5; i++) {
    pwm.setPWM(i, 0, pulso);
  }
}