#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- CALIBRAÇÃO (Ajusta aqui se necessário) ---
#define SERVOMIN  100 
#define SERVOMAX  500

// Variáveis para guardar os valores de pulso já calculados
int PULSO_ABERTO;  // Valor para 90 graus (Dedos/Cotovelo repouso)
int PULSO_FECHADO; // Valor para 0 graus (Dedos/Cotovelo ativo)

// --- PINOS PCA ---
#define PORTA_COTOVELO 0
#define PORTA_DEDO1    1 
#define PORTA_DEDO2    2 
#define PORTA_DEDO3    3 
#define PORTA_DEDO4    4 
#define PORTA_DEDO5    5
#define PORTA_ROTACAO  6  // NOVA PORTA PARA O SERVO DE ROTAÇÃO

// --- COMMS ---
char g_bufferPC[100];    
bool g_mensagemPronta = false; 
int g_bufferIndex = 0;

void setup() {
  // 1. Velocidade máxima na série que comunica com o RaspberryPi
  Serial2.begin(115200); 
  Serial3.begin(9600); // O Uno mantém-se a 9600

  // 2. Velocidade máxima no I2C (PCA9685)
  pwm.begin();
  pwm.setPWMFreq(50);
  Wire.setClock(400000); // Aumenta de 100kHz para 400kHz

  // 3. PRÉ-CÁLCULO
  // Nota: Ajustar estes maps conforme a mecânica da mão
  PULSO_ABERTO = map(0, 0, 180, SERVOMIN, SERVOMAX);
  PULSO_FECHADO  = map(180, 0, 180, SERVOMIN, SERVOMAX);

  delay(10);
  
  // Posição inicial
  moverTodos(0);
  Serial.println("MEGA PRONTO (MODO TURBO 115200) + ROTACAO");
}

void loop() {
  // Leitura otimizada da Série
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
  // Formato recebido: $orient,flex,d1,d2,d3,d4,d5,rotacao
  
  // 1. Stepper (Uno) -> Primeira Token
  char* token = strtok(g_bufferPC, ",");
  if (token == NULL) return;
  Serial3.write(token[0]); 
  
  // 2. Servos PCA
  // Cotovelo (Digital: 0 ou 1)
  moverDigital(PORTA_COTOVELO); 
  
  // Dedos (Digital: 0 ou 1)
  moverDigital(PORTA_DEDO1);
  moverDigital(PORTA_DEDO2);
  moverDigital(PORTA_DEDO3);
  moverDigital(PORTA_DEDO4);
  moverDigital(PORTA_DEDO5);
  
  // 3. Rotação (Analógico: 0 a 180)
  moverAnalogico(PORTA_ROTACAO); 
}

// Função para servos digitais (0 ou 1)
// Lê a próxima token e move para Aberto ou Fechado
inline void moverDigital(int porta) {
  char* token = strtok(NULL, ",");
  if (token != NULL) {
    // Se "1", usa PULSO_FECHADO. Se "0", PULSO_ABERTO
    int pulso = (token[0] == '1') ? PULSO_FECHADO : PULSO_ABERTO;
    pwm.setPWM(porta, 0, pulso);
  }
}

// Função para servos analógicos (0 a 180)
// Lê a próxima token, converte para int e mapeia para servo
inline void moverAnalogico(int porta) {
  char* token = strtok(NULL, ",");
  if (token != NULL) {
    int angulo = atoi(token); // Converte string "123" para int 123
    // Proteção limites
    if (angulo < 0) angulo = 0;
    if (angulo > 180) angulo = 180;
    
    int pulso = map(angulo, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(porta, 0, pulso);
  }
}

void moverTodos(int ang) {
  int pulso = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  for (int i = 0; i <= 6; i++) { // Agora vai até 6
    pwm.setPWM(i, 0, pulso);
  }
}
