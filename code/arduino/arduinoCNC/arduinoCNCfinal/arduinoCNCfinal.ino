#include <SoftwareSerial.h>

// --- Pinos do CNC Shield (Eixo X) ---
#define STEP_PIN 2
#define DIR_PIN  5
#define ENABLE_PIN 8

// --- Pinos para Comunicação com o Mega ---
#define SOFT_RX_PIN 10
#define SOFT_TX_PIN 11

// --- Nossos Cálculos ---
// (200 passos * 16 microsteps) / 4 = 800 passos
const int PASSOS_PARA_90_GRAUS = 800;

// Variável para guardar a posição atual
// false = Posição 0
// true  = Posição 1 (90 graus)
bool estaNaPosicao90 = false;

// Configura a porta série de software
SoftwareSerial megaSerial(SOFT_RX_PIN, SOFT_TX_PIN); // RX, TX

void setup() {
  // Configura os pinos do motor
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Ativa os drivers do motor
  digitalWrite(ENABLE_PIN, LOW);

  // Inicia a porta série para o PC (Debug)
  Serial.begin(9600);
  Serial.println("Uno pronto para receber comandos...");

  // Inicia a porta série para o Mega
  megaSerial.begin(9600);
}

void loop() {
  // Verifica se o Mega enviou algum dado
  if (megaSerial.available() > 0) {
    char comando = megaSerial.read();

    // --- LÓGICA DE MOVIMENTO ---

    // Se o comando for '1' (ir para 90°) E não estivermos já lá
    if (comando == '1' && !estaNaPosicao90) {
      Serial.println("Recebido '1'. Movendo para 90 graus...");
      // moveMotor(passos, direcao_para_90_graus)
      moveMotor(PASSOS_PARA_90_GRAUS, HIGH); // Assumindo HIGH como 0->90
      estaNaPosicao90 = true;
      Serial.println("Movimento concluído. Na posição 90.");
    }
    
    // Se o comando for '0' (voltar a 0°) E estivermos em 90°
    else if (comando == '0' && estaNaPosicao90) {
      Serial.println("Recebido '0'. Retornando para 0 graus...");
      // moveMotor(passos, direcao_para_0_graus)
      moveMotor(PASSOS_PARA_90_GRAUS, LOW); // Assumindo LOW como 90->0
      estaNaPosicao90 = false;
      Serial.println("Movimento concluído. Na posição 0.");
    }
  }
}

/**
 * Função para mover o motor
 * @param numPassos - Quantos passos dar
 * @param direcao - O valor para o pino de Direção (HIGH ou LOW)
 */
void moveMotor(int numPassos, bool direcao) {
  digitalWrite(DIR_PIN, direcao);
  
  for (int x = 0; x < numPassos; x++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500); // Velocidade do seu código original
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
}