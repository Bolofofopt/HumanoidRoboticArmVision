#include <SoftwareSerial.h>

// --- Pinos do CNC Shield (Eixo X) ---
#define STEP_PIN 2
#define DIR_PIN  5
#define ENABLE_PIN 8

// --- Pinos para Comunicação com o Mega ---
#define SOFT_RX_PIN 10
#define SOFT_TX_PIN 11

// --- Nossos Cálculos ---
const int PASSOS_PARA_90_GRAUS = 800;
const int PASSOS_PARA_180_GRAUS = 1600; // 800 * 2

// Variável para guardar a posição atual
// 0 = Posição 0 (0 graus)
// 1 = Posição 1 (+90 graus)
// 2 = Posição 2 (-90 graus)
int posicaoAtual = 0; 

// Configura a porta série de software
//SoftwareSerial megaSerial(SOFT_RX_PIN, SOFT_TX_PIN); // RX, TX

void setup() {
  // Configura os pinos do motor
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Ativa os drivers do motor
  digitalWrite(ENABLE_PIN, LOW);

  // Inicia a porta série para o PC (Debug)
  Serial.begin(9600);
  Serial.println("Uno escravo pronto (v2). Posição atual: 0");

  // Inicia a porta série para o Mega
  //megaSerial.begin(9600);
}

void loop() {
  // Verifica se o Mega enviou algum dado
  if (Serial.available() > 0) {
    char comando = Serial.read();

    // --- LÓGICA DE MOVIMENTO (Máquina de Estados) ---

    // ----- IR PARA POSIÇÃO 0 (Comando '0') -----
    if (comando == '0' && posicaoAtual != 0) {
      Serial.print("Recebido '0'. Indo para 0 graus...");
      if (posicaoAtual == 1) { // Estava em +90
        moveMotor(PASSOS_PARA_90_GRAUS, LOW); // Gira -90 graus
        Serial.println(" (vindo de +90)");
      } 
      else if (posicaoAtual == 2) { // Estava em -90
        moveMotor(PASSOS_PARA_90_GRAUS, HIGH); // Gira +90 graus
        Serial.println(" (vindo de -90)");
      }
      posicaoAtual = 0;
    }
    
    // ----- IR PARA POSIÇÃO 1 (Comando '1', +90 graus) -----
    else if (comando == '1' && posicaoAtual != 1) {
      Serial.print("Recebido '1'. Indo para +90 graus...");
      if (posicaoAtual == 0) { // Estava em 0
        moveMotor(PASSOS_PARA_90_GRAUS, HIGH); // Gira +90 graus
        Serial.println(" (vindo de 0)");
      } 
      else if (posicaoAtual == 2) { // Estava em -90
        moveMotor(PASSOS_PARA_180_GRAUS, HIGH); // Gira +180 graus
        Serial.println(" (vindo de -90)");
      }
      posicaoAtual = 1;
    }

    // ----- IR PARA POSIÇÃO 2 (Comando '2', -90 graus) -----
    else if (comando == '2' && posicaoAtual != 2) {
      Serial.print("Recebido '2'. Indo para -90 graus...");
      if (posicaoAtual == 0) { // Estava em 0
        moveMotor(PASSOS_PARA_90_GRAUS, LOW); // Gira -90 graus
        Serial.println(" (vindo de 0)");
      } 
      else if (posicaoAtual == 1) { // Estava em +90
        moveMotor(PASSOS_PARA_180_GRAUS, LOW); // Gira -180 graus
        Serial.println(" (vindo de +90)");
      }
      posicaoAtual = 2;
    }
  }
}

/**
 * Função para mover o motor
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