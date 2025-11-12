#include <Servo.h>

// --- Pinos para os Servos
#define PIN_SERVO_COTOVELO 2
#define PIN_SERVO_DEDO1    3
#define PIN_SERVO_DEDO2    4
#define PIN_SERVO_DEDO3    5
#define PIN_SERVO_DEDO4    6
#define PIN_SERVO_DEDO5    7


Servo servoCotovelo;
Servo servoDedo1; //Indicador
Servo servoDedo2; //Do meio
Servo servoDedo3; //Anular
Servo servoDedo4; //Mindinho
Servo servoDedo5; //Polgar

// --- Variáveis para a Comunicação com RPi ---
char g_bufferRPi[100];    // Buffer para guardar a mensagem
bool g_mensagemPronta = false; // Flag que indica se uma mensagem completa chegou
int g_bufferIndex = 0;

void setup() {
  // Inicia Serial para Debug (PC)
  Serial.begin(9600);
  Serial.println("Mega (Mestre) iniciado.");
  Serial.println("Ouvindo RPi na Serial2...");
  Serial.println("Enviando para Uno na Serial1...");

  // Inicia Serial1 para comunicar com o Arduino Uno (Escravo)
  Serial1.begin(9600);

  // Inicia Serial2 para comunicar com o Raspberry Pi (Mestre do Mega)
  Serial2.begin(9600);

  // Anexa os 6 servos aos seus pinos
  servoCotovelo.attach(PIN_SERVO_COTOVELO);
  servoDedo1.attach(PIN_SERVO_DEDO1);
  servoDedo2.attach(PIN_SERVO_DEDO2);
  servoDedo3.attach(PIN_SERVO_DEDO3);
  servoDedo4.attach(PIN_SERVO_DEDO4);
  servoDedo5.attach(PIN_SERVO_DEDO5);
}

void loop() {
  // 1. Verifica constantemente se o RPi enviou dados
  ouvirRaspberryPi();

  // 2. Se uma mensagem completa foi recebida, processa-a
  if (g_mensagemPronta) {
    Serial.print("Mensagem recebida: $");
    Serial.println(g_bufferRPi);
    
    processarMensagem();
    
    // Limpa a flag para esperar pela próxima mensagem
    g_mensagemPronta = false;
    g_bufferIndex = 0; // Prepara o buffer para a próxima
  }
}

/**
 * @brief Ouve a porta Serial2 (RPi) e guarda a mensagem no buffer.
 * Esta função é o "parser" que procura por '$' e '\n'.
 */
void ouvirRaspberryPi() {
  static bool emMensagem = false; // Flag para saber se estamos a ler uma msg

  while (Serial2.available() > 0 && !g_mensagemPronta) {
    char inChar = Serial2.read();

    if (inChar == '$') {
      // Início de uma nova mensagem, reinicia o buffer
      g_bufferIndex = 0;
      emMensagem = true;
    } 
    else if (emMensagem) {
      if (inChar == '\n') {
        // Fim da mensagem
        g_bufferRPi[g_bufferIndex] = '\0'; // Termina a string
        g_mensagemPronta = true; // Avisa o loop() que a msg está pronta
        emMensagem = false;
      } 
      else {
        // Adiciona o caracter ao buffer
        if (g_bufferIndex < 99) { // Proteção contra overflow
          g_bufferRPi[g_bufferIndex] = inChar;
          g_bufferIndex++;
        }
      }
    }
  }
}

/**
 * @brief Processa a string completa que está no buffer.
 * Separa os valores e envia os comandos para o Uno e para os Servos.
 */
void processarMensagem() {
  // strtok() é uma função C que divide uma string (token = pedaço)
  // O primeiro argumento é a string, os seguintes são cópias (NULL)
  
  // 1. Obter o <Base_Ang> (para o Stepper/Uno)
  char* token = strtok(g_bufferRPi, ",");
  if (token == NULL) return; // Mensagem mal formatada
  
  int baseAng = atoi(token); // Converte "90" (texto) para 90 (número)

  // Envia o comando para o Uno via Serial1
  // Baseado na nossa conversa anterior: '1' = 90 graus, '0' = 0 graus
  if (baseAng >= 90) {
    Serial1.write('1');
    Serial.println("  -> Comando para Uno: '1' (90 graus)");
  } else {
    Serial1.write('0');
    Serial.println("  -> Comando para Uno: '0' (0 graus)");
  }

  // 2. Obter os 6 ângulos restantes (para os Servos)
  
  // <Cotovelo_Ang>
  token = strtok(NULL, ",");
  if (token == NULL) return;
  servoCotovelo.write(atoi(token));

  // <Dedo1_Ang>
  token = strtok(NULL, ",");
  if (token == NULL) return;
  servoDedo1.write(atoi(token));

  // <Dedo2_Ang>
  token = strtok(NULL, ",");
  if (token == NULL) return;
  servoDedo2.write(atoi(token));

  // <Dedo3_Ang>
  token = strtok(NULL, ",");
  if (token == NULL) return;
  servoDedo3.write(atoi(token));

  // <Dedo4_Ang>
  token = strtok(NULL, ",");
  if (token == NULL) return;
  servoDedo4.write(atoi(token));

  // <Dedo5_Ang>
  token = strtok(NULL, ",");
  if (token == NULL) return;
  servoDedo5.write(atoi(token));
  
  Serial.println("  -> Comandos de Servos enviados.");
}