#include <Servo.h>

// --- Pinos para os Servos ---
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

// --- Variáveis para a Comunicação com PC (Teste) ---
char g_bufferPC[100];    // Buffer para guardar a mensagem
bool g_mensagemPronta = false; // Flag que indica se uma mensagem completa chegou
int g_bufferIndex = 0;

void setup() {
  // Inicia Serial para Debug (PC)
  Serial.begin(9600);
  Serial.println("Mega (Mestre) iniciado.");
  Serial.println("Ouvindo PC na 'Serial'...");
  Serial.println("Enviando para Uno na 'Serial2' (Pinos 16, 17)...");

  // Inicia Serial2 para comunicar com o Arduino Uno (Escravo)
  Serial2.begin(9600);
  Serial.println("Serial2 ligada comunicação com Arduino");

  // Anexa os 6 servos aos seus pinos
  servoCotovelo.attach(PIN_SERVO_COTOVELO);
  servoDedo1.attach(PIN_SERVO_DEDO1);
  servoDedo2.attach(PIN_SERVO_DEDO2);
  servoDedo3.attach(PIN_SERVO_DEDO3);
  servoDedo4.attach(PIN_SERVO_DEDO4);
  servoDedo5.attach(PIN_SERVO_DEDO5);
}

void loop() {
  // 1. Verifica constantemente se o PC enviou dados
  ouvirPC();

  // 2. Se uma mensagem completa foi recebida, processa-a
  if (g_mensagemPronta) {
    Serial.print("Mensagem recebida: $");
    Serial.println(g_bufferPC);
    
    processarMensagem();
    
    // Limpa a flag para esperar pela próxima mensagem
    g_mensagemPronta = false;
    g_bufferIndex = 0; // Prepara o buffer para a próxima
  }
}

/**
 * @brief Ouve a porta Serial (PC) e guarda a mensagem no buffer.
 */
void ouvirPC() {
  static bool emMensagem = false; // Flag para saber se estamos a ler uma msg

  while (Serial3.available() > 0 && !g_mensagemPronta) {
    char inChar = Serial3.read();

    if (inChar == '$') {
      g_bufferIndex = 0;
      emMensagem = true;
    } 
    else if (emMensagem) {
      if (inChar == '\n') {
        g_bufferPC[g_bufferIndex] = '\0'; // Termina a string
        g_mensagemPronta = true;
        emMensagem = false;
      } 
      else {
        if (g_bufferIndex < 99) {
          g_bufferPC[g_bufferIndex] = inChar;
          g_bufferIndex++;
        }
      }
    }
  }
}

/**
 * @brief Processa a string completa que está no buffer.
 */
void processarMensagem() {
  
  // 1. Obter o <Estado> (para o Stepper/Uno)
  // strtok divide o buffer 'g_bufferPC' no primeiro delimitador ','
  char* token = strtok(g_bufferPC, ",");
  if (token == NULL) return; // Mensagem mal formatada
  
  // token agora é uma string (char*) que contém "0", "1" ou "2"

  // ******* A GRANDE MUDANÇA ESTÁ AQUI *******
  
  // Enviamos o *primeiro caracter* da string 'token'.
  // Se token = "1", isto envia o caracter '1'
  Serial2.write(token[0]);
  
  // Imprime no monitor do PC o que foi enviado
  Serial.print("  -> Comando para Uno: ");
  Serial.println(token); // Imprime a string "0", "1", ou "2"


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