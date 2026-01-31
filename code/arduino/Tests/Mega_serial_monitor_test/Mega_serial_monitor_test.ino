#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- CALIBRAÇÃO ---
#define SERVOMIN  100 
#define SERVOMAX  450

int PULSO_ABERTO; 
int PULSO_FECHADO;

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
  // Inicializamos Serial a 115200 (Asegúrate que el monitor esté igual)
  Serial.begin(115200); 
  Serial3.begin(9600); 

  pwm.begin();
  pwm.setPWMFreq(50);
  Wire.setClock(400000); 

  PULSO_ABERTO = map(0, 0, 180, SERVOMIN, SERVOMAX);
  PULSO_FECHADO  = map(180, 0, 180, SERVOMIN, SERVOMAX);

  delay(10);
  moverTodos(0); // Posición inicial
  
  Serial.println("=========================================");
  Serial.println("   MODO DEBUG ACTIVADO - LISTO");
  Serial.println("   Asegurate de tener 'Nueva Linea' activado");
  Serial.println("   Ejemplo de comando: $0,1,1,1,1,1,0");
  Serial.println("=========================================");
}

void loop() {
  while (Serial.available() > 0 && !g_mensagemPronta) {
    char inChar = Serial.read();
    
    // DEBUG: Ver si llega algo (descomenta si no ves absolutamente nada)
    // Serial.print(inChar); 

    if (inChar == '$') {
      g_bufferIndex = 0;
      Serial.println("\n[DEBUG] Inicio de mensaje detectado ($)");
    } 
    else if (inChar == '\n') {
      g_bufferPC[g_bufferIndex] = '\0';
      g_mensagemPronta = true;
      Serial.print("[DEBUG] Fin de linea detectado. Buffer completo: ");
      Serial.println(g_bufferPC);
    } 
    else if (g_bufferIndex < 99) {
      g_bufferPC[g_bufferIndex] = inChar;
      g_bufferIndex++;
    }
  }

  if (g_mensagemPronta) {
    Serial.println("[DEBUG] Procesando mensaje...");
    processarMensagemRapida();
    g_mensagemPronta = false;
    g_bufferIndex = 0;
    Serial.println("[DEBUG] Esperando siguiente comando...\n");
  }
}

void processarMensagemRapida() {
  // 1. Stepper (Uno)
  // Nota: strtok destruye el buffer original cortándolo en pedazos
  char* token = strtok(g_bufferPC, ",");
  
  if (token == NULL) {
    Serial.println("[ERROR] El mensaje esta vacio o mal formado.");
    return;
  }
  
  Serial.print("   -> Stepper (Serial3): ");
  Serial.println(token[0]);
  Serial3.write(token[0]); 

  // 2. Servos PCA
  Serial.println("   -> Moviendo Servos:");
  moverRapido(PORTA_COTOVELO, "Codo");
  moverRapido(PORTA_DEDO1, "Dedo 1");
  moverRapido(PORTA_DEDO2, "Dedo 2");
  moverRapido(PORTA_DEDO3, "Dedo 3");
  moverRapido(PORTA_DEDO4, "Dedo 4");
  moverRapido(PORTA_DEDO5, "Dedo 5");
}

// He añadido el nombre para el debug
void moverRapido(int porta, String nombreServo) {
  char* token = strtok(NULL, ",");
  if (token != NULL) {
    int pulso = (token[0] == '1') ? PULSO_FECHADO : PULSO_ABERTO;
    
    // DEBUG DETALLADO
    Serial.print("      ");
    Serial.print(nombreServo);
    Serial.print(" [Pin ");
    Serial.print(porta);
    Serial.print("] Valor: ");
    Serial.print(token[0]);
    Serial.print(" -> PWM: ");
    Serial.println(pulso);

    pwm.setPWM(porta, 0, pulso);
  } else {
    Serial.print("      [ERROR] Faltan datos para ");
    Serial.println(nombreServo);
  }
}

void moverTodos(int ang) {
  int pulso = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  for (int i = 0; i <= 5; i++) {
    pwm.setPWM(i, 0, pulso);
  }
}