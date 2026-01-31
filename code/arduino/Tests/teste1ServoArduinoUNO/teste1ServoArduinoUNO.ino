#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Inicializa o controlador PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// CONFIGURAÇÕES DO SERVO MG996R
// Dependendo do fabricante do MG996R, estes valores podem variar ligeiramente.
// Se o servo bater no fim de curso e fizer barulho, ajusta estes números.
#define SERVOMIN  150 // Valor de pulso para 0 graus (aprox.)
#define SERVOMAX  600 // Valor de pulso para 180 graus (aprox.)

// Porta onde o servo está ligado (O teu pedido)
#define SERVO_PORTA 0 

void setup() {
  Serial.begin(9600);
  Serial.println("Teste do Servo MG996R na porta 0");

  pwm.begin();
  
  // Os servos analógicos como o MG996R funcionam a ~50 Hz ou 60 Hz
  pwm.setPWMFreq(50); 
  
  delay(10);
  
  Serial.println("aqui");

}

void loop() {
  // Mover para 0 Graus
  Serial.println("Movendo para 0 graus");
  moverServoGraus(SERVO_PORTA, 0);
  delay(1000); // Espera 1 segundo

  // Mover para 90 Graus (Centro)
  Serial.println("Movendo para 90 graus");
  moverServoGraus(SERVO_PORTA, 90);
  delay(1000);

  // Mover para 180 Graus
  Serial.println("Movendo para 180 graus");
  moverServoGraus(SERVO_PORTA, 180);
  delay(1000);
}

// Função auxiliar para converter graus (0-180) em pulsos do PCA9685 (0-4096)
void moverServoGraus(int porta, int graus) {
  // Limita os graus entre 0 e 180 para segurança
  if (graus < 0) graus = 0;
  if (graus > 180) graus = 180;

  // Mapeia o ângulo para o valor de pulso correspondente
  int pulso = map(graus, 0, 180, SERVOMIN, SERVOMAX);
  
  // Envia o comando para o PCA9685
  // setPWM(porta, momento_ligar, momento_desligar)
  pwm.setPWM(porta, 0, pulso);
}