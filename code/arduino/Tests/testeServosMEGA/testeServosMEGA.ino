#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 
#define SERVOMAX  600 
#define SERVO_PORTA 0 

void setup() {
  Serial.begin(9600);
  Serial.println("--- TESTE LENTO SERVO 0 ---");

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(100);
}

void loop() {
  Serial.println("A tentar mover...");
  
  // Varredura lenta de 0 a 180
  for (int pulso = SERVOMIN; pulso < SERVOMAX; pulso += 5) {
    pwm.setPWM(SERVO_PORTA, 0, pulso);
    delay(20); // Movimento suave
  }
  
  delay(1000);

  // Varredura lenta de 180 a 0
  for (int pulso = SERVOMAX; pulso > SERVOMIN; pulso -= 5) {
    pwm.setPWM(SERVO_PORTA, 0, pulso);
    delay(20);
  }
  
  delay(1000);
}