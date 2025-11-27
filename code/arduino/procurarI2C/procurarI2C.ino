#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);
  Serial.println("\n--- I2C Scanner (Mega) ---");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("A procurar...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("ENCONTRADO no endereco: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Erro desconhecido em 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("Nenhum dispositivo I2C encontrado\n");
  else
    Serial.println("--- Fim da leitura ---\n");

  delay(5000);           
}