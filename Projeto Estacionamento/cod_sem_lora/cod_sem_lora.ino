#include "Arduino.h"
#include "heltec.h"

int pinSensor = 21; //PINO DIGITAL UTILIZADO PELO SENSOR

void setup () {
   //modo do pino do sensor, input pois ele tem um tratamento interno
  pinMode(pinSensor, INPUT);
  Serial.begin(9600);
}

void loop() {

  bool sensor = digitalRead(pinSensor);

  if (!sensor) {
   Serial.println("Veiculo detectado");

  } else {
    Serial.println("Veiculo não detectado");
  }

  delay(1000);//delay 1 segundo
}
