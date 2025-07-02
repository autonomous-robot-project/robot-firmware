#include <Arduino.h>


void handshake() {
  while (!Serial)
    ;

  bool synced = false;
  while (!synced) {
    if (Serial.available()) {
      String answer = Serial.readStringUntil('\n');
      if (answer == "INIT") {
        Serial.println("READY");
        synced = true;
        break;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  handshake();
  Serial.println("ESP32 ready.");
}

void loop() {
  // Print a message to the standard Serial Monitor (via USB/UART0)
  // Serial.println("heartbeat");

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    Serial.println("Comando recebido: " + cmd);
  }

//   delay(500);
}
