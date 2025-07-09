#include "uart_comm.h"

void handshake()
{
  while (!Serial)
    ;

  bool synced = false;
  while (!synced)
  {
    if (Serial.available())
    {
      String answer = Serial.readStringUntil('\n');
      if (answer == "INIT")
      {
        Serial.println("READY");
        synced = true;
        break;
      }
    }
  }
}
