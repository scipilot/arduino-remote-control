// Demo Infra Red Transmit
// Sends a static byte, on receiving serial comms (from serial monitor?)

#include <IRremote.h>

IRsend irsend;

void setup() {
  Serial.begin(9600);
}

void loop() {

  if (Serial.read() != -1) {
    Serial.println("command!");
    //    for (int i = 0; i < 3; i++) {
    //irsend.sendSony(0xa90, 12); // Sony TV power code

    // FE6897 = Mute
    // IRsend::sendNEC (unsigned long data,  int nbits)
    irsend.sendNEC(0xFE6897, 32);
    Serial.println("Sent");
    delay(500);
    //    }
  }
}
