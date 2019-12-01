// Initial draft of keyboard controlled ID

// Keypad: Connect SCL on 8, SDO on 9
// IR Led: Connect "S" to PWM Pin 3, ground via 100-300 Ohms

// TODO: NEC sends 0xFFFFFF for repeat (not the same command) so detect key held down, then switch to sending FFFFFF
// TODO: sleep and interrupt to conserve power?

#include <IRremote.h>
#include "gva.h" // my home made map of GVA (Good Guys cheap own-brand)

IRsend irsend;

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(100);
  static int lastKey = -1;

  int key = 0;
  long command;

  // Read keypad
  key = XC4602_read(8, 9);
  if (key) {
    Serial.print("Key ");
    Serial.println(key);

    Serial.print("LastKey ");
    Serial.println(lastKey);
    if (key == lastKey) {
      // NEC sends 0xFFFFFF for repeat (not the same command) so detect key held down, then switch to sending FFFFFF
      command = 0xFFFFFF;
    }
    else {
      // Map the key to a command
      command = mapKeyToCommand(key);
      lastKey = key;
    }
    // Send the command e.g. FE6897 = Mute, 32 bits for NEC
    // IRsend::sendNEC (unsigned long data,  int nbits)
    irsend.sendNEC(command, 32);
    
    Serial.print("Sent ");
    Serial.println(command, HEX);
  }
  else {
    // reset if no key after timeout, to allow for slow repeat.
    lastKey = -1;
  }
}

// ---- lib ----

// returns key number or 0 if no key pressed
int XC4602_read(int sclpin, int sdopin) {
  int key = 0;                           //default to no keys pressed
  pinMode(sclpin, OUTPUT);
  digitalWrite(sclpin, HIGH);
  pinMode(sdopin, INPUT);
  delay(2);                               //ensure data is reset to first key
  for (int i = 1; i < 17; i++) {
    digitalWrite(sclpin, LOW);            //toggle clock
    digitalWrite(sclpin, HIGH);
    if (!digitalRead(sdopin)) {
      key = i; //valid data found
    }
  }
  return key;
}

//struct {
//  long command;
//  int keyCode;
//} key;

long keyMap[16] = {
  GVA_32TDC15_POWER,   // 1
  GVA_32TDC15_MUTE,    // 2
  GVA_32TDC15_UP,      // 3
  GVA_32TDC15_DOWN,    // 4
  GVA_32TDC15_RIGHT,   // 5
  GVA_32TDC15_LEFT,    // 6
  GVA_32TDC15_VOL_DN,  // 7
  GVA_32TDC15_VOL_UP,  // 8
  GVA_32TDC15_EJECT,   // 9
  // GVA_32TDC15_PLAY_PAUSE, code not known yet!
  GVA_32TDC15_GRN_STEP, // 10 alt-pause
  GVA_32TDC15_ENTER, // 11 alt-play after alt-pause
  GVA_32TDC15_P_MODE,  // 12
  GVA_32TDC15_S_MODE,  // 13
  GVA_32TDC15_SOURCE,  // 14
  0xFEF00F, //= left
  0xFEF00F, //= left
};

long mapKeyToCommand(int key) {
  long command;

  // command = 0xFE6897;

  command = keyMap[key - 1];

  return command;
}
