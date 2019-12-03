// Initial draft of keyboard controlled ID
// This does it in a loop
// For version with sleep and interrupt to conserve power - see other sketch. It's more complicated!

// Keypad: Connect SCL on 8, SDO on 9
// IR Led: Connect "S" to PWM Pin 3, ground via 100-300 Ohms


/*
   NEC sends 0xFFFFFFFF for repeat (not the same command) so detect key held down, then switch to sending REPEAT code.
   It actually sends no data, just
   - 9000us AGC pulse
   - 2250us Header Space (instead of the normal 4500)
   - 560us burst
   Then you need to wait, so the entire repeating sequence still takes 110ms.

   Capture of volume down repeat on the GVA Remote (using IRremote/examples/IRRecvDump):

    FE58A7
    Decoded NEC: FE58A7 (32 bits)
    Raw (68): 8950 -4500 600 -550 550 -550 550 -550 550 -550 550 -550 550 -550 550 -550 550 -550 550 -1700 550 -1650 550 -1650 550 -1650 550 -1700 550 -1650 550 -1650 550 -550 550 -550 550 -1700 550 -550 550 -1650 550 -1650 550 -550 550 -550 550 -550 600 -1650 550 -550 550 -1650 550 -550 550 -550 550 -1700 550 -1650 550 -1650 550
    FFFFFFFF
    Decoded NEC: FFFFFFFF (0 bits)
    Raw (4): 9000 -2200 550
    FFFFFFFF
    Decoded NEC: FFFFFFFF (0 bits)
    Raw (4): 9000 -2250 550
    FFFFFFFF
    Decoded NEC: FFFFFFFF (0 bits)
    Raw (4): 8950 -2250 550
    FFFFFFFF
    Decoded NEC: FFFFFFFF (0 bits)
    Raw (4): 9000 -2200 600
    FFFFFFFF

  This wanders slightly off the above: 550-600 burst, header 2200-2250
*/

#include <IRremote.h>
#include "gva.h" // my home made map of GVA (Good Guys cheap own-brand)

IRsend irsend;

// ---- Logging ----

// remember last time, to provide log offsets for timing
long getDelta() {
  static unsigned long m;
  unsigned long now = millis();
  long delta = now - m;
  m = now;

  return delta;
}
// Logs with time since last message for timing (replaces Serial.println)
void log(char *msg) {
  Serial.print(getDelta());
  Serial.print(": ");
  Serial.println(msg);
}
void log(unsigned long msg, int format) {
  Serial.print(getDelta());
  Serial.print(": ");
  Serial.println(msg, format);
}

// ----

void runtest(unsigned long data,  int nbits) {
  Serial.println("-- Same test again --");
  Serial.println(data, HEX);
  Serial.print(" nbits = ");
  Serial.println(nbits);
  if (nbits == 0) Serial.println("nbits == 0"); else Serial.println("nbits != 0 ");
  unsigned long  mask2 = (1UL << (0 - 1));
  unsigned long  mask3 = (1UL << (nbits - 1));
  Serial.print(" 0 - 1 = ");
  Serial.println(0 - 1);
  Serial.print(" nbits - 1 = ");
  Serial.println(nbits - 1);
  Serial.print(" (1UL << (0 - 1) = ");
  Serial.println(1UL << (0 - 1));
  Serial.print(" (1UL << (nbits - 1) = ");
  Serial.println(1UL << (nbits - 1));
}


void setup() {
  Serial.begin(9600);
}

void loop() {
  //  delay(100);
  static int lastKey = -1;

  int key = 0, nbits = 32;
  long command;

  // Read keypad
  key = XC4602_read(8, 9);
  if (key) {
    //    Serial.print("Key:");
    //    Serial.print(key);
    //    Serial.print(" LastKey:");
    //    Serial.println(lastKey);
    if (key == lastKey) {
      // NEC sends 0xFFFFFFFF for repeat (not the same command) so detect key held down, then switch to sending FFFFFFFF
      command =  REPEAT;
      nbits = 0;

      // Note: I modified sendNEC to works with repeat, avoiding undefined shifting.
      // https://github.com/z3t0/Arduino-IRremote/issues/28
      // https://stackoverflow.com/questions/59156677/how-can-shift-left-give-different-results-in-different-functions
      // TODO: PR/ re-patch in future!
    }
    else {
      // Map the key to a command
      command = mapKeyToCommand(key);
      lastKey = key;
    }
    
    // Send the command e.g. FE6897 = Mute, 32 bits for NEC, (no data bits for repeat signal).
    irsend.sendNEC(command, nbits);
    log(command, HEX);
    delay(92);
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

// Map2: first paper cover trial
long keyMap2[16] = {
  GVA_32TDC15_P_MODE,   // 1
  GVA_32TDC15_S_MODE,   // 2
  GVA_32TDC15_SOURCE,   // 3
  GVA_32TDC15_MUTE,     // 4
  GVA_32TDC15_MENU,     // 5
  GVA_32TDC15_UP,       // 6
  GVA_32TDC15_EJECT,    // 7
  GVA_32TDC15_VOL_UP,   // 8
  GVA_32TDC15_LEFT,     // 9
  GVA_32TDC15_ENTER,    // 10 alt-play after alt-pause
  GVA_32TDC15_RIGHT,    // 11
  GVA_32TDC15_VOL_DN,   // 12
  GVA_32TDC15_EXIT,     // 13
  GVA_32TDC15_DOWN,     // 14
  GVA_32TDC15_PLAY_PAUSE, // 15   // GVA_32TDC15_GRN_STEP, // 10 alt-pause
  GVA_32TDC15_D_MENU,   // 16
};
long mapKeyToCommand(int key) {
  long command;

  command = keyMap2[key - 1];

  return command;
}
