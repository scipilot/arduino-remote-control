// Initial draft of BUTTON controlled IR
// This was forked from the Capacitive Keypad "TransmitFromKeypad" which was unreliable.
//

// IR Led: Connect "S" to PWM Pin "ledPin" const, ground via 100-300 Ohms
// Keys: TBC

// todo: make a short initial repeat delay, e.g. 150ms, then a faster repeat loop e.g. 50ms (or none really, the sending takes 80ms)

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

#define LOOP_DELAY 500

const COL1_MASK = 7;
const COL2_MASK = 11;
const COL3_MASK = 13;
const COL4_MASK = 14;

// Row / Col pins of the 4x4 switch matrix
int col1Pin = 5;
int col2Pin = 6;
int col3Pin = 7;
int col4Pin = 8;
int row1Pin = 9;
int row2Pin = 10;
int row3Pin = 11;
int row4Pin = 12;
int ledPin = 13;

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
  Serial.print("IRTransmitFromKeypad: setup...");
  pinMode(ledPin, OUTPUT); // LED for diag when command is transmitted
  pinMode(col1Pin, OUTPUT);
  pinMode(col2Pin, OUTPUT);
  pinMode(col3Pin, OUTPUT);
  pinMode(col4Pin, OUTPUT);
  pinMode(row1Pin, INPUT_PULLUP);
  pinMode(row2Pin, INPUT_PULLUP);
  pinMode(row3Pin, INPUT_PULLUP);
  pinMode(row4Pin, INPUT_PULLUP);
  delay(2);

}

void loop() {
  //delay(LOOP_DELAY);
  static int lastKey = -1;

  int key = 0, nbits = 32;
  long command;

  // Read keypad
  key = SwitchMatrix_read();
  if (key) {
        Serial.print(" Key:");
        Serial.print(key);
        Serial.print(" LastKey:");
        Serial.print(lastKey);
    if (key == lastKey) {
      // NEC sends 0xFFFFFFFF for repeat (not the same command) so detect key held down, then switch to sending FFFFFFFF
      command =  REPEAT;
      nbits = 0;

      // Note: I modified sendNEC to work with repeat, avoiding undefined shifting.
      // https://github.com/z3t0/Arduino-IRremote/issues/28
      // https://stackoverflow.com/questions/59156677/how-can-shift-left-give-different-results-in-different-functions
      // See PR for fix https://github.com/z3t0/Arduino-IRremote/pull/609
    }
    else {
      // Map the key to a command
      command = mapKeyToCommand(key);
      lastKey = key;
    }
    
    // Flash led while sending
    digitalWrite(ledPin, HIGH); // Debug LED
    
    // Send the command e.g. FE6897 = Mute, 32 bits for NEC, (no data bits for repeat signal).
    irsend.sendNEC(command, nbits);
    Serial.print(" NEC:");
    Serial.print(command, HEX);
    //log(command, HEX);
    Serial.println("");
    delay(92);

    digitalWrite(ledPin, LOW); // debug LED
  }
  else {
    // reset if no key after timeout, to allow for slow repeat.
    lastKey = -1;
  }
}

// ---- lib ----

// returns key number or 0 if no key pressed
int SwitchMatrix_read() {
  int key = 0;                           //default to no keys pressed
  int col, row;

  // To find the switch pressed, must scan through all four outputs and read the inputs, then map.
  
  // TODO LOOP THIS!
//  for (int i = 1; i <= 4; i++) {

    col = 0;
    col << 1;
    digitalWrite(col1Pin, HIGH);
    col << 0;
    digitalWrite(col2Pin, LOW);
    col << 0;
    digitalWrite(col3Pin, LOW);
    col << 0;
    digitalWrite(col4Pin, LOW);

    row = 0;
    row << digitalRead(row4Pin);
    row << digitalRead(row3Pin);
    int row2 = digitalRead(row2Pin);
    row << row2;
  Serial.print("r2=");
  Serial.println(row2);
    int row1 = digitalRead(row1Pin);
    row << row1;
  Serial.print("r1=");
  Serial.println(row1);
  Serial.println(row);
                                                                                                     
    if(row) return lookupKey(row,col);


  delay(50);
  digitalWrite(col1Pin, LOW);
  delay(20);
  digitalWrite(col2Pin, HIGH);
  delay(50);
  digitalWrite(col2Pin, LOW);
  delay(20);
  digitalWrite(col3Pin, HIGH);
  delay(50);
  digitalWrite(col3Pin, LOW);
  delay(20);
  digitalWrite(col4Pin, HIGH);
  delay(51);
  digitalWrite(col4Pin, LOW);
  delay(200);

/* 2nd scan etc...
 *  
    digitalWrite(col1Pin, LOW);
    digitalWrite(col2Pin, HIGH);
    digitalWrite(col3Pin, LOW);
    digitalWrite(col4Pin, LOW);

    row = 0;
    row << digitalRead(row4pin);
    row << digitalRead(row3pin);
    row << digitalRead(row2pin);
    row << digitalRead(row1pin);

    if(key) return key;
*/
//  }
  
  return key;
}

// Sets the output pins to the state in the bit mask.
// mask: 1 byte, lower nibble: 1110, 1101, 1011, 0111 = 14, 13, 11 or 7
int setOutputs(int mask){
    digitalWrite(colPin1, mask & 1 ? HIGH : LOW);
    digitalWrite(colPin2, mask & 2 ? HIGH : LOW);
    digitalWrite(colPin3, mask & 4 ? HIGH : LOW);
    digitalWrite(colPin4, mask & 8 ? HIGH : LOW);
}

// Lookup the index of the row-col combo, only supports one key at a time though.
// Encoding: Col nibble is shifted above Row nibble = 1 byte
int SwitchMatrixMap[16] = {

  B00010001,    // 1
  B00010010,    // 2
  B00010100,    // 3
  B00011000,    // 4

  B00100001,    // 5
  B00100010,    // 6
  B00100100,    // 7
  B00101000,    // 8

  B01010001,    // 9
  B01010010,    // 10
  B01010100,    // 11
  B01011000,    // 12

  B10000001,    // 13
  B10000010,    // 14
  B10000100,    // 15
  B10001000,    // 16
};

// Converts the row-col combination into our key index. 
// Returns 1-16, or 0 for error/not found.
int lookupKey(int row, int col){
  int hash = row + col * 16; // shift col into upper nibble

  for(int i; i=0; i++){
    if(SwitchMatrixMap[i] == hash) return i+1;
  }

  // oops, could be a multi-key combo?
  return 0;
}

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
