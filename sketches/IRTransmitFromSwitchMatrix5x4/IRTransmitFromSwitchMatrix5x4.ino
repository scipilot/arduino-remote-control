// v2 of of microswitch-controlled IR - with a 5 out x 4 in matrix (in the second wooden case).
// This was forked from the 4x4 Keypad "TransmitFromSwitchMatrix" (in plastic case).
//
//
// IR Led: Connect "S" to PWM Pin 3, ground via 100-300 Ohms
// Keypad: 4 x 4 microswitch matrix with 1k Ohmn on each output column. Inputs are row1Pin etc, which are pulled high internally.
//
// todo: make a short initial repeat delay, e.g. 150ms, then a faster repeat loop e.g. 50ms (or none really, the sending takes 80ms)

/*
   The IDLE sleep feature (better alternative to duty-cycle-sleep, but requires more circuitry):
  - Requires interrupt Pin 2 to be strapped to the 4 inputs via diodes (to prevent cross-interference).
  - In sleep, the matrix outputs are all set low, and strobing stops.
  - The interrupt pin floats high, via its internal pullups.
  - When any switch is pressed during sleep, the interrupt pin is pulled low  via the matrix outputs, triggering the wakeup process.
  - Then the key can be read normally .
*/
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

  Benchmarking REPEAT timing (26/04/2020) found the repeat cycle (see specs/IR-NEC-repeat-timing.png)
  - with all the printSerials in place (but no monitor connected)
  - with the extra delay(92)
  - with LOOP_DELAY 50  = 266ms and doesn't work
  - with lOOP_DELAY  1  = 216ms and doesn't wrk
  - then removed the delay(92) below!
  - with lOOP_DELAY  1  = 126ms and WORKS (should be 110ms)

*/

#include <IRremote.h>
#include "gva.h" // my home made map of GVA (Good Guys cheap own-brand)
#include <LowPower.h>

#include <avr/sleep.h>
#include <avr/power.h>

// borrows from https://github.com/genotix/ulp_nanov3_sleep/blob/master/low_power_interrupt_v9/low_power_interrupt_v9.ino
//#define debugmode     false
//#define testmode      false   // You can use the plotter in test mode
//#include "tools.h"
//#include "cpu.h"

IRsend irsend;

#define LOOP_DELAY 100

const int BIT1_MASK = B00001;
const int BIT2_MASK = B00010;
const int BIT3_MASK = B00010;
const int BIT4_MASK = B01000;
const int BIT5_MASK = B10000;

const int COL1_MASK = B01111;
const int COL2_MASK = B10111;
const int COL3_MASK = B11011;
const int COL4_MASK = B11101;
const int COL5_MASK = B11110;

const int ROW1_MASK = COL1_MASK;
const int ROW2_MASK = COL2_MASK;
const int ROW3_MASK = COL3_MASK;
const int ROW4_MASK = COL4_MASK;

// IDLE Sleep mode, requires an interrupt pin
int wakePin = 2;
// IR PIN is PWM pin 3 by default
// Row / Col pins of the 5x4 switch matrix
int colAPin = 8;
int colBPin = 7;
int colCPin = 6;
int colDPin = 5;
int colEPin = 4;
int row1Pin = 12; // v2 PCB layout was easier in this order [12,10,9,11]
int row2Pin = 10;
int row3Pin = 9;
int row4Pin = 11;
// Diag / UX feedback LED (helps to know it's working!)
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

// ---- IDLE sleeping

unsigned long idleTime, idleTimeout = 5000;

// How long since the last key was pressed?
void setIdleTime() {
  idleTime = millis();
  Serial.println("setIdleTime now:"); Serial.println(idleTime);
}
long getIdleTime() {
  //Serial.println("getIdleTime millis() - idleTime:"); Serial.println(millis() - idleTime);
  //long now = millis();
  //long delta = now - idleTime;
  //Serial.println("getIdleTime delta"); Serial.println(delta);
  //return delta;
  return millis() - idleTime;
}
bool getIdleTimeout() {
  return getIdleTime() > idleTimeout;
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

// Set pins to active state
void pinSetup(){
  pinMode(ledPin, OUTPUT); // LED for diag when command is transmitted
  pinMode(colAPin, OUTPUT);
  pinMode(colBPin, OUTPUT);
  pinMode(colCPin, OUTPUT);
  pinMode(colDPin, OUTPUT);
  pinMode(colEPin, OUTPUT);
  pinMode(row1Pin, INPUT_PULLUP);
  pinMode(row2Pin, INPUT_PULLUP);
  pinMode(row3Pin, INPUT_PULLUP);
  pinMode(row4Pin, INPUT_PULLUP);
  pinMode(wakePin, INPUT_PULLUP);
}
// Set pins to the optimum low-power state for sleeping (see benchmarks, seems to vary on different hardware)
// Note: for interupt design, must take outputs low during sleep, so any key-switch can pull down the interrupt pin while the strobing is paused.
void pinSleep(){
    for (byte i = 0; i <= A5; i++){
      pinMode (i, OUTPUT);
      digitalWrite (i, LOW);
    }
  pinMode(wakePin, INPUT_PULLUP);
}

/* Example of low power benchmark (try INPUT/OUTPUT * LOW/HIGH - see notes)
 void setupGammonDemoLowPower () {

  for (byte i = 0; i <= A5; i++)
    {
    pinMode (i, INPUT);    // changed as per below
    digitalWrite (i, LOW);  //     ditto
    }
    
  // disable ADC
  ADCSRA = 0;  
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  noInterrupts ();           // timed sequence follows
  sleep_enable();
 
  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
  interrupts ();             // guarantees next instruction executed
  sleep_cpu ();              // sleep within 3 clock cycles of above

}  // end of setup
*/

void setup() {
  Serial.begin(9600);
  Serial.print("IRTransmitFromKeypad: setup...");
  pinSetup();
  delay(2);
  setIdleTime();

  // TODO remove POWER SAVING EXPERIMENTS
  // disable_wdt(); // todo experiment with power reduction
  // disable_adc();
  // disable_uart();             // Disable internal serial logic
  // inputs_lowpower(); // can't use
}

void loopNull(){}

// Main loop
void loop() {
  //delay(LOOP_DELAY);
  static int lastKey = -1;
  int key = 0, nbits = 32;
  long command;

  // TODO: This is being replaced by the idle-timeout-sleep instead of a sleep duty cycle
  // Power saving option: DUTY SLEEP (note: this breaks the serial port timing!)
  // Enter power down state for 8 s with ADC and BOD module disabled
  // LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);

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
      // https://github.com/z3t0/Arduino-IRremote/issues/28
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
    log(command, HEX);
    Serial.println("");

    digitalWrite(ledPin, LOW); // debug LED

    setIdleTime();
  }
  else {
    // reset if no key after timeout, to allow for slow repeat.
    lastKey = -1;
  }

  // todo remove, disable the sleeping
  //return;

  // Check for idle timeout and go to sleep to save power
  if (getIdleTimeout()) {
    Serial.print(" Idle Timeout!");
    //  TODO remove already done by low-power lib? sleep_enable();                           // enables the sleep bit in the mcucr register

    Serial.print(" Idle Timeout: setting up interrupt pins");
    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(wakePin), wakeUp, CHANGE);

    // Take all outputs low, so the switches can pull down the interrupt pin, while the strobing is paused.
    // setOutputs(31);
    pinSleep(); // now does all pins for power saving

    Serial.print(" Idle Timeout: Going to sleep...");
    delay(100);
    
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Debug: Flash led to indicate wakeup
    digitalWrite(ledPin, HIGH); // Debug LED

    delay(100);
    Serial.print(" Idle Timeout: Wakeup!");
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(wakePin));

    // restore active pin function
    pinSetup();

    // Begin the idle countdown again...
    setIdleTime();
    digitalWrite(ledPin, LOW); // debug LED
  }
}

// Interrupt handler, don't really need to do anything
void wakeUp() {
}

// ---- lib ----

// returns key number or 0 if no key pressed
int SwitchMatrix_read() {
  int key = 0;                           //default to no keys pressed
  int col, rows, row;
  int row1, row2, row3, row4;

  // To find the switch pressed, must scan through all four outputs and read the inputs, then map.
  col = B00001;
  for (int i = 1; i <= 5; i++) {

    // Strobe the outputs
    setOutputs(col);
    // Delay SEEMS NOT NEEDED:
    //delay(50); // let pins settle? (was 10 which adds 27ms to cycle, found 1 works OK)

    rows = 0;
    row4 = row = digitalRead(row4Pin);
    rows = rows + BIT4_MASK * !row;
    row3 = row = digitalRead(row3Pin);
    rows = rows + BIT3_MASK * !row;
    row2 = row = digitalRead(row2Pin);
    rows = rows + BIT2_MASK * !row;
    row1 = row = digitalRead(row1Pin);
    rows = rows + BIT1_MASK * !row;
    if (rows) {
      // Print a "standard key identifier" for easier testing: A1 thru E4, e.g. where A = output1 (D8) +  1 = input 1 (D12)
      if (i == 1) Serial.print("A");
      if (i == 2) Serial.print("B");
      if (i == 3) Serial.print("C");
      if (i == 4) Serial.print("D");
      if (i == 5) Serial.print("E");
      if (!row1) Serial.print("1");
      if (!row2) Serial.print("2");
      if (!row3) Serial.print("3");
      if (!row4) Serial.print("4");
      Serial.print(" c=");
      Serial.print(col);
      Serial.print(" r4=");
      Serial.print(row4);
      Serial.print(" r3=");
      Serial.print(row3);
      Serial.print(" r2=");
      Serial.print(row2);
      Serial.print(" r1=");
      Serial.print(row1);
      Serial.print(" =>");
      Serial.println(rows);

    }
    // First key found takes precenence
    if (rows) {
      setOutputs(0);
      return lookupKey(rows, col);
    }

    // Else keep trying
    col = col << 1;
  }

  setOutputs(0);
  return 0;
}

// Sets the output pins to the state in the bit mask.
// mask: 1 byte, lower nibble: 11110, 11101, 11011, 10111, 01111 = 30, 29, 27, 23 or 15
// Note: 0 is "on" (I invert it because low and not-powered are not distinguishable)
int setOutputs(int mask) {
  digitalWrite(colAPin, mask & 1 ? LOW : HIGH);
  digitalWrite(colBPin, mask & 2 ? LOW : HIGH);
  digitalWrite(colCPin, mask & 4 ? LOW : HIGH);
  digitalWrite(colDPin, mask & 8 ? LOW : HIGH);
  digitalWrite(colEPin, mask & 16 ? LOW : HIGH);
}

// Lookup the index of the row-col combo, only supports one key at a time though.
// Encoding: Col nibble is shifted above Row nibble = 1 byte
// TODO now with 5 cols, the upper "nibble" is 5 bits! is that OK?
int SwitchMatrixMap[20] = {

  0b000010001,    // 1
  0b000010010,    // 2
  0b000010100,    // 3
  0b000011000,    // 4

  0b000100001,    // 5
  0b000100010,    // 6
  0b000100100,    // 7
  0b000101000,    // 8

  0b001000001,    // 9
  0b001000010,    // 10
  0b001000100,    // 11
  0b001001000,    // 12

  0b010000001,    // 13
  0b010000010,    // 14
  0b010000100,    // 15
  0b010001000,    // 16

  0b100000001,    // 17
  0b100000010,    // 18
  0b100000100,    // 19
  0b100001000,    // 20
};

// Converts the row-col combination into our key index.
// Returns 1-16, or 0 for error/not found.
int lookupKey(int row, int col) {
  //    Serial.print(" lookup row=");
  //    Serial.print(row);
  //    Serial.print(" col=");
  //    Serial.print(col);

  int hash = row + (col << 4); // shift col into upper nibble

  //    Serial.print(" hash=");
  //    Serial.println(hash);

  for (int i = 0; i <= 19; i++) {
    // Serial.print(" SwitchMatrixMap[i]=");
    // Serial.println(SwitchMatrixMap[i]);
    //    if(SwitchMatrixMap[i] == hash) Serial.println(i+1);
    if (SwitchMatrixMap[i] == hash) return i + 1;
  }

  // oops, could be a multi-key combo?
  return 0;
}

/*
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
  // Map3: microswitch 4 row/ 4 col (same layout as Map2)
  long keyMap3[16] = {
  GVA_32TDC15_MUTE,     //  1
  GVA_32TDC15_VOL_UP,   //  2
  GVA_32TDC15_VOL_DN,   //  3
  GVA_32TDC15_D_MENU,   //  4
  GVA_32TDC15_SOURCE,   //  5
  GVA_32TDC15_EJECT,    //  6
  GVA_32TDC15_RIGHT,    //  7
  GVA_32TDC15_PLAY_PAUSE, // 8   // GVA_32TDC15_GRN_STEP, // 10 alt-pause
  GVA_32TDC15_S_MODE,   //  9
  GVA_32TDC15_UP,       // 10
  GVA_32TDC15_ENTER,    // 11 alt-play after alt-pause
  GVA_32TDC15_DOWN,     // 12
  GVA_32TDC15_P_MODE,   // 13
  GVA_32TDC15_MENU,     // 14
  GVA_32TDC15_LEFT,     // 15
  GVA_32TDC15_EXIT,     // 16
  };
*/

// Map4: microswitch 4 row/ 5 col
long keyMap4[20] = {
  GVA_32TDC15_MUTE,     //  1
  GVA_32TDC15_VOL_UP,   //  2
  GVA_32TDC15_VOL_DN,   //  3
  GVA_32TDC15_D_MENU,   //  4
  GVA_32TDC15_SOURCE,   //  5
  GVA_32TDC15_EJECT,    //  6
  GVA_32TDC15_RIGHT,    //  7
  GVA_32TDC15_PLAY_PAUSE, // 8   // GVA_32TDC15_GRN_STEP, // 10 alt-pause
  GVA_32TDC15_S_MODE,   //  9
  GVA_32TDC15_UP,       // 10
  GVA_32TDC15_ENTER,    // 11 alt-play after alt-pause
  GVA_32TDC15_DOWN,     // 12
  GVA_32TDC15_P_MODE,   // 13
  GVA_32TDC15_MENU,     // 14
  GVA_32TDC15_LEFT,     // 15
  GVA_32TDC15_EXIT,     // 16
  GVA_32TDC15_EXIT,     // 17
  GVA_32TDC15_EXIT,     // 18
  GVA_32TDC15_EXIT,     // 19
  GVA_32TDC15_EXIT,     // 20
};
long mapKeyToCommand(int key) {
  long command;

  command = keyMap4[key - 1];

  return command;
}
