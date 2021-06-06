// v2 of of microswitch-controlled IR - with a 5 out x 4 in matrix (in the second wooden case).
// This was forked from the 4x4 Keypad "TransmitFromSwitchMatrix" (in plastic case).
//
//
// IR Led: Connect "S" to PWM Pin 3, ground via 100-300 Ohms
// Keypad: 4 x 4 microswit ch matrix with 1k Ohmn on each output column. Inputs are row1Pin etc, which are pulled high internally.
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

#define DEBUG_SERIAL_PRINT
#if defined DEBUG_SERIAL_PRINT
#define DEBUG_PRINTLN(X) Serial.println(X)
#else
#define DEBUG_PRINTLN(X)
#endif
#if defined DEBUG_SERIAL_PRINT
#define DEBUG_PRINTLNHEX(X) Serial.println(X,"HEX")
#else
#define DEBUG_PRINTLNHEX(X)
#endif
#if defined DEBUG_SERIAL_PRINT
#define DEBUG_PRINT(X) Serial.print(X)
#else
#define DEBUG_PRINT(X)
#endif

const int BIT1_MASK = B00001;
const int BIT2_MASK = B00010;
const int BIT3_MASK = B00100;
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
// -- Ardiuno Uno Layout
//int colAPin = 9;
//int colBPin = 11;
//int colCPin = 10;
//int colDPin = 12;
//int colEPin = 4;
//int row1Pin = 8;
//int row2Pin = 6;
//int row3Pin = 5;
//int row4Pin = 7;
// -- Atmega328-P (upside down) layout
int colAPin = 12;
int colBPin = 11;
int colCPin = 10;
int colDPin = 9;
int colEPin = 4;
int row1Pin = 8;
int row2Pin = 6;
int row3Pin = 5;
int row4Pin = 7;

// Diag / UX feedback LED (helps to know it's working!)
int ledPin = 13;

volatile bool woke = false;

// ---- Logging ----

// remember last time, to provide log offsets for timing
long getDelta() {
  static unsigned long m;
  unsigned long now = millis();
  long delta = now - m;
  m = now;

  return delta;
}
// Logs with time since last message for timing
void log(char *msg) {
  DEBUG_PRINT(getDelta());
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(msg);
}
void log(unsigned long msg) {
  DEBUG_PRINT(getDelta());
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(msg);
}

// ---- IDLE sleeping

unsigned long idleTime, idleTimeout = 5000;

// How long since the last key was pressed?
void setIdleTime() {
  idleTime = millis();
  DEBUG_PRINT("setIdleTime now: "); DEBUG_PRINTLN(idleTime); 
}
long getIdleTime() {
  //DEBUG_PRINTLN("getIdleTime millis() - idleTime:"); DEBUG_PRINTLN(millis() - idleTime);
  //long now = millis();
  //long delta = now - idleTime;
  //DEBUG_PRINTLN("getIdleTime delta"); DEBUG_PRINTLN(delta);
  //return delta;
  return millis() - idleTime;
}
bool getIdleTimeout() {
  return getIdleTime() > idleTimeout;
}


// ----

void runtest(unsigned long data,  int nbits) {
  DEBUG_PRINTLN("-- Same test again --");
  DEBUG_PRINTLNHEX(data);
  DEBUG_PRINT(" nbits = ");
  DEBUG_PRINTLN(nbits);
  if (nbits == 0) DEBUG_PRINTLN("nbits == 0"); else DEBUG_PRINTLN("nbits != 0 ");
  unsigned long  mask2 = (1UL << (0 - 1));
  unsigned long  mask3 = (1UL << (nbits - 1));
  DEBUG_PRINT(" 0 - 1 = ");
  DEBUG_PRINTLN(0 - 1);
  DEBUG_PRINT(" nbits - 1 = ");
  DEBUG_PRINTLN(nbits - 1);
  DEBUG_PRINT(" (1UL << (0 - 1) = ");
  DEBUG_PRINTLN(1UL << (0 - 1));
  DEBUG_PRINT(" (1UL << (nbits - 1) = ");
  DEBUG_PRINTLN(1UL << (nbits - 1));
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
/* void pinSleep(){
    for (byte i = 0; i <= A5; i++){
      pinMode (i, OUTPUT);
      digitalWrite (i, LOW); //TODO: THIS doesn't work on the Atmega328 directly??
    }
  pinMode(wakePin, INPUT_PULLUP);
}*/
void pinSleep(){
//  pinMode(colCPin, OUTPUT);
//  pinMode(colDPin, OUTPUT);
//  pinMode(colEPin, OUTPUT);
//  pinMode(colAPin, OUTPUT);
//  pinMode(colBPin, OUTPUT);
  digitalWrite (colAPin, LOW);
  digitalWrite (colBPin, LOW);
  digitalWrite (colCPin, LOW);
  digitalWrite (colDPin, LOW);
  digitalWrite (colEPin, LOW);
//  pinMode(row1Pin, INPUT_PULLUP);
//  pinMode(row2Pin, INPUT_PULLUP);
//  pinMode(row3Pin, INPUT_PULLUP);
//  pinMode(row4Pin, INPUT_PULLUP);
//  pinMode(wakePin, INPUT_PULLUP);
}
// opposite of pinSleep (optimised for speed)
void pinWake(){
  // nothing to do atm
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
  DEBUG_PRINT("IRTransmitFromKeypad: setup...");
  pinSetup();
  delay(2);
  setIdleTime();

  // I can leave the wake interrupt on all the time, it just then fires when awake too. (tested)
  //attachInterrupt(digitalPinToInterrupt(wakePin), wakeUp, CHANGE);

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

  if(woke) {
    DEBUG_PRINTLN("WOKE!");
    woke = false;
  }

  // Read keypad
  key = SwitchMatrix_read();
  if (key) {
    DEBUG_PRINT(" Key:");
    DEBUG_PRINT(key);
    DEBUG_PRINT(" LastKey:");
    DEBUG_PRINT(lastKey);
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
    DEBUG_PRINT(" NEC:");
    DEBUG_PRINTLNHEX(command);
    
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
    //  TODO remove already done by low-power lib? YES NOT NEEDED
    // sleep_enable();                           // enables the sleep bit in the mcucr register

    DEBUG_PRINT(" Idle Timeout: setting up interrupt pins");
    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(wakePin), wakeUp, CHANGE);

    // Take all outputs low, so any switche can pull down the interrupt pin, while the strobing is paused.
    // also optimised pins for power saving
    pinSleep(); 
    
    DEBUG_PRINT(" Idle Timeout: Going to sleep...");
    Serial.flush();
    //delay(100);
    
    // Enter power down state with ADC and BOD module disabled.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    // ... Wakes up when interrupt pin changes.

    // Debug: Flash led to indicate wakeup
    digitalWrite(ledPin, HIGH); // Debug LED

    //delay(50);
    DEBUG_PRINT(" Idle Timeout: Wakeup!");
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(wakePin));

    // restore active pin function
    pinWake();

    // Begin the idle countdown again...
    setIdleTime();
    digitalWrite(ledPin, LOW); // debug LED
  }
}

// Interrupt handler, don't really need to do anything
void wakeUp() {
  woke = true;
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
      if (i == 1) DEBUG_PRINT("A");
      if (i == 2) DEBUG_PRINT("B");
      if (i == 3) DEBUG_PRINT("C");
      if (i == 4) DEBUG_PRINT("D");
      if (i == 5) DEBUG_PRINT("E");
      if (!row1) DEBUG_PRINT("1");
      if (!row2) DEBUG_PRINT("2");
      if (!row3) DEBUG_PRINT("3");
      if (!row4) DEBUG_PRINT("4");
      DEBUG_PRINT(" c=");
      DEBUG_PRINT(col);
      DEBUG_PRINT(" r4=");
      DEBUG_PRINT(row4);
      DEBUG_PRINT(" r3=");
      DEBUG_PRINT(row3);
      DEBUG_PRINT(" r2=");
      DEBUG_PRINT(row2);
      DEBUG_PRINT(" r1=");
      DEBUG_PRINT(row1);
      DEBUG_PRINT(" =>");
      DEBUG_PRINTLN(rows);

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
  //DEBUG_PRINT(" lookup row=");
  //DEBUG_PRINT(row);
  //DEBUG_PRINT(" col=");
  //DEBUG_PRINT(col);

  int hash = row + (col << 4); // shift col into upper nibble

  //DEBUG_PRINT(" hash=");
  //DEBUG_PRINTLN(hash);

  for (int i = 0; i <= 19; i++) {
    //DEBUG_PRINT(" SwitchMatrixMap[i]=");
    //DEBUG_PRINTLN(SwitchMatrixMap[i]);
    if(SwitchMatrixMap[i] == hash) DEBUG_PRINTLN(i+1);
    if(SwitchMatrixMap[i] == hash) return i + 1;
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

// Map4: microswitch 4 row / 5 col - Wooden Case #1
long keyMap4[20] = {
  GVA_32TDC15_MUTE,         //  1 A1
  GVA_32TDC15_UP,           //  2 A2
  GVA_32TDC15_POWER,        //  3 A3
  GVA_32TDC15_MENU,         //  4 A4  // TV Menu, aspect ratio
  GVA_32TDC15_LEFT,         //  5 B1
  GVA_32TDC15_DOWN,         //  6 B2
  GVA_32TDC15_RIGHT,        //  7 B3
  GVA_32TDC15_SOURCE,       // 8 B4  
  GVA_32TDC15_EXIT,         //  9 C1
  GVA_32TDC15_ENTER,        // 10 C2
  GVA_32TDC15_PLAY_PAUSE,   // 11 C3 alt-play after alt-pause // GVA_32TDC15_GRN_STEP, // alt-pause
  GVA_32TDC15_D_SETUP_OSD,  // 12 C4  
  GVA_32TDC15_D_MENU,       // 13 D1
  GVA_32TDC15_VOL_UP,       // 14 D2
  GVA_32TDC15_EJECT,        // 15 D3
  GVA_32TDC15_S_MODE,       // 16 D4
  GVA_32TDC15_FAST_BACK,    // 17 E1
  GVA_32TDC15_VOL_DN,       // 18 E2
  GVA_32TDC15_FAST_FORW,    // 19 E3
  GVA_32TDC15_P_MODE,       // 20 E4
};
long mapKeyToCommand(int key) {
  long command;

  command = keyMap4[key - 1];

  return command;
}
