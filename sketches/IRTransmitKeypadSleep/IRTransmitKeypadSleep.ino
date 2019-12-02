// Second version of keyboard controlled sending IR IDs

// This version trials using the Low Power mode, and an interrupt from the keypad

// TODO: need to prove we can wake on the interruypt and then read the keypad signal OK

// Keypad: Connect SCL on 8, SDO on 9
// IR Led: Connect "S" to PWM Pin 3, ground via 100-300 Ohms

#include <IRremote.h>
#include <gva.h> // my home made map of GVA (Good Guys cheap own-brand)
#include <LowPower.h>

IRsend irsend;

/* TTP229-BSF keypad datasheet:
  "The 2-wires serial mode supports always polling data for other device on the system.
  Or other device can wait that TTP229-BSF outputs the data valid (DV) signal by the SDO pin,
  and it can give the clock signal to TTP229-BSF SCL pin and get the key data from SDO pin."

  So need to bind and interrupt pin to SDO on 9. Is that safe?
  Pin 2 is the only other interrupt pin: INT0
*/
const int wakeUpPin = 2;
bool keyReady = false;

void setup() {
  Serial.begin(9600);
  Serial.println("setup...");

  // Configure wake up pin as input.
  // This will consumes few uA of current.
  pinMode(wakeUpPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);
}

// interrupt function
// TODO this gets called multiple times during the key read process... perhaps detatch the interupt during key read
void wakeUp() {
  Serial.println("wakeup!");

  // Semaphore to the loop
  keyReady = true;
}

void loop() {
  //delay(100); // delay not needed with sleep
  keyReady = false;

  ///*
  // Enter power down state with ADC and BOD module disabled.
  // Wakes up again when the interrupt pin falls low.
  Serial.print("LowPower.powerDown...");  delay(50); // need delay else the logging doesn't get sent before power down.
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  delay(50); // also helps the serial output
  Serial.println("...LowPower.powerDown ended!"); 
  // */

  if (keyReady) readKeySendCommand();
}

void readKeySendCommand() {
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
