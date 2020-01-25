// Second version of keyboard controlled sending IR IDs

// This version trials using the Low Power mode, and an interrupt from the keypad

// TODO: need to prove we can wake on the interruypt and then read the keypad signal OK

// TODO: Also see this example of adding an interrupt on the SDO Pin - no need to crosslink https://forum.arduino.cc/index.php?topic=301382.msg2252580#msg2252580
//    See stumpy.cpp, he uses INT1 on Pin 3, and also jumpers TP1/SAHL for active high serial out.
//                                              presumably for the rising edge detection setup.

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


void setup() {
  Serial.begin(9600);
  Serial.println("setup...");

  // Configure wake up pin as input.
  // This will consumes few uA of current.
  pinMode(wakeUpPin, INPUT);
  // initial listen
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);
}

// interrupt function
// Note: this gets called multiple times during the key read process... even detatching the interrupt does't stop the final KEYUP trigger. So I semaphored back and ignore it.
void wakeUp() {
  if (!keyReady) {
    //delay(50); // helps the serial output
    //Serial.println("wakeup!");  delay(50); // helps the serial output

    // Semaphore to the loop
    keyReady = true;
  }
  else {
    //delay(50); // helps the serial output
    //Serial.println("IGNORE wakeup!");  delay(50); // helps the serial output
  }
}

void loop() {

  // Check semaphore.
  // Only clear the flag when they lift their finger (no key detected) to continue transmitting the NEC 0xFFFFFFFF repeats..
  if (keyReady) {

    // Disable interrupt to prevent further triggers (as the serial IO will now flap the pin as we clock it)
    detachInterrupt(digitalPinToInterrupt(wakeUpPin));

    int key = readKeySendCommand();
    if (!key) {
      // Done for now, go back to sleep
      // Wake up when the interrupt pin falls low:
      // INT0 is strapped to the keypad SDO to detect "DV" message (Data Valid) at the start of the serial message (can be configured high or low).
      attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);

      // Semaphore to the interrupt handler, we're ready to listen to keys again (to ignore the key-up trigger) TODO: do this better!
      keyReady = false;

      // Enter power down state with ADC and BOD module disabled.
      //Serial.print("LowPower.powerDown...");  delay(50); // need delay else the logging doesn't get sent before power down.
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

      //delay(50); // also helps the serial output
      //Serial.println("...LowPower.powerDown ended!");

    }
    // GVA remote test showed a 100-102ms delay between 0xFFFFFFFF repeats, sometimes hiccupping to 200ms.
    // delay(100) here gave 180ms delay
    // delay(10) here gave 99ms delay
    else delay(101); // delay only needed to slow down the key repeat 0xFFFFFFFF transmissions, to roughly what I saw the real remote doing
  }
}

// Returns any key detected, or none
int readKeySendCommand() {
  static int lastKey = -1;
  int key = 0;
  long command;
  bool newKey = false;

  // Read keypad
  key = XC4602_read(8, 9);
  if (key) {
    //Serial.print("Key ");
    //Serial.println(key);

    //Serial.print("LastKey ");
    //Serial.println(lastKey);
    if (key == lastKey) {
      // NEC sends 0xFFFFFFFF for repeat (not the same command) so detect key held down, then switch to sending 0xFFFFFFFF
      // But I couldn't get it to work on the GVA tv. I tried a raw mode, a modified header...
      command = REPEAT; // 0xFFFFFFFF;

      // 3. try sending the same command faster, after initial delay
      //command = mapKeyToCommand(key);
      irsend.sendNEC(command, 32);
      
      // 2. try mod in sendNEC
      // irsend.sendNEC(command, 0);

      // 1. tried this raw suggestion but it didn't work?
      // sendRawRepeat();
      // delay(80);// makes up for lack of irsend processing time
    }
    else {
      // Map the key to a command
      command = mapKeyToCommand(key);
      lastKey = key;
      newKey = true;
      // Send the command e.g. FE6897 = Mute, 32 bits for NEC
      // IRsend::sendNEC (unsigned long data,  int nbits)
      irsend.sendNEC(command, 32);
    }

    //Serial.print("Sent ");
    log(command, HEX);

    //if (newKey) delay(100); // initial delay to single keypress repeat (most keyboards have this although the VGA remote doesn't seem to)
  }
  else {
    // reset if no key after timeout, to allow for slow repeat.
    lastKey = -1;
  }

  return key;
}

// ---- lib ----

// this didn't work!
// https://github.com/z3t0/Arduino-IRremote/issues/28#issuecomment-207855989_
void sendRawRepeat() {
  unsigned int buf[3];
  buf[0] = 9000;  // Mark 9ms
  buf[1] = 2250;  // Space 2.25ms
  buf[2] = 560;   // Burst
  irsend.sendRaw(buf, 3, 38);
}

// I Patched IRsend::sendNEC to amend header space
//    // PJ: added to fix repeat
//    if (nbits == 0) space(NEC_RPT_SPACE);
//    else space(NEC_HDR_SPACE);


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

// Map 1, generally useful ones, no layout!
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
  GVA_32TDC15_PLAY_PAUSE, // 10   // GVA_32TDC15_GRN_STEP, // 10 alt-pause
  GVA_32TDC15_ENTER, // 11 alt-play after alt-pause
  GVA_32TDC15_P_MODE,  // 12
  GVA_32TDC15_S_MODE,  // 13
  GVA_32TDC15_SOURCE,  // 14
  GVA_32TDC15_MENU,   //
  GVA_32TDC15_D_MENU, //= left
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
