//Connect SCL on 8, SDO on 9
void setup() {
  Serial.begin(9600);
}

void loop() {
  int key = 0;

  key = XC4602_read(8, 9);
  if (key) Serial.println(key);
  delay(100);
}

int XC4602_read(int sclpin, int sdopin) { //returns key number or 0 if no key pressed
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
