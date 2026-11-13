// Standalone wiring test: no serial, no GUI.
// Wiring: D8 -> PUL+, D9 -> PUL-, D10 -> DIR+, D11 -> DIR-
// Forever: 1600 steps one way (onboard LED ON), pause, 1600 steps back (LED OFF), pause.
// Steps at 400/s, slow enough that any driver should follow.

const uint8_t PUL = 8, PUL_GND = 9, DIR = 10, DIR_GND = 11, LED = 13;

void move(bool forward) {
  digitalWrite(DIR, forward ? HIGH : LOW);
  digitalWrite(LED, forward ? HIGH : LOW);
  delayMicroseconds(200);  // CL57Y needs DIR stable >125us before the first pulse
  for (int i = 0; i < 1600; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(20);
    digitalWrite(PUL, LOW);
    delayMicroseconds(2480);
  }
}

void setup() {
  pinMode(PUL, OUTPUT);     digitalWrite(PUL, LOW);
  pinMode(PUL_GND, OUTPUT); digitalWrite(PUL_GND, LOW);
  pinMode(DIR, OUTPUT);     digitalWrite(DIR, LOW);
  pinMode(DIR_GND, OUTPUT); digitalWrite(DIR_GND, LOW);
  pinMode(LED, OUTPUT);
}

void loop() {
  move(true);
  delay(1000);
  move(false);
  delay(1000);
}
