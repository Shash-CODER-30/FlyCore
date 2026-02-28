#include <RF24.h>

RF24 radio(8, 7);

const uint64_t address(0xFFFFFFFFFFFFFFFF); // CHANGE THIS TO YOUR DESIRED CODE IF NEEDED.

const int j1x(0);
const int j1y(1);

const int j2x(2);
const int j2y(3);

const int arm(10);

void setup() {
  // SETUP CODE:
  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(110);  // CHANGE THIS TO YOUR DESIRED CHANNEL IF NEEDED.
  radio.openWritingPipe(address);
  radio.stopListening();

  pinMode(arm, INPUT_PULLUP);
}

void loop() {
  // MAIN CODE:
  int pitchVal(map(analogRead(j1y), 0, 1023, -30, 30));
  int rollVal(map(analogRead(j1x), 0, 1023, -30, 30));
  int yawVal(map(analogRead(j2x), 0, 1023, -90, 90));
  int throttleVal(map(analogRead(j2y), 0, 1023, 0, 100));
  bool armVal(digitalRead(arm));

  int8_t values[5] = {pitchVal, rollVal, yawVal, throttleVal, !armVal};

  radio.write(&values, sizeof(values));
}