
const int motorCounterPin = 3;
volatile int pulseCount = 0;

void counterInterrupt() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);
  pinMode(motorCounterPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(motorCounterPin), counterInterrupt, RISING);
}

void loop() {
  Serial.println("Pulse Count: " + String(pulseCount));
  delay(10);
}
