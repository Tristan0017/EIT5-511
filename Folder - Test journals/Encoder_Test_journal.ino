int Interrupts = 0;
int currentState = 0;
int lastState = 0;
int reader = 8;
int measureTime = 5; // Seconds
//float distancePrInterrupt = 0.004188842565 ; //Meter
float velocity = 0; // Meter / seconds
void setup() {
  Serial.begin(9600);
  pinMode(reader, INPUT);
  lastState = digitalRead(reader);
}
void loop() {
  Serial.println("start");
  unsigned long Time = millis();
  while (Time + (measureTime * 1000) > millis()) {
    currentState = digitalRead(reader);
    if (currentState == 1 && lastState == 0) {
      Interrupts++;
    }
    lastState = currentState;
  }
  velocity = 1.4472 * Interrupts / measureTime;
  Interrupts = 0;
  Serial.print("Hastighed: ");
  Serial.print(velocity);
  Serial.print(" m/s");
  Serial.print(" ; ");
  Serial.print(velocity);
  Serial.println(" km/t");
}
