int counter = 0;
int currentState = 0;
int lastState = 0;
int reader = 8;
int measureTime = 5; // Seconds
float distancePrTick = 0.0042 ; //Meter 4090589255
float velocity = 0; // Meter / seconds
void setup() {
  Serial.begin(9600);
  pinMode(reader, INPUT);
  lastState = digitalRead(reader);
}

void loop() {
  Serial.println("start");
  unsigned long tid = millis();
  while (tid + (measureTime * 1000) > millis()) {
    currentState = digitalRead(reader);
    if (currentState == 1 && lastState == 0) {
      counter++;
      //Serial.println(counter);
    }
    lastState = currentState;
  }
  velocity = distancePrTick * counter / measureTime;
  counter = 0;
  Serial.print("Hastighed: ");
  Serial.print(velocity);
  Serial.print(" m/s");
  Serial.print(" ; ");
  Serial.print(velocity * 3600 / 1000);
  Serial.println(" km/t");
}
