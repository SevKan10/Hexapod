#define batPin 34
#define LED 2
void setup() {
  Serial.begin(115200);
  pinMode(batPin, INPUT);
  pinMode(LED, OUTPUT);
}

void loop() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {sum += analogRead(batPin);delay(2);}
  int rawADC = sum / 10;
  // int batPercent = map(analogRead(batPin), 0, 4095, 0, 100);
  int batPercent = map(rawADC, 2640, 3475, 0, 100);
  batPercent = constrain(batPercent, 0, 100);
  Serial.printf("Battery Percent: %i%%\n", batPercent);
//  Serial.println("Battery Percent: " + String(batPercent) + "%");

  if(batPercent <= 20){digitalWrite(LED,1);} else{digitalWrite(LED,0);}
  delay(10);
}
