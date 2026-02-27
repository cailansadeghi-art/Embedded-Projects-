// HC-SR04 live position + velocity grapher
// Works with Arduino UNO R4 WiFi + Serial Plotter

const int trigPin = 9;
const int echoPin = 8;

float lastDistance = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Send a header for readability in Serial Monitor (Serial Plotter ignores)
  Serial.println("distance_cm,velocity_cms");
}

float getDistanceCM() {
  // Trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo time (microseconds)
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms

  if (duration == 0) return -1;  // no reading

  // speed of sound = 0.0343 cm/us
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}

void loop() {
  float dist = getDistanceCM();

  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;  // convert to seconds

  float velocity = 0;

  if (lastTime != 0 && dist > 0 && lastDistance > 0) {
    velocity = (dist - lastDistance) / dt;  // cm/s
  }

  lastDistance = dist;
  lastTime = now;

  // Serial Plotter expects comma-separated or space-separated values.
  Serial.print(dist);
  Serial.print(",");
  Serial.println(velocity);

  delay(50); // ~20 Hz update rate
}
