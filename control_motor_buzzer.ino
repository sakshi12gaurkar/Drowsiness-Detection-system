const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int buzzerPin = 8;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'A') { // Alert state
      // Motor stop
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      // Buzzer on
      digitalWrite(buzzerPin, HIGH);
    }
    else if (command == 'N') { // Normal state
      // Motor forward at full speed
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 255);
      // Buzzer off
      digitalWrite(buzzerPin, LOW);
    }
  }
}