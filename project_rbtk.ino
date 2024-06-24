#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Tambahkan library untuk sensor ultrasonik
#define trigPin 2
#define echoPin 3

Servo servo1;
Servo servo2;

const int servo1Pin = 9;  // Pin servo 1
const int servo2Pin = 10; // Pin servo 2

const float L1 = 10.0; // Panjang lengan 1 (cm)
const float L2 = 10.0; // Panjang lengan 2 (cm)

float xTargets[] = {4, 6, 8, 10, 12}; // Asumsi target
float yTargets[] = {4, 6, 8, 10, 12}; // Asumsi target

int stepDelay = 20;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Alamat I2C LCD dan ukuran (16 kolom x 2 baris)

void calculateInverseKinematics(float x, float y, float &theta1, float &theta2) {
    float r = sqrt(x * x + y * y);

    if (r > (L1 + L2) || r < abs(L1 - L2)) {
        Serial.println("Target out of reach");
        return;
    }

    float cosTheta2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cosTheta2 = constrain(cosTheta2, -1.0, 1.0);

    theta2 = acos(cosTheta2);

    float k1 = L1 + L2 * cos(theta2);
    float k2 = L2 * sin(theta2);

    float phi = atan2(y, x);
    float psi = atan2(k2, k1);

    theta1 = phi - psi;

    theta1 = degrees(theta1);
    theta2 = degrees(theta2);

    Serial.print("Raw Theta1 (degrees): ");
    Serial.println(theta1);
    Serial.print("Raw Theta2 (degrees): ");
    Serial.println(theta2);

    theta1 = constrain(130 - theta1, 0, 180);
    theta2 = constrain(130 - theta2, 0, 180);

    Serial.print("Adjusted Theta1 (degrees): ");
    Serial.println(theta1);
    Serial.print("Adjusted Theta2 (degrees): ");
    Serial.println(theta2);

    // Tampilkan sudut di LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Theta1: ");
    lcd.print(theta1);

    lcd.setCursor(0, 1);
    lcd.print("Theta2: ");
    lcd.print(theta2);
}

long readUltrasonicDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2;
    
    return distance;
}

void setup() {
    Serial.begin(9600);

    // Inisialisasi LCD
    lcd.init();
    lcd.backlight();

    // Attach servo ke pin
    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);

    // Inisialisasi pin ultrasonik
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void loop() {
    long distance = readUltrasonicDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    for (int i = 0; i < sizeof(xTargets) / sizeof(xTargets[0]); i++) {
        if (distance >= xTargets[i] && distance <= xTargets[i] + 2) {
            float x = xTargets[i];
            float y = yTargets[i];

            float theta1 = 0, theta2 = 0;
            calculateInverseKinematics(x, y, theta1, theta2);

            moveServo(servo1, theta1);
            moveServo(servo2, theta2);

            delay(1000);
        }
    }
}

void moveServo(Servo &servo, int targetAngle) {
    int currentAngle = servo.read();

    if (currentAngle < targetAngle) {
        for (int angle = currentAngle; angle <= targetAngle; angle++) {
            servo.write(angle);
            delay(stepDelay);
        }
    } else {
        for (int angle = currentAngle; angle >= targetAngle; angle--) {
            servo.write(angle);
            delay(stepDelay);
        }
    }
}
