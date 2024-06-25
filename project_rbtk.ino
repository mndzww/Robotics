#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

Servo servo1;
Servo servo2;

const int servo1Pin = 9;  // Pin servo 1
const int servo2Pin = 10; // Pin servo 2

const float L1 = 10.0; // Panjang lengan 1 (cm)
const float L2 = 10.0; // Panjang lengan 2 (cm)

float xTargets[] = {4, 6, 8, 10, 12};
float yTargets[] = {4, 6, 8, 10, 12};

float theta1Values[] = {158.57, 149.90, 140.55, 130, 116.95};
float theta2Values[] = {0, 0.21, 18.9, 40, 66.10};

int stepDelay = 0; // Menghilangkan delay untuk responsivitas tinggi

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Alamat I2C LCD dan ukuran (16 kolom x 2 baris)

// Mendefinisikan pin untuk sensor ultrasonik
const int trigPin = 7;
const int echoPin = 6;
const int distanceThreshold = 10; // Jarak ambang batas dalam cm

// Fungsi untuk menghitung inverse kinematics
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

    // Menampilkan sudut yang disesuaikan di LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    char buffer[7];
    dtostrf(theta1, 6, 2, buffer);
    lcd.print("Theta1: ");
    lcd.print(buffer);

    lcd.setCursor(0, 1);
    dtostrf(theta2, 6, 2, buffer);
    lcd.print("Theta2: ");
    lcd.print(buffer);
}

// Fungsi untuk menggerakkan servo ke sudut target
void moveServo(Servo &servo, int targetAngle, int &lastAngle) {
    if (targetAngle != lastAngle) {
        int currentAngle = servo.read();
        Serial.print("Current Angle: ");
        Serial.println(currentAngle);
        Serial.print("Target Angle: ");
        Serial.println(targetAngle);

        if (currentAngle < targetAngle) {
            for (int angle = currentAngle; angle <= targetAngle; angle++) {
                servo.write(angle);
                delay(stepDelay); // Tidak ada delay untuk responsivitas tinggi
                Serial.print("Moving Angle: ");
                Serial.println(angle);
            }
        } else {
            for (int angle = currentAngle; angle >= targetAngle; angle--) {
                servo.write(angle);
                delay(stepDelay); // Tidak ada delay untuk responsivitas tinggi
                Serial.print("Moving Angle: ");
                Serial.println(angle);
            }
        }

        lastAngle = targetAngle;
    }
}

int lastTheta1 = -1; // Untuk menyimpan sudut target terakhir servo1
int lastTheta2 = -1; // Untuk menyimpan sudut target terakhir servo2

// Fungsi untuk membaca jarak dari sensor ultrasonik
long readUltrasonicDistance(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = (duration / 2) / 29.1; // Konversi ke cm
    return distance;
}

void setup() {
    Serial.begin(9600);

    // Inisialisasi LCD
    lcd.init();
    lcd.backlight();

    // Menyambungkan servo ke pin
    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);

    // Inisialisasi pin sensor ultrasonik
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

// Fungsi untuk membaca jarak penghalang dengan sensor
void loop() {
    long distance = readUltrasonicDistance(trigPin, echoPin);
    Serial.print("Distance: ");
    Serial.println(distance);

    for (int i = 0; i < sizeof(xTargets) / sizeof(xTargets[0]); i++) {
        float x = xTargets[i];
        float y = yTargets[i];
        float targetDistance = sqrt(x * x + y * y);
        float threshold = 1.0; // Ambang toleransi untuk kedekatan target

        // Memeriksa apakah objek berada di dekat titik target
        if (abs(distance - targetDistance) <= threshold) {
            float theta1, theta2;
            calculateInverseKinematics(x, y, theta1, theta2);

            Serial.print("Moving to target: (");
            Serial.print(x);
            Serial.print(", ");
            Serial.print(y);
            Serial.println(")");

            moveServo(servo1, theta1, lastTheta1);
            moveServo(servo2, theta2, lastTheta2);

            break; // Keluar dari loop setelah menemukan target yang cocok
        }
    }
}
