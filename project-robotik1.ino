#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Inisialisasi alamat I2C LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Alamat I2C umum untuk modul 16x2

// Definisikan pin untuk sensor ultrasonik
const int trigPin = 9;
const int echoPin = 10;

#include <Servo.h>

// Inisialisasi objek servo
Servo myservo;

// Pin yang digunakan untuk mengendalikan servo
const int servoPin = 2;

void setup() {
  // Attach servo ke pin yang ditentukan
  myservo.attach(servoPin);

  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();

  // Inisialisasi pin sensor ultrasonik
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Mula-mula, LCD akan menampilkan pesan inisialisasi
  lcd.print("Ultrasonik");
  lcd.setCursor(0, 1);
  lcd.print("Sensor...");
  delay(2000);
  lcd.clear();
}

void loop() {
  // Mengirimkan pulse ultrasonik
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Menerima pulsa kembali dan menghitung jarak
  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0343 / 2;  // Waktu perjalanan pulsa dikalikan dengan kecepatan suara (34 cm/ms), dibagi dua karena perjalanan pulsa bolak-balik

  // Menampilkan jarak pada LCD
  lcd.clear();
  lcd.print("Jarak: ");
  lcd.print(distance);
  lcd.print(" cm");
  delay(1000);  // Delay untuk memperbarui layar
  
  // Sweeping dari 0 derajat ke 180 derajat
  for (int pos = 0; pos <= 180; pos += 1) {
    // Gerakkan servo ke posisi saat ini
    myservo.write(pos);
    delay(15); // Tunggu sedikit agar servo mencapai posisi yang diinginkan
  }
  
  // Sweeping dari 180 derajat ke 0 derajat
  for (int pos = 180; pos >= 0; pos -= 1) {
    // Gerakkan servo ke posisi saat ini
    myservo.write(pos);
    delay(15); // Tunggu sedikit agar servo mencapai posisi yang diinginkan
  }

}
