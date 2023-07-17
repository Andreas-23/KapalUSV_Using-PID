#include <ESP32Servo.h>
#include <Adafruit_GFX.h>      //memasukkan library oled dalam sketch
#include <Adafruit_SSD1306.h>  //memasukkan library oled 128x64 dalam sketch

#define SCREEN_WIDTH 128                                                   //menetapkan lebar oled
#define SCREEN_HEIGHT 64                                                   //menetapkan tinggi oled
#define OLED_RESET -1                                                      //menetapkan tampilan oled secara berurutan
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  //menginisialisasi oled
int16_t x, y;                                                              //menginisialisasi letak
uint16_t w, h;                                                             //menginisialisasi letak

#define TRIG_PIN1 15
#define ECHO_PIN1 2
#define TRIG_PIN2 17
#define ECHO_PIN2 16
#define TRIG_PIN3 13
#define ECHO_PIN3 12

//motor driver
#define motor1Pin1 27  //menetapkan pin motor driver1
#define motor1Pin2 26  //menetapkan pin motor driver2
// SET DUTYCYCLE      0   = 0 %   64  = 25 %   127 = 50 %   191 = 75 %     255 = 100 %*/
int kecepatan = 50;  //menginisialisasi kecepatan lurus, motor driver

Servo servo;
#define servoPin 4
// Inisialisasi sudut posisi servo
const int servoMiddle = 90;
const int servoLeft = 180;
const int servoRight = 0;

int interval = 15;

const int targetDistance = 10;  // Jarak target yang diinginkan
const float Kp = 0.5;           // Konstanta proporsional
const float Ki = 0.2;           // Konstanta integral
const float Kd = 0.1;           // Konstanta diferensial

int previousError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);

  //OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //begin sd card
  display.clearDisplay();                     //HAPUS OLET BAWAAN
    //ROLEPLAY DISPLAY
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.getTextBounds("START", 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.println("START");

  display.getTextBounds("MOTOR", 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 15);
  display.println("MOTOR");

    display.setTextSize(1);
  display.setTextColor(WHITE);
  display.getTextBounds("Anggarudin", 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 35);
  display.println("Anggarudin");

    display.getTextBounds("Andreas", 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 45);
  display.println("Andreas");

      display.getTextBounds("Angel", 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 55);
  display.println("Angel");

  display.display();
  delay(2500);
  display.clearDisplay();

  // Inisialisasi servo
  servo.attach(servoPin);

  //GERAKKAN SERVO
  Serial.println("30");
  servo.write(30);
  delay(1000);
  Serial.println("90");
  servo.write(90);
  delay(1000);
  Serial.println("170");
  servo.write(170);
  delay(1000);
  Serial.println("90");
  servo.write(90);
  delay(1000);

  //ROLEPLAY DISPLAY II
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.getTextBounds("MOTOR", 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.println("MOTOR");

  display.getTextBounds("READY", 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 25);
  display.println("READY");

  display.display();
  delay(1000);
  display.clearDisplay();

  //Motor Control
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  digitalWrite(motor1Pin1, HIGH);  //Motor dc
}

void loop() {
  long duration, sensorkanan, sensorkiri, sensorlurus;
  //Motor
  analogWrite(27, kecepatan);
  analogWrite(26, 0);

  // Sensor 1
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);
  duration = pulseIn(ECHO_PIN1, HIGH);
  sensorkanan = duration * 0.034 / 2;

  // Sensor 2
  digitalWrite(TRIG_PIN2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);
  duration = pulseIn(ECHO_PIN2, HIGH);
  sensorlurus = duration * 0.034 / 2;

  // Sensor 3
  digitalWrite(TRIG_PIN3, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN3, LOW);
  duration = pulseIn(ECHO_PIN3, HIGH);
  sensorkiri = duration * 0.034 / 2;

  int error = targetDistance - sensorlurus;  // Perhitungan error

  // Pengendali PID
  float output = Kp * error + Ki * integral + Kd * (error - previousError);

  // Batasan output
  if (output > 180) {
    output = 180;
  } else if (output < 0) {
    output = 0;
  }

  display.setCursor(0, 15);  //oled display
  display.print("S R = ");
  display.print(sensorkanan);
  display.print(" cm");
  display.display();

  display.setCursor(0, 25);  //oled display
  display.print(" S M = ");
  display.print(sensorlurus);
  display.print(" cm");
  display.display();

  display.setCursor(0, 35);  //oled display
  display.print(" S L = ");
  display.print(sensorkiri);
  display.println(" cm");
  display.display();

    display.setCursor(0, 45);  //oled display
  display.print(" Error = ");
  display.println(error);
  display.display();

      display.setCursor(0, 55);  //oled display
  display.print(" Output = ");
  display.println(output);
  display.display();

  // Logika gerakan servo berdasarkan jarak sensor
  if (sensorkanan < interval && sensorkiri > interval && sensorlurus > interval) {
    servo.write(servoLeft);  // Putar ke kiri (posisi 180)
    Serial.print("KIRI<<  ");
    display.getTextBounds("BELOK KE KIRI", 0, 0, &x, &y, &w, &h);
    display.setTextSize(1.5);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println("BELOK KE KIRI");
    display.display();
    display.clearDisplay();
  } else if (sensorkanan > interval && sensorkiri < interval && sensorlurus > interval) {
    servo.write(servoRight);  // Putar ke kanan (posisi 0)
    Serial.print("KANAN>> ");
    display.getTextBounds("BELOK KE KANAN", 0, 0, &x, &y, &w, &h);
    display.setTextSize(1.5);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println("BELOK KE KANAN");
    display.display();
    display.clearDisplay();
  } else if (sensorkanan < interval && sensorkiri > interval && sensorlurus < interval) {
    servo.write(servoLeft);  // Putar ke kiri (posisi 180)
    Serial.print("KIRI<<  ");
    display.getTextBounds("BELOK KE KIRI", 0, 0, &x, &y, &w, &h);
    display.setTextSize(1.5);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println("BELOK KE KIRI");
    display.display();
    display.clearDisplay();
  } else if (sensorkanan > interval && sensorkiri < interval && sensorlurus < interval) {
    servo.write(servoRight);  // Putar ke kanan (posisi 0)
    Serial.print("KANAN>> ");
    display.getTextBounds("BELOK KE KANAN", 0, 0, &x, &y, &w, &h);
    display.setTextSize(1.5);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println("BELOK KE KANAN");
    display.display();
    display.clearDisplay();
  } else {
    servo.write(servoMiddle);  // Tetap posisi tengah (posisi 90)
    Serial.print(">>LURUS<< ");
    display.getTextBounds("LURUS", 0, 0, &x, &y, &w, &h);
    display.setTextSize(1.5);
    display.setCursor((SCREEN_WIDTH - w) / 2, 0);
    display.println("LURUS");
    display.display();
    display.clearDisplay();
  }

  delay(100);

  // Print the distances to Serial Monitor
  Serial.print("Sensor 1 = ");
  Serial.print(sensorkanan);
  Serial.print(" cm || Sensor 2 = ");
  Serial.print(sensorlurus);
  Serial.print(" cm || Sensor 3 = ");
  Serial.print(sensorkiri);
  Serial.print(" cm || Error = ");
  Serial.print(error);
  Serial.print(" || Output = ");
  Serial.println(output);

  // Update nilai error dan integral
  previousError = error;
  integral += error;

  delay(1000);
}