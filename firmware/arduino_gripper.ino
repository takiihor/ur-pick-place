#include <Adafruit_NeoPixel.h>
#include <Servo.h>


#define RS485_CONTROL 5


#define BUTTON_D2     2
#define ANALOG_A0     A0


#define NEOPIXEL_PIN  A3
#define NUM_LEDS      3


Adafruit_NeoPixel pixels(NUM_LEDS, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);

const uint8_t SERVO1_PIN = 10;
const uint8_t SERVO2_PIN = 11;
const uint8_t FSR_1_PIN  = A2;
const uint8_t FSR_2_PIN  = A1;

Servo servo1;
Servo servo2;

bool Servoinit = 0;

byte servo1pos = 45;
byte servo2pos = 165;

void setup() {
  pinMode(BUTTON_D2, INPUT_PULLUP);
  pinMode(ANALOG_A0, INPUT_PULLUP);

  pinMode(FSR_1_PIN, INPUT);
  pinMode(FSR_2_PIN, INPUT);

  delay(500);


  pixels.begin();
  pixels.setBrightness(50);
  setAllLedsColor(150, 0, 150);


  pinMode(RS485_CONTROL, OUTPUT);
  digitalWrite(RS485_CONTROL, LOW);


  Serial.begin(9600);


}

void Servoinitial() {
  if (Servoinit == 0) {
    servo1.attach(SERVO1_PIN, 544, 2400);
    servo2.attach(SERVO2_PIN, 544, 2400);
    Servoinit = 1;
  }
  servo1.write(servo1pos);
  delay(100);
  servo2.write(servo2pos);
}



void Grab1() {
  setAllLedsColor(155, 0, 0);
  servo1pos = 115;
  servo2pos = 90;
  Servoinitial();

  sendRS485ASCII("Tight\n");
}

void Grab2() {
  setAllLedsColor(0, 155, 0);
  servo1pos = 90;
  servo2pos = 123;
  Servoinitial();

  sendRS485ASCII("Loose\n");
}

void Grab3() {
  setAllLedsColor(0, 0, 255);
  servo1pos = 40;
  servo2pos = 165;
  Servoinitial();

  sendRS485ASCII("Open\n");
}

void PadsPressure() {
  int FSR1 = analogRead(FSR_1_PIN);
  int FSR2 = analogRead(FSR_2_PIN);
  delay(10);
  FSR1 = FSR1 + analogRead(FSR_1_PIN);
  FSR2 = FSR2 + analogRead(FSR_2_PIN);
  delay(10);
  FSR1 = FSR1 + analogRead(FSR_1_PIN);
  FSR2 = FSR2 + analogRead(FSR_2_PIN);

  String msgToSend = "FSR1:" + String(FSR1) + " / FSR2:" + String(FSR2) + "\n";
  sendRS485ASCII(msgToSend);
}

void setAllLedsColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}


void sendRS485ASCII(String msg) {
  digitalWrite(RS485_CONTROL, HIGH);
  delayMicroseconds(50);
  Serial.print(msg);
  Serial.flush();
  delay(1);
  digitalWrite(RS485_CONTROL, LOW);
}

void loop() {

  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();

    switch (receivedData.toInt()){
      case 1:
        Grab1();
        break;
      case 2:
        Grab2();
        break;
      case 3:
        Grab3();
        break;
      case 0:
        PadsPressure();
        break;
      default:
        setAllLedsColor(155, 25, 234);
        break;
    }
  }
}
