#include <LiquidCrystal.h>
#include <SoftwareSerial.h>


#define ARDUINO_RX 10 //TX of the Serial MP3 Player module
#define ARDUINO_TX 11 //RX of the Serial MP3 Player module

SoftwareSerial mySerial(ARDUINO_RX, ARDUINO_TX);  //initialize software serial object

//create arrays to store commands/data for each MP3 function
byte selectSD[8] = {0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xEF};
byte setVol[8] = {0x7E, 0xFF, 0x06, 0x06, 0x00, 0x00, 0x1E, 0xEF};
byte playTrack[8] = {0x7E, 0xFF, 0x06, 0x17, 0x00, 0x01, 0x03, 0xEF};

int leftForward = 3;
int leftBackward = 2;
int rightForward = 4;
int rightBackward = 5;
int redLED = 12;
int blueLED = 13;
int rightSensor = 7;
int leftSensor = 8;
int soundSensor = 6;
int straight = 1;
int right = 1;
int left = 1;
int back = 1;


void setup()
{
  Serial.begin(9600);
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(redLED, OUTPUT); //red LED
  pinMode(blueLED, OUTPUT); //blue LED
  pinMode(A4, OUTPUT); //right taillight
  pinMode(A5, OUTPUT); //left taillight

  mySerial.begin(9600);
  for (uint8_t i = 0; i < 8; i++) {
    mySerial.write(selectSD[i]);     //select SD card storage
    //Serial.println(selectSD[i]);  //for debugging, output array values to Serial monitor
  }

  for (uint8_t i = 0; i < 8; i++) {
    mySerial.write(setVol[i]);      //set volume ($1E (decimal 30) = MAX)
  }

  for (uint8_t i = 0; i < 8; i++) { //play track 002 in folder 01/
    mySerial.write(playTrack[i]);
  }
}

void driveForward() {
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);
}

void driveBackward() {
  digitalWrite(A4, HIGH);
  digitalWrite(A5, HIGH);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightBackward, HIGH);
}

void turnLeft() {
  digitalWrite(A5, HIGH);
  digitalWrite(A4, LOW);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightBackward, HIGH);
}

void turnRight() {
  digitalWrite(A5, LOW);
  digitalWrite(A4, HIGH);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightBackward, LOW);
}

void loop()
{

  if (digitalRead(soundSensor) == 1) {
    digitalWrite(redLED, HIGH);
    digitalWrite(blueLED, HIGH);
  }
  else if (digitalRead(soundSensor) == 0) {
    digitalWrite(redLED, LOW);
    digitalWrite(blueLED, LOW);
  }
  // IF THE SENSOR RETURNS 1 IT MEANS IT IS ON BLACK

  if (digitalRead (leftSensor) == 1 && digitalRead(rightSensor) == 1) {
    driveForward();
  }
  else if (digitalRead (leftSensor) == 1 && digitalRead(rightSensor) == 0) {
    turnRight();
  }
  else if (digitalRead (leftSensor) == 0 && digitalRead(rightSensor) == 1) {
    turnLeft();
  }
  else if (digitalRead (leftSensor) == 0 && digitalRead(rightSensor) == 0) {
    driveBackward();
  }
}
