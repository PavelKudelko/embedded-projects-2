#include <Wire.h>
#include <LIDARLite.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "DFRobot_TCS34725.h"

LIDARLite myLidarLite;
LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
// compass address
const int ADDRESS = 0x60;
// joystick pins
const int joyX = A8;
const int joyY = A9;
const int joyButton = 19;
// pulse count per cm (found by experiments)
const int PULSE_AVG = 13.2;
// encoder pins for pulse counting
#define ENCA_R 2
#define ENCA_L 3
volatile int pulseCountR = 0;
volatile int pulseCountL = 0;
unsigned long startTime = 0;
volatile bool stop_motors = false;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const int DEBOUNCE = 200;
// motors
#define Motor_forward         1
#define Motor_return          0
#define Motor_L_dir_pin      7
#define Motor_R_dir_pin      8
#define Motor_L_pwm_pin      9
#define Motor_R_pwm_pin      10

const int potPin = A1;

// offset for getting real values
volatile int COMPASS_READING_OFFSET = 0;

const int LIDAR_SAMPLES = 30;
int lidar_vals[LIDAR_SAMPLES] = {0};
int curIndx = 0;

// length of the car
const int ROBOT_LENGTH = 20;

//lidar correction
const int LIDAR_CORR_VAL = 10;

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;

void encoderISR() {
  pulseCountR++;
}

void encoderISRleft() {
  pulseCountL++;
}
// compass reading handling
uint8_t readCompassBearing() {
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x01);
  Wire.endTransmission(false);

  Wire.requestFrom(ADDRESS, 1, true);
  if (Wire.available()) {
    return Wire.read();
  }
  // If connection fails, return 0
  return 0;
}

// Calibrate compass
void calibrateCompass() {
  uint8_t initVal = readCompassBearing();
  if (initVal <= 127) {
    COMPASS_READING_OFFSET = -initVal; // Offset for values closer to 0
  } else {
    COMPASS_READING_OFFSET = 255 - initVal; // Offset for values closer to 255
  }
  Serial.print("Compass Calibration Offset: ");
  Serial.println(COMPASS_READING_OFFSET);
}

// Get corrected compass bearing in degrees
int getCorrectedCompassBearing() {
  int rawBearing = readCompassBearing();
  int correctedRaw = (rawBearing + COMPASS_READING_OFFSET) % 255; // Apply offset in 0–255 range
  if (correctedRaw < 0) {
    correctedRaw += 255; // Ensure value is non-negative
  }
  // Convert to degrees
  float correctedDegrees = (correctedRaw * 360.0) / 255.0;
  return static_cast<int>(correctedDegrees); // Return as integer
}
int avg_lidar_val(int vals[], int size) {
  int sum = 0;
  for (int i = 0; i < size; i++) {
    sum += vals[i];
  }
  return sum / size;
}

DFRobot_TCS34725 tcs = DFRobot_TCS34725(&Wire, TCS34725_ADDRESS, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  delay(2000);
  Serial.begin(9600);
  Serial1.begin(9600);
  // define second serial monitor
  // Serial2.begin(9600);
  while (!Serial && !Serial1) {
    ; // wait for serial port to connect
  }
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLidarLite.configure(0); // Change this number to try out alternate configurations

  pinMode(ENCA_R, INPUT);
  pinMode(ENCA_L, INPUT);
  // motors setup
  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);
  // potentialmeter setup
  pinMode(potPin, INPUT);
  // ISR functions (pulse count for L and R and joy button)
  attachInterrupt(digitalPinToInterrupt(ENCA_R), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), encoderISRleft, RISING);
  attachInterrupt(digitalPinToInterrupt(joyButton), buttonISR, FALLING);
  pinMode(joyButton, INPUT_PULLUP);
  lcd.begin(20, 4);
  Wire.begin();
  // Compass calibration
  calibrateCompass();
  Serial.begin(115200);
  Serial.println("Color View Test!");

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void buttonISR() {
  static unsigned long lastDebounceTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > DEBOUNCE) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
    lcd.clear();
  }
}

void loop() {
  
  checkRGBsensor();

  handleSerialControl();

  if (buttonPressed) {
    stopMotors();
    buttonPressed = false;
  }
  int distance = get_dist();
  Serial1.println("LIDAR:" + String(distance));
  Serial1.println("COMPASS:" + String(getCorrectedCompassBearing()));

  if (distance <= 10) {
    Serial1.println("WARNING");
  }
  delay(150); // Delay to control update frequency
}

void checkRGBsensor() {
  uint16_t clear, red, green, blue;
  tcs.getRGBC(&red, &green, &blue, &clear);
  tcs.lock(); // turn off LED

  // Serial.print("C: "); Serial.print(clear);
  // Serial.print("\tR: "); Serial.print(red);
  // Serial.print("\tG: "); Serial.print(green);
  // Serial.print("\tB: "); Serial.println(blue);

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  // Serial.print("\t");
  // Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  // Serial.println();

  // Serial.print((int)r);
  // Serial.print(" ");
  // Serial.print((int)g);
  // Serial.print(" ");
  // Serial.println((int)b);
  Serial1.println("RGB:" + String((int)r) + ";" + String((int)g) + ";" + String((int)b));
}

void displayLidarValues() {
  int distance = get_dist();
  lcd.setCursor(0, 3);
  lcd.print("LIDAR: ");
  lcd.print(distance);
  lcd.print(" cm  "); 
}


void turnExact(int angle, String direction = "") {
  if (angle == 0) return;  // No turn needed
  int startHeading = getCorrectedCompassBearing();
  int currentHeading = startHeading;
  int accumulatedAngle = 0;
  // decide turn dir. If no str provided decide dir by the closest
  bool isRightTurn = (direction == "") ? (angle > 0) : (direction == "right");  int targetAngle = abs(angle);
  // Loop until the accumulated angle reaches the target angle
  while (accumulatedAngle < targetAngle) {
    int newHeading = getCorrectedCompassBearing();
    int deltaAngle = newHeading - currentHeading;

    if (deltaAngle > 180) {
      deltaAngle -= 360;
    } else if (deltaAngle < -180) {
      deltaAngle += 360;
    }

    if (isRightTurn && deltaAngle > 0) {
      accumulatedAngle += deltaAngle;
    } else if (!isRightTurn && deltaAngle < 0) {
      accumulatedAngle -= deltaAngle;
    }

    currentHeading = newHeading;
    if (isRightTurn) {
      turnRight(40); 
    } else {
      turnLeft(40);
    }
    delay(50);
  }

  stopMotors(); // Stop the motors after the turn
}

void handleSerialControl() {
  // Check if there is incoming data from the serial monitor
  if (Serial.available() > 0) {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');
    //Serial.print("Message received, content: ");
    //Serial.println(message);

    int pos_drive_dist = message.indexOf("Move");
    int pos_turn = message.indexOf("Turn");

    if (pos_drive_dist > -1) {
      Serial.println("Command = Move");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = Move");
      pos_drive_dist = message.indexOf(":");

      if (pos_drive_dist > -1) {
        // Extract the distance
        String stat = message.substring(pos_drive_dist + 1);
        int stat_int = stat.toInt();
        // some error handling
        if (stat_int <= 51 && stat_int >= -51) {
          lcd.print(stat_int);
          move(stat_int, 100);
        } else {
          lcd.setCursor(0, 1);
          lcd.print("Error: invalid value");
        }
      }
      else {
        lcd.setCursor(0, 1);
        lcd.print("Error: missing value");
      }
    }
    // Handle Measure command (no parameters)
    else if (pos_turn > -1) {
      Serial.print("Command = Turn ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = Turn ");
      pos_turn = message.indexOf(":");

      if (pos_turn > -1) {
        String stat = message.substring(pos_turn + 1); // Extract the angle
        // angle str to int convert
        int stat_int = stat.toInt();
        // Since it's a slide bar on the web page (0 to 360)
        // validating angle and error handling is not needed here
        lcd.print(stat_int);
        turnExact(stat_int);
      } else {
        lcd.setCursor(0, 1);
        lcd.print("Error: missing angle");
      }
    }
    // Handle invalid or unrecognized commands
    else {
     lcd.setCursor(0, 0);
     lcd.print("Error: unknown cmd");
    }
  }
}

void move(int cm, int speedPercent) {
  int distance = get_dist();
  
  // If initially blocked, try to find clear path
  if (distance < 10) {
    int rotations = 0;
    bool clearPathFound = false;

    Serial1.println("WARNING");
    
    // Try up to 4 times
    while (rotations < 4 && !clearPathFound) {
      turnExact(90, "right");
      rotations++;
      
      distance = get_dist();
      if (distance >= 10) {
        clearPathFound = true;
        break;
      }
    }
    
    if (!clearPathFound) {
      Serial.println("Surrounded by obstacles - cannot move");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Surrounded by obstacles - cannot move");
      // send message to esp
      //Serial1.println("CANT-MOVE");
      return;
    }
  }
  
  // Get fresh distance reading after any rotations
  distance = get_dist();
  
  // Calculate target distance based on direction
  int targetDist;
  if (cm > 0) {
    // Moving forward: current distance should decrease
    targetDist = distance - cm;
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_forward);
  } else {
    // Moving backward: current distance should increase
    targetDist = distance - cm;  // Note: -cm makes it positive
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_return);
  }
  
  // Continue moving until we reach target distance
  while (true) {
    distance = get_dist();
    
    // Check if we've reached target
    if (cm > 0 && distance <= targetDist) break;
    if (cm < 0 && distance >= targetDist) break; 
    
    // Emergency stop if obstacle is too close
    if (distance < 10) {
      stopMotors();
      Serial.println("Emergency stop - obstacle detected");
      // Serial1.println("EMERGENCY-STOP");
      return;
    }
    
    // Adjust speed based on obstacle proximity
    int adjustedSpeed;
    if (distance < 30) {
      // Slow speed (30%) when obstacles are nearby
      adjustedSpeed = map(30, 0, 100, 0, 255);
      Serial.println("Obstacle nearby - reducing speed");
      if (distance <= 10) {
        Serial1.println("WARNING");
      }
    } else {
      // Fast speed (70%) when path is clear
      adjustedSpeed = map(70, 0, 100, 0, 255);
    }
    
    // Apply the adjusted speed
    analogWrite(Motor_L_pwm_pin, adjustedSpeed);
    analogWrite(Motor_R_pwm_pin, adjustedSpeed);
    
    // Small delay to allow for LIDAR readings to update
    delay(50);
  }
  
  stopMotors();
}

int get_dist() {
  for (int i = 0; i < LIDAR_SAMPLES; i++) {
    lidar_vals[i] = myLidarLite.distance() - LIDAR_CORR_VAL;
  }

  return avg_lidar_val(lidar_vals, LIDAR_SAMPLES); // Calculate average
}

void turnLeft(int speedPercent) {
  int pwmValue = map(speedPercent, 0, 100, 0, 255);
  digitalWrite(Motor_L_dir_pin, Motor_forward);
  digitalWrite(Motor_R_dir_pin, Motor_return);
  analogWrite(Motor_L_pwm_pin, pwmValue);
  analogWrite(Motor_R_pwm_pin, pwmValue);
}

void turnRight(int speedPercent) {
  int pwmValue = map(speedPercent, 0, 100, 0, 255);
  digitalWrite(Motor_R_dir_pin, Motor_forward);
  digitalWrite(Motor_L_dir_pin, Motor_return);
  analogWrite(Motor_L_pwm_pin, pwmValue);
  analogWrite(Motor_R_pwm_pin, pwmValue);
}

void drive(int speed, bool direction) {
  int pwmValue = map(speed, 0, 100, 0, 255);
  if (direction) {
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_forward);

    analogWrite(Motor_L_pwm_pin, pwmValue);
    analogWrite(Motor_R_pwm_pin, pwmValue);
  }
  else {
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_return);

    analogWrite(Motor_L_pwm_pin, pwmValue);
    analogWrite(Motor_R_pwm_pin, pwmValue);
  }
}

void stopMotors() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}