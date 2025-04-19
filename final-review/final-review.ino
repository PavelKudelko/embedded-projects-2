#include <Wire.h>
#include <LIDARLite.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "DFRobot_TCS34725.h"

LIDARLite myLidarLite;
LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
// compass address
const int ADDRESS = 0x60;
// compass offset
const int OFFSET = 2;
// joystick pins
// const int joyX = A8;
// const int joyY = A9;
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
int buttonPressCount = 0;
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

struct RGBColor {
  int red;
  int green;
  int blue;
};

RGBColor FIRST_COLOR;
RGBColor SECOND_COLOR;
RGBColor THIRD_COLOR;
RGBColor FOURTH_COLOR;
RGBColor FIFTH_COLOR;

String CURRENT_COLOR = "";

bool colorIsSet[4] = {false, false, false, false};

// lidar measurments
int startNorth = 0, startEast = 0, startSouth = 0, startWest = 0;
int endNorth = 0, endEast = 0, endSouth = 0, endWest = 0;

enum Dir { NORTH, EAST, SOUTH, WEST };

Dir currentDirection = NORTH;


int consecutiveLeftCalls = 0;
int consecutive180Calls = 0;

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
    // wait for serial port to connect
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
  //Serial.begin(115200);
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
    //lcd.clear();

    buttonPressCount ++;
  }
}

void loop() {

  // checkRGBsensor();
  handleSerialControl();
  int distance = get_dist();
  // print to esp
  // Serial1.println("LIDAR:" + String(distance));
  // Serial1.println("COMPASS:" + String(getCorrectedCompassBearing()));
  // // print to serial monitor
  // Serial.println("LIDAR:" + String(distance));
  // Serial.println("COMPASS:" + String(getCorrectedCompassBearing()));

  if (!areColorsSet()) {
    setColors();
  }
  
  if (areColorsSet()) {
    detectColors();
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calibrate colors");
    lcd.setCursor(0, 1);
    lcd.print("Btn Count: " + String(buttonPressCount));
  }
  
  // if (distance <= 10) {
  //   Serial1.println("WARNING");
  //   // Debug obstacle info
  //   lcd.clear();
  //   lcd.setCursor(0, 0);
  //   lcd.print("OBSTACLE NEAR!");
  //   lcd.setCursor(0, 1);
  //   lcd.print("Dist: " + String(distance) + "cm");
  // }
  
}

void setColors() {
    if (!areColorsSet()) {
    switch (buttonPressCount) {
      case 1:
        break;
      case 2:
        if (!colorIsSet[0]) calibrateColor(1);
        break;
      case 3:
        if (!colorIsSet[1]) calibrateColor(2);
        break;
      case 4:
        if (!colorIsSet[2]) calibrateColor(3);
        break;
      case 5:
        if (!colorIsSet[3]) calibrateColor(4);
        break;
      default:
        buttonPressCount = 0;
        break;
    }
  }
}

void detectColors() {
  RGBColor currColor = checkRGBsensor();
  
  if (!areColorsSet()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error: Colors not");
    lcd.setCursor(0, 1);
    lcd.print("calibrated!");
    return;
  }
  
  // Calculate color distance between current color and each calibrated color
  int dist1 = colorDistance(currColor, FIRST_COLOR);
  int dist2 = colorDistance(currColor, SECOND_COLOR);
  int dist3 = colorDistance(currColor, THIRD_COLOR);
  int dist4 = colorDistance(currColor, FOURTH_COLOR);
  
  // // Debug RGB values
  // Serial.println("Current RGB: " + String(currColor.red) + "," + 
  //                String(currColor.green) + "," + String(currColor.blue));
  
  // Find the minimum distance
  int minDist = dist1;
  int closestColor = 1;
  String colorName = "Color 1";
  
  if (dist2 < minDist) {
    minDist = dist2;
    closestColor = 2;
    colorName = "Color 2";
  }
  
  if (dist3 < minDist) {
    minDist = dist3;
    closestColor = 3;
    colorName = "Color 3";
  }
  
  if (dist4 < minDist) {
    minDist = dist4;
    closestColor = 4;
    colorName = "Color 4";
  }
  CURRENT_COLOR = colorName;

  // Display the color name on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Detected: ");
  lcd.print(colorName);
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(minDist);
}


int colorDistance(const RGBColor& color1, const RGBColor& color2) {
  int rDiff = color1.red - color2.red;
  int gDiff = color1.green - color2.green;
  int bDiff = color1.blue - color2.blue;
  
  // Using Euclidean distance
  return rDiff*rDiff + gDiff*gDiff + bDiff*bDiff;
}

bool areColorsSet() {
  // Serial.print("Colors Set: ");
  // Serial.print(colorIsSet[0]); Serial1.print(" ");
  // Serial.print(colorIsSet[1]); Serial1.print(" ");
  // Serial.print(colorIsSet[2]); Serial1.print(" ");
  // Serial.println(colorIsSet[3]);
  return colorIsSet[0] && colorIsSet[1] && colorIsSet[2] && colorIsSet[3];
}

void calibrateColor(int numOfColor) {
  if (numOfColor == 1) {
    FIRST_COLOR = checkRGBsensor();
    colorIsSet[0] = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("COLOR 1 SET");
    delay(500); 
  }
  else if (numOfColor == 2) { 
    SECOND_COLOR = checkRGBsensor();
    colorIsSet[1] = true; 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("COLOR 2 SET");
    delay(500);
  }
  else if (numOfColor == 3) { 
    THIRD_COLOR = checkRGBsensor();
    colorIsSet[2] = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("COLOR 3 SET");
    delay(500);
  }
  else if (numOfColor == 4) { 
    FOURTH_COLOR = checkRGBsensor();
    colorIsSet[3] = true; 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("COLOR 4 SET");
    delay(500);
  }
}

RGBColor checkRGBsensor() {
  uint16_t clear, red, green, blue;
  tcs.getRGBC(&red, &green, &blue, &clear);
  tcs.lock(); // turn off LED
  
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  
  RGBColor color;
  color.red = (int)r;
  color.green = (int)g;
  color.blue = (int)b;
  
  return color;
}

void displayLidarValues() {
  int distance = get_dist();
  lcd.setCursor(0, 3);
  lcd.print("LIDAR: ");
  lcd.print(distance);
  lcd.print(" cm  "); 
}

void turnExact(int angle, String direction = "") {
  if (angle == 0) return; // No turn needed
  
  int startHeading = getCorrectedCompassBearing();
  int currentHeading = startHeading;
  int accumulatedAngle = 0;
  
  // Decide turn direction
  bool isRightTurn;
  if (direction == "") {
    // If no direction specified, use the sign of angle
    isRightTurn = (angle > 0);
  } else {
    // Explicitly use the provided direction
    isRightTurn = (direction == "right");
  }
  
  int targetAngle = abs(angle);
  
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
  
  stopMotors();
}

void handleSerialControl() {
  // Check if there is incoming data from the serial monitor
  if (Serial.available() > 0) {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');

    int pos_drive_goal = message.indexOf("DriveGoal");

    if (pos_drive_goal > -1) {
      Serial.print("Command = DriveGoal ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = DriveGoal ");

      driveGoal();
    }
    else if (message.indexOf("calibrateEncoder") > -1 ) {
      Serial.println("Command = calibrateEncoder");

      calibrateEncoder();
    }
    else if (message.indexOf("getEEPROM") > -1 ) {
      Serial.println("Command = getEEPROM");
      
      getEEPROM();
    }
    else if (message.indexOf("measureStart") > -1) {
      Serial.println("Command = measureStart");

      measureStart();
    }
    else if (message.indexOf("measureEnd") > -1) {
      Serial.println("Command = measureEnd");

      measureEnd();
    }
    else if (message.indexOf("calibrateCompass") > -1) {
      Serial.println("Command = calibrateCompass");

      calibrateCompass();
    }
    else if (message.indexOf("followWall") > -1 ) {
      Serial.println("Command = followWall");

      followWall(true);
    }
    // Handle invalid or unrecognized commands
    else {
     lcd.setCursor(0, 0);
     lcd.print("Error: unknown cmd");
    }
  }
}

void findNorth () {
  find_heading(0);
}
void findSouth () {
  find_heading(180);
}
void findWest() {
  find_heading(270);
}
void findEast() {
  find_heading(90);
}

void find_heading(int targetBearing) {
  int currentHeading;

  while (true) {
    currentHeading = getCorrectedCompassBearing();
    // Calculate shortest angle difference
    float angleDifference = currentHeading - targetBearing;
    if (angleDifference > 180) angleDifference -= 360;
    else if (angleDifference < -180) angleDifference += 360;

    if (abs(angleDifference) <= OFFSET) {
      stopMotors();
      break;
    }
    // Turn in the direction of shortest path
    if (angleDifference > 0) {
      turnLeft(50);
    } else {
      turnRight(50);
    }
    delay(50);
  }
}


void measureStart() {
  if (buttonPressed) {
    buttonPressed = false;
    stopMotors();
  }

  findNorth();
  startNorth = get_dist();

  findEast();
  startEast = get_dist();

  findSouth();
  startSouth = get_dist();

  findWest();
  startWest = get_dist();

  Serial.print("Start Distances: ");
  Serial.print(startNorth); Serial.print(", ");
  Serial.print(startEast); Serial.print(", ");
  Serial.print(startSouth); Serial.print(", ");
  Serial.println(startWest);
  findNorth();
}

void measureEnd() {
  if (buttonPressed) {
    buttonPressed = false;
    stopMotors();
  }

  findNorth();
  endNorth = get_dist();

  findEast();
  endEast = get_dist();

  findSouth();
  endSouth = get_dist();

  findWest();
  endWest = get_dist();

  Serial.print("End Distances: ");
  Serial.print(endNorth); Serial.print(", ");
  Serial.print(endEast); Serial.print(", ");
  Serial.print(endSouth); Serial.print(", ");
  Serial.println(endWest);
  findNorth();
}

void getEEPROM(){
  uint8_t retrievedValue = EEPROM.read(0);
  double distPerPulse = retrievedValue / 100.0; // Convert back to double

  Serial.print("EEPROM val: ");
  Serial.println(distPerPulse);
}

void calibrateEncoder() {
  int initDist = get_dist();
  pulseCountR = 0;
  pulseCountL = 0;
  int finalDist = initDist;  

  // Drive until we have traveled 20cm
  while (abs(initDist - finalDist) < 1) { 
    drive(30, true);
    finalDist = get_dist();
  }
  stopMotors();

  Serial.println("### calibration results ###");
  Serial.print("dist travelled: ");
  Serial.println(abs(initDist - finalDist));
  Serial.print("pulse counts: ");
  Serial.print(pulseCountR);
  Serial.print("|");
  Serial.println(pulseCountL);

  double distPerPulse = 0.0;
  if (pulseCountR > 0) { // Prevent division by zero
    distPerPulse = (double)(abs(initDist - finalDist)) / pulseCountR;
  }

  Serial.print("dist per pulse: ");
  Serial.println(distPerPulse);
  Serial.println("### end of results ###");

  // Store to EEPROM
  uint8_t scaledValue = round(distPerPulse * 100);
  EEPROM.update(0, scaledValue);
  lcd.setCursor(0, 3);
  lcd.print("EEPROM val updated");
}

int getColorNumberFast() {
  // Check if colors are calibrated
  if (!areColorsSet()) {
    return -1;  // Error code for uncalibrated colors
  }
  
  // Get current color
  RGBColor currColor = checkRGBsensor();
  
  // Calculate color distances efficiently
  int dist1 = colorDistance(currColor, FIRST_COLOR);
  int dist2 = colorDistance(currColor, SECOND_COLOR);
  int dist3 = colorDistance(currColor, THIRD_COLOR);
  int dist4 = colorDistance(currColor, FOURTH_COLOR);
  
  int minDist = dist1;
  int colorNum = 1;
  
  if (dist2 < minDist) {
    minDist = dist2;
    colorNum = 2;
  }
  
  if (dist3 < minDist) {
    minDist = dist3;
    colorNum = 3;
  }
  
  if (dist4 < minDist) {
    minDist = dist4;
    colorNum = 4;
  }
  
  return colorNum;
}

bool areStartAndEndSet() {
  if (startNorth == 0 && startSouth == 0 ) {
    Serial.println("Set start point first!");
    return false;
  }
  if (endNorth == 0 && endSouth == 0) {
    Serial.println("Set end point first!");
    return false;
  }
  return true;
}

void stepForward(int speed, int ms) {
  drive(speed, true);
  delay(ms);
  stopMotors();
}

// somehow working code
void followWall(bool rightWall) {
  const int THRESH   = 25;   // what counts as “open” (cm)
  const int STEP_MS  = 200;  // how long each forward step lasts
  int speed = 50;

  findNorth();
  turnExact(90);
  while (get_dist() > 15) {
    drive(50, true);
  }

  while (true) {

    // color zones adjust speed
    int floorCol = getColorNumberFast();
    if      (floorCol == 3) speed = 35;
    else if (floorCol == 4) speed = 75;

    // define your three “peeks” relative to the north axis
    // Dir front = NORTH;
    // Dir right = rightWall ? EAST : WEST;
    // Dir left  = rightWall ? WEST : EAST;


    // 1) Right‑hand preference
    if (canMoveRight(THRESH)) {
      stepForward(speed, STEP_MS);
      continue;
    }
    // 2) Else straight
    else if (canMoveLeft(THRESH)) {
      // turnExact(2, "left");
      stepForward(speed, STEP_MS);
      continue;
    }
    // 3) Else left
    else if (canMove180(THRESH)) {
      // turnExact(2, "left");
      stepForward(speed, STEP_MS);
      continue;
    }
    else {
      turnExact(90, "left");
      stepForward(speed, STEP_MS);
      continue;
    }
  }
}

bool canMoveRight(int TH) {
  turnExact(2, "right");
  int dist = get_dist();
  int color  = getColorNumberFast();
  Serial.print("canMoveRight: ");
  Serial.print(dist);
  Serial.print(" ");
  Serial.println(color);

  return dist > TH && color != 2;
}

bool canMoveLeft(int TH) {
  turnExact(2, "left");
  int dist = get_dist();
  int color  = getColorNumberFast();
  Serial.print("canMoveLeft: ");
  Serial.print(dist);
  Serial.print(" ");
  Serial.println(color);

  if (color == 2) {
    drive(50, false);
    delay(200);
    turnExact(6, "left");
  }

  return dist > TH && color != 2;
}

bool canMove180(int TH) {
  turnExact(2, "left");
  int dist = get_dist();
  int color  = getColorNumberFast();
  Serial.print("canMove180: ");
  Serial.print(dist);
  Serial.print(" ");
  Serial.println(color);

  if (color == 2) {
    drive(50, false);
    delay(200);
    turnExact(6, "left");
  }

  return dist > TH && color != 2;
}

// not really working yet
void driveGoal() {
  // First, make sure colors are set for speed control
  if (!areColorsSet()) {
    Serial.println("Set colors first!");
    delay(3000);
    return;
  }
  // need to set coords first
  if (!areStartAndEndSet()) {
    return;
  }
  
  // We need to use the measurements stored in endNorth, endEast, endSouth, endWest
  // These should be set previously using measureEnd()
  
  // Start navigating towards the goal
  bool goalReached = false;
  int currentHeading = getCorrectedCompassBearing();
  
  while (!goalReached) {
    // First, determine our current position relative to the goal
    // by measuring in all four directions
    findNorth();
    int northDist = get_dist();
    findEast();
    int eastDist = get_dist();
    findSouth();
    int southDist = get_dist();
    findWest();
    int westDist = get_dist();
    
    // Calculate how close we are to the goal
    int northDiff = abs(northDist - endNorth);
    int eastDiff = abs(eastDist - endEast);
    int southDiff = abs(southDist - endSouth);
    int westDiff = abs(westDist - endWest);
    
    // If we're close enough in all directions, we've reached the goal
    // Define an acceptable error margin
    int errorMargin = 5; // cm
    if (northDiff <= errorMargin && eastDiff <= errorMargin && 
        southDiff <= errorMargin && westDiff <= errorMargin) {
      goalReached = true;
      break;
    }
    
    // Determine which direction to move to get closer to the goal
    // Choose the direction with the biggest difference
    int maxDiff = max(max(northDiff, eastDiff), max(southDiff, westDiff));
    
    // Check color for speed control
    int currColor = getColorNumberFast();
    int moveSpeed = 50; // Default speed 50%
    
    // Adjust speed based on color
    if (currColor == 2) moveSpeed = 35; // Blue marking - 35% speed
    else if (currColor == 3) moveSpeed = 75; // Green marking - 75% speed
    else if (currColor == 4) {
      // Red marking - virtual wall - back up and try different direction
      drive(40, false);
      delay(500);
      turnExact(90, "right");
      continue;
    }
    
    // Move in the direction that gets us closer to the goal
    if (maxDiff == northDiff) {
      if (northDist > endNorth) {
        // Need to go north
        findNorth();
        drive(moveSpeed, true);
      } else {
        // Need to go south
        findSouth();
        drive(moveSpeed, true);
      }
    } else if (maxDiff == eastDiff) {
      if (eastDist > endEast) {
        // Need to go east
        findEast();
        drive(moveSpeed, true);
      } else {
        // Need to go west
        findWest();
        drive(moveSpeed, true);
      }
    } else if (maxDiff == southDiff) {
      if (southDist > endSouth) {
        // Need to go south
        findSouth();
        drive(moveSpeed, true);
      } else {
        // Need to go north
        findNorth();
        drive(moveSpeed, true);
      }
    } else {
      if (westDist > endWest) {
        // Need to go west
        findWest();
        drive(moveSpeed, true);
      } else {
        // Need to go east
        findEast();
        drive(moveSpeed, true);
      }
    }
    
    // Drive forward and check for obstacles
    int obstacleCheck = get_dist();
    if (obstacleCheck < 15) {
      // Obstacle detected, stop and try a different direction
      stopMotors();
      turnExact(90, "right");
      continue;
    }
    
    // Move forward for a bit, then reassess
    delay(500);
    stopMotors();
  }
  
  stopMotors();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GOAL REACHED!");
  delay(2000);
}

// void driveGoal() {
//   if (!areColorsSet()) {
//     Serial.println("Set colors first!");
//     lcd.clear();
//     lcd.setCursor(0, 0);
//     lcd.print("Set colors first!");
//     delay(3000);
//     lcd.clear();
//     return;
//   }
  
//   // face north
//   findNorth();
//   int acceptable_error = 1;
  
//   // Get initial distance measurement
//   int distance = get_dist();
  
//   // decided that color4 will always be the goal color
//   while (abs(getCorrectedCompassBearing()) > 5 || abs(distance - endNorth) > acceptable_error) {
//     // Update distance measurement in each loop iteration
//     distance = get_dist();
    
//     getColorNumberFast();
//     if (getColorNumberFast() == 2) {
//       drive(35, true);
//     }
//     if (getColorNumberFast() == 3) {
//       drive(75, true);
//     }
//     if (getColorNumberFast() == 4) {
//       drive(60, false);
//       delay(450);
//       turnExact(2, "right");
//     }
    
//     //delay for stability
//     // delay(20);
//     drive(50, true);
//     //detectColors();
//   }
  
//   stopMotors();
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("GOAL REACHED");
//   delay(3000);
//   return;
// }

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