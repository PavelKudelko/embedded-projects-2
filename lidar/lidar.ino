#include <Wire.h>
#include <LIDARLite.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

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

// Function declarations
void handleSerialControl();
void stopMotors();
void followCommand(int param = 20);
void turnExact(int angle, String direction = "");
void driveTurn();
void driveCircle();
int get_dist();
void measure();
float display(float area, float volume, int dist1, int dist2, int dist3, int dist4);
void turnLeft(int speedPercent);
void turnRight(int speedPercent);
void drive(int speed, bool direction);

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

void setup() {
  delay(2000);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  // read eeprom:

  // Serial.println("EEPROM Contents:");
  // // Iterate through all EEPROM addresses
  // for (int address = 0; address < EEPROM.length(); address++) {
  //   byte value = EEPROM.read(address);
  //   Serial.print(address);
  //   Serial.print("\t");
  //   Serial.println(value, DEC);
  // }
  // Serial.println("EEPROM Read Complete.");

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
  // // Collecting lidar data
  // int dist = get_dist(); // Get distance measurement
  // Serial.println(dist);
  handleSerialControl();

  if (buttonPressed) {
    stopMotors();
    buttonPressed = false;
  }

  Serial.println(get_dist());

  delay(500); // Delay to control update frequency
}

void displayLidarValues() {
    int distance = get_dist();
    lcd.setCursor(0, 3);
    lcd.print("LIDAR: ");
    lcd.print(distance);
    lcd.print(" cm  "); 
}

void followCommand(int param = 20) {
  while (true) {
    // Check if the button is pressed to stop the car
    if (buttonPressed) {

      stopMotors();  // Stop the car
      return;  // Exit the followCommand function, stopping the loop
    }

    // dist from potentialmeter
    int param = map(analogRead(potPin), 0, 1023, 5, 40);
    // Follow logic
    if (get_dist() > param) {
      drive(50, true);  // Move forward
    }
    else if (get_dist() < param){
      drive(50, false);  // Stop or move backward
    }
    // just stop if it reaches needed dist
    else {
      stopMotors();
    }

    
  }
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
      turnRight(25); 
    } else {
      turnLeft(25);
    }
    delay(50);
  }

  stopMotors(); // Stop the motors after the turn
}

void driveTurn() {
  while (true) {
    if (buttonPressed) {
      stopMotors();
      return;
    }

    int param = map(analogRead(potPin), 0, 1023, 5, 40);
    lcd.setCursor(0, 3);
    lcd.print(param);

    if (get_dist() >= param) {
      drive(40, true);
    }
    else {
      drive(40, true);
      turnExact(90);
    }
    
  }
}

void driveCircle() {
  float targetDistance1 = 25; // First target distance
  float targetDistance2 = 20; // Second target distance

  int initPulse = pulseCountR;

  // Drive forward until Lidar detects the target distance
  while (get_dist() > targetDistance1) { 
    drive(50, true); 
    Serial.println(get_dist());
  }
  turnExact(90, "left");

  // Drive forward until Lidar detects the second target distance
  while (get_dist() > targetDistance2) {
    drive(50, true); 
  }
  turnExact(90, "left");

  // Drive forward again until Lidar detects the first target distance
  while (get_dist() > targetDistance1) {
    drive(50, true);
  }
  turnExact(90, "left");

  // Drive forward until Lidar detects the second target distance again
  while (get_dist() > targetDistance2) {
    drive(50, true);
  }
  turnExact(90, "left");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("dist: ");
  lcd.print(pulseCountR - initPulse);
}


void handleSerialControl() {
  // Check if there is incoming data from the serial monitor
  if (Serial.available() > 0) {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');
    Serial.print("Message received, content: ");
    Serial.println(message);
    
    // Handle Follow command (no parameters)
    int pos_follow = message.indexOf("follow");

    if (pos_follow > -1) {
      Serial.println("Command = Follow");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = Follow");

      followCommand();
    }
    // Handle Measure command (no parameters)
    else if (message.indexOf("measure") > -1) {
      Serial.println("Command = Measure");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = Measure");

      // Execute measure command logic
      measure();
    }
    else if (message.indexOf("driveTurn") > -1) {
      Serial.println("Command = driveTurn");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = driveTurn");

      driveTurn();
    }
    else if (message.indexOf("driveCircle") > -1) {
      Serial.println("Command = driveCircle");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = driveCircle");

      driveCircle();
    }
    else if (message.indexOf("calibrate") > - 1) {
      Serial.println("Command = calibrate");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = driveCircle");

      calibrate();
    }
    else if (message.indexOf("getEEPROM") > - 1) {
      Serial.println("Command = getEEPROM");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = getEEPROM");

      getEEPROM();
    }
    // Handle invalid or unrecognized commands
    else {
      lcd.setCursor(0, 0);
      lcd.print("Error: unknown cmd");
    }
  }
}

void getEEPROM(){
  uint8_t retrievedValue = EEPROM.read(0);
  double distPerPulse = retrievedValue / 100.0; // Convert back to double

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EEPROM val: ");
  lcd.print(distPerPulse);
}

void calibrate() {
  int initDist = get_dist();
  pulseCountR = 0;
  pulseCountL = 0;
  int finalDist = initDist;  

  // Drive until we have traveled 20cm
  while (abs(initDist - finalDist) < 20) { 
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

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("dist: ");
  lcd.print(abs(initDist - finalDist));
  lcd.setCursor(0, 1);
  lcd.print("pls cnt: ");
  lcd.print(pulseCountR);
  lcd.print("|");
  lcd.print(pulseCountL);
  lcd.setCursor(0, 2);
  lcd.print("distPerPulse: ");
  lcd.print(distPerPulse);

  // Store to EEPROM
  uint8_t scaledValue = round(distPerPulse * 100);
  EEPROM.update(0, scaledValue);
  lcd.setCursor(0, 3);
  lcd.print("EEPROM val updated");
}

int get_dist() {
  for (int i = 0; i < LIDAR_SAMPLES; i++) {
    lidar_vals[i] = myLidarLite.distance() - LIDAR_CORR_VAL;
  }

  return avg_lidar_val(lidar_vals, LIDAR_SAMPLES); // Calculate average
}

void measure() {
  if (buttonPressed) {
    buttonPressed = false;
    stopMotors();
  }
  int dist1 = get_dist(); // First distance
  turnExact(90);

  int dist2 = get_dist(); // Second distance
  turnExact(90);

  int dist3 = get_dist(); // Third distance
  turnExact(90);

  int dist4 = get_dist(); // Fourth distance

  // Debugging distances
  Serial.print("Distances: ");
  Serial.print(dist1); Serial.print(", ");
  Serial.print(dist2); Serial.print(", ");
  Serial.print(dist3); Serial.print(", ");
  Serial.println(dist4);

  // Check for valid inputs
  if (dist1 < 0 || dist2 < 0 || dist3 < 0 || dist4 < 0 || ROBOT_LENGTH < 0) {
      Serial.println("Error: Negative distance or robot length!");
      lcd.setCursor(0, 0);
      lcd.print("Invalid distances!");
      return;
  }

  // Step-by-step area calculation
  float side1 = dist1 + ROBOT_LENGTH + dist3;
  float side2 = dist2 + ROBOT_LENGTH + dist4;

  // Debugging sides
  Serial.print("Side 1 (m): ");
  Serial.println(side1 / 100.0); // Convert to meters for debugging
  Serial.print("Side 2 (m): ");
  Serial.println(side2 / 100.0); // Convert to meters for debugging

  // Final area calculation
  float area = (side1 * side2) / 10000.0; // Convert cm^2 to m^2
  Serial.print("Intermediate area (cm^2): ");
  Serial.println(side1 * side2);
  Serial.print("Converted area (m^2): ");
  Serial.println(area);

  // Volume calculation
  float volume = area * 0.3; // Height = 0.3 m
  Serial.print("Volume (m^3): ");
  Serial.println(volume);

  // Display results
  display(area, volume, dist1, dist2, dist3, dist4);
}

float display(float area, float volume, int dist1, int dist2, int dist3, int dist4) {
  // Display area and volume on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Area: ");
  lcd.print(area);
  lcd.print(" m^2");
  lcd.setCursor(0, 1);
  lcd.print("Volume: ");
  lcd.print(volume);
  lcd.print(" m^3");

  // Print area and volume to Serial Monitor
  Serial.print("Area: ");
  Serial.print(area);
  Serial.print(" m^2, Volume: ");
  Serial.print(volume);
  Serial.println(" cm^3");

  // Display the four distances on the LCD
  lcd.setCursor(0, 3);
  lcd.print(dist1);
  lcd.print("|");
  lcd.print(dist2);
  lcd.print("|");
  lcd.print(dist3);
  lcd.print("|");
  lcd.print(dist4);
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
