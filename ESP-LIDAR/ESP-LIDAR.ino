#include <ESP8266WiFi.h>        // Library for Wi-Fi functionality
#include <ESP8266WebServer.h>   // Library to create and manage a web server
#include <FS.h>                 // Library for working with file systems (SPIFFS)


// Wi-Fi and Authentication Credentials
const char* WIFI_SSID = "Titenet-IoT";        // Wi-Fi network name (SSID)
const char* WIFI_PASSWORD = "7kDtaphg";       // Wi-Fi network password

// HTTP Authentication Credentials
const char* HTTP_USERNAME = "username";       // Change this to your desired username
const char* HTTP_PASSWORD = "pwd";            // Change this to your desired password

// Global variables for sensor data
String lidarData = "0 cm";
String cmpsVal = "0°";           // Store compass value
bool isWarning = false;
unsigned long lastWarningTime = 0;
const unsigned long WARNING_DURATION = 2000;
String rgbData = "0;0;0";


String encoderVal = "0";  // This will store the calibrated value

ESP8266WebServer server(80);    // Create an instance of the WebServer on port 80 (default HTTP port)

void setup() {
  Serial.begin(9600);

  // Initialize the file system (SPIFFS) on the ESP8266
  if (!SPIFFS.begin()) {
    Serial.println("Error while mounting SPIFFS");
    return;
  }

  // Connect to the Wi-Fi network using the provided SSID and password
  Serial.print("\nConnecting to " + String(WIFI_SSID));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  } 
  Serial.println("\nIP address: " + WiFi.localIP().toString());

  // Root route with explicit authentication handling
  server.on("/", HTTP_GET, []() { 
    // Always require authentication
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      server.requestAuthentication(BASIC_AUTH, "Secure Area", "Authentication Required");
      return;
    }
    
    // Serve the main page
    File file = SPIFFS.open("/index.html", "r");
    if (!file) {
      server.send(500, "text/plain", "Failed to open index.html");
      return;
    }
    server.streamFile(file, "text/html");
    file.close();
  });

  // Static files with authentication
  server.on("/style.css", HTTP_GET, []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    File file = SPIFFS.open("/style.css", "r");
    server.streamFile(file, "text/css");
    file.close();
  });

  server.on("/script.js", HTTP_GET, []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    File file = SPIFFS.open("/script.js", "r");
    server.streamFile(file, "text/javascript");
    file.close();
  });

  server.on("/favicon.ico", HTTP_GET, []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    File file = SPIFFS.open("/favicon.png", "r");
    server.streamFile(file, "image/png");
    file.close();
  });

  // Explicit logout route
  server.on("/logout", HTTP_GET, []() {
    // Send a 401 Unauthorized response to force re-authentication
    server.sendHeader("WWW-Authenticate", "Basic realm=\"Secure Area\"");
    server.send(401, "text/plain", "Logged Out. Please re-authenticate.");
  });

  server.on("/logged-out", HTTP_GET, []() {
    // Serve a logout confirmation page
    File file = SPIFFS.open("/logout.html", "r");
    if (!file) {
      server.send(500, "text/plain", "Logout page not found");
      return;
    }
    server.streamFile(file, "text/html");
    file.close();
  });

  // Movement routes with authentication
  server.on("/forwards5", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleMove(5); 
  });      
  
  server.on("/forwards20", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleMove(20); 
  });    
  
  server.on("/backwards5", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleMove(-5); 
  });    
  
  server.on("/backwards20", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleMove(-20); 
  }); 
  server.on("/Drive50", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleMove(50); 
  });   
  server.on("/DriveGoal", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleDriveGoal(); 
  });   

  // Other routes with authentication
  server.on("/compass", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleCompass(); 
  });                 
  
  server.on("/lidar", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleLidar(); 
  });                     
  
  server.on("/compass_value", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleCompassValue(); 
  }); 
  
  server.on("/warning", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleWarning(); 
  });
  
  server.on("/rgb", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleRGB(); 
  });
  
  server.on("/calibrateEncoder", []() { 
    if (!server.authenticate(HTTP_USERNAME, HTTP_PASSWORD)) {
      return server.requestAuthentication();
    }
    handleCalibrateEncoder(); 
  });
  
  // Handle not found requests
  server.onNotFound(handleNotFound);

  // Start the web server
  server.begin();
}

void loop() {
  server.handleClient();

  // Check if warning should be cleared based on time
  if (isWarning && (millis() - lastWarningTime > WARNING_DURATION)) {
    isWarning = false;
  }

  // Check if data is available from the Arduino Mega
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();  // Remove any trailing newline or spaces

    if (data.startsWith("LIDAR:")) {
      lidarData = data.substring(6);
      lidarData.trim();
    }
    else if (data.startsWith("COMPASS:")) {
      cmpsVal = data.substring(8);
      cmpsVal.trim();
    }
    else if (data.startsWith("WARNING")) {
      isWarning = true;
      lastWarningTime = millis(); // Update the last warning time
    }
    else if (data.startsWith("RGB:")) {
      rgbData = data.substring(4);
      rgbData.trim();
    }
    else if (data.startsWith("ENCODER_VAL:")) {
      encoderVal = data.substring(12);
      encoderVal.trim();
    }
    else {
      Serial.println("Command not found");
    }
  }
}

// Handle not found requests
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}

// Handle movement commands
void handleMove(int distance) {
  Serial.println("Move:" + String(distance));
  server.send(200);
}

// Handle compass command
void handleCompass() {
  if (server.hasArg("value")) {
    String valueString = server.arg("value");
    Serial.println("Turn:" + valueString);
  }
  server.send(200);
}

// Handle LIDAR command
void handleLidar() {
  server.send(200, "text/plain", lidarData);
}

// Handle compass value request
void handleCompassValue() {
  server.send(200, "text/plain", cmpsVal);
}

// Handle warning status
void handleWarning() {
  server.send(200, "text/plain", isWarning ? "true" : "false");
}

// Handle RGB data
void handleRGB() {
  server.send(200, "text/plain", rgbData);
}

void  handleDriveGoal() {
  Serial.println("DriveGoal");
  server.send(200);
}

void handleCalibrateEncoder() {
  Serial.println("CALIBRATE_ENCODER");
  server.send(200, "text/plain", encoderVal);
}

