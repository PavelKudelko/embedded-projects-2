#include <ESP8266WiFi.h>        // Library for Wi-Fi functionality
#include <ESP8266WebServer.h>   // Library to create and manage a web server
#include <FS.h>                 // Library for working with file systems (SPIFFS)

const char* ssid = "Titenet-IoT";        // Wi-Fi network name (SSID)
const char* password = "7kDtaphg";  // Wi-Fi network password

const String HTTP_USERNAME = "FastCar";
const String HTTP_PASSWORD = "esp8266";

String lidarData = "0 cm";
String cmpsVal = "0°"; // Store compass value
bool isWarning = false;
unsigned long lastWarningTime = 0;
const unsigned long WARNING_DURATION = 2000;
String rgbData = "0;0;0";

ESP8266WebServer server(80);    // Create an instance of the WebServer on port 80 (default HTTP port)

volatile bool isAuthenticated = false;

bool handleAuth() {
  if (!server.authenticate(HTTP_USERNAME.c_str(), HTTP_PASSWORD.c_str())) {
    server.requestAuthentication(BASIC_AUTH, "Login Required", "Please enter your credentials");
    return false;
  }
  isAuthenticated = true;
  return true;
}

void setup() {
  Serial.begin(9600);

  // Initialize the file system (SPIFFS) on the ESP8266
  if (!SPIFFS.begin()) {        // Initialize and check success on SPIFFS
    Serial.println("Error while mounting SPIFFS");
    return;
  }

  // Connect to the Wi-Fi network using the provided SSID and password
  Serial.print("\nConnecting to " + (String)ssid);
  WiFi.begin(ssid, password);               // Begin connecting to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {   // Wait in a loop until the connection is successful
    delay(500);
    Serial.print(".");
  } Serial.println("\nIP address: " + WiFi.localIP().toString()); // Print the IP address of the ESP8266 when connected

  // Set up the web pages with authentication
  server.on("/", HTTP_GET, []() {
    if (!handleAuth()) return;
    File file = SPIFFS.open("/index.html", "r");
    server.streamFile(file, "text/html");
    file.close();
  });             // Serves and shows the main webpage (HTML file) when someone visits the home page

  // Static files with authentication
  server.on("/style.css", HTTP_GET, []() {
    if (!handleAuth()) return;
    File file = SPIFFS.open("/style.css", "r");
    server.streamFile(file, "text/css");
    file.close();
  });     // Serve the CSS file for styling
  server.on("/script.js", HTTP_GET, []() {
    if (!handleAuth()) return;
    File file = SPIFFS.open("/script.js", "r");
    server.streamFile(file, "text/javascript");
    file.close();
  });     // Serve the JavaScript file
  server.on("/favicon.ico", HTTP_GET, []() {
    if (!handleAuth()) return;
    File file = SPIFFS.open("/favicon.png", "r");
    server.streamFile(file, "image/png");
    file.close();
  }); // Serve a favicon (small icon) for the website

  // Logout page route
  server.on("/logout_page", HTTP_GET, []() {
    File file = SPIFFS.open("/logout.html", "r");
    server.streamFile(file, "text/html");
    file.close();
    isAuthenticated = false; // Reset authentication
  });

  // Define custom actions/function calls for specific URLs
  server.on("/forwards5", [](){ 
    if (!handleAuth()) return;
    handleMove(5); 
  });      
  server.on("/forwards20", [](){ 
    if (!handleAuth()) return;
    handleMove(20); 
  });    
  server.on("/backwards5", [](){ 
    if (!handleAuth()) return;
    handleMove(-5); 
  });    
  server.on("/backwards20", [](){ 
    if (!handleAuth()) return;
    handleMove(-20); 
  });  
  server.on("/compass", [](){ 
    if (!handleAuth()) return;
    handleCompass(); 
  });                 
  server.on("/lidar", [](){ 
    if (!handleAuth()) return;
    handleLidar(); 
  });                     
  server.on("/compass_value", [](){ 
    if (!handleAuth()) return;
    handleCompassValue(); 
  }); 
  server.on("/warning", [](){ 
    if (!handleAuth()) return;
    handleWarning(); 
  });
  server.on("/rgb", [](){ 
    if (!handleAuth()) return;
    handleRGB(); 
  });
  
  server.on("/logout", []() {
      isAuthenticated = false;
      server.sendHeader("Location", "/logout_page", true);
      server.send(302, "text/plain", "");  // Empty body for clean redirect
  });
  //  If someone tries to access a URL that does not exist (e.g. due to a typo), call the handleNotFound function
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
    else {
      Serial.println("Command not found");
    }
  }
}

void handlelogout() {
  Serial.println("entered logout");
  isAuthenticated = false;
  // Clear authentication and force browser to forget credentials
  // server.sendHeader("WWW-Authenticate", "Basic realm=\"Login Required\"");
  
  // Serve the logout page
  File file = SPIFFS.open("/logout.html", "r");
  server.streamFile(file, "text/html");
  file.close();
}

// This function is called when a non-existing URL is accessed (404 error)
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found"); // Send a 404 response with a plain text message
}

// This function handles movement commands like "/forwards5" or "/backwards20"
void handleMove(int distance) {
  Serial.println("Move:" + String(distance));       // Print the movement distance to the serial monitor for Arduino Mega
  server.send(200);                                 // Send a 200 OK response to the client (browser)
}

// This function handles the compass command (like "/compass?value=30")
void handleCompass() {
  if (server.hasArg("value")) {                     // Check if there is a "value" argument in the request URL
    String valueString = server.arg("value");       // Get the "value" argument from the URL
    Serial.println("Turn:" + valueString);          // Print the compass value to the serial monitor
  }
  server.send(200);
}

// This function handles the lidar command (like "/lidar")
void handleLidar() {
  server.send(200, "text/plain", lidarData); // Send the latest Lidar value with a 200 OK status.
}

void handleCompassValue() {
  server.send(200, "text/plain", cmpsVal);
}

void handleWarning() {
  server.send(200, "text/plain", isWarning ? "true" : "false");
}

void handleRGB() {
  server.send(200, "text/plain", rgbData);
}

//void handleCantMove() {
//  server.send(200, "text/plain", "CANTMOVE");
//}
//
//void handleStop() {
//  server.send(200, "text/plain", "EMERGENCY-STOP");
//}