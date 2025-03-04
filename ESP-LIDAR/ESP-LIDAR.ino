#include <ESP8266WiFi.h>        // Library for Wi-Fi functionality
#include <ESP8266WebServer.h>   // Library to create and manage a web server
#include <FS.h>                 // Library for working with file systems (SPIFFS)

const char* ssid = "SOURCE";        // Wi-Fi network name (SSID)
const char* password = "Pelle!23";  // Wi-Fi network password
String lidarData = "0 cm";          // Variable to store LIDAR data
String cmpsVal = "0°"; // Store compass value
bool isWarning = false;
unsigned long lastWarningTime = 0;
const unsigned long WARNING_DURATION = 2000;
String rgbData = "0;0;0"

ESP8266WebServer server(80);    // Create an instance of the WebServer on port 80 (default HTTP port)


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


  // Set up the web pages (URLS) and files that the server will show/use when someone visits the site
  server.serveStatic("/", SPIFFS, "/index.html");             // Serves and shows the main webpage (HTML file) when someone visits the home page
  server.serveStatic("/style.css", SPIFFS, "/style.css");     // Serve the CSS file for styling
  server.serveStatic("/script.js", SPIFFS, "/script.js");     // Serve the JavaScript file
  server.serveStatic("/favicon.ico", SPIFFS, "/favicon.png"); // Serve a favicon (small icon) for the website


  // Define custom actions/function calls for specific URLs (ie when a button press from webpage requests a particular URL)
  server.on("/forwards5", [](){ handleMove(5); });      // When requesting URL "/forwards5", call the handleMove function with parameter 5
  server.on("/forwards20", [](){ handleMove(20); });    // [](){ handleMove(x); } is a lambda function that contains the code
  server.on("/backwards5", [](){ handleMove(-5); });    // to be executed when the route is visited. These lambda functions call
  server.on("/backwards20", [](){ handleMove(-20); });  // handleMove(x) as soon as it's triggered.
  server.on("/compass", handleCompass);                 // When requesting URL "/compass", call the handleCompass function
  server.on("/lidar", handleLidar);                     // When requesting URL "/lidar", call the handleLidar function.
  server.on("/compass_value", handleCompassValue); // compass value
  server.on("/warning", handleWarning);
//  server.on("/cantmove", handleCantMove);
//  server.on("/stop", handleStop);
  server.on("/rgb", handleRGB);

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
