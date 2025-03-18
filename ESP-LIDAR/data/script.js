const compassSlider = document.getElementById('compassSlider'); // Get the compass slider element from the HTML page
const lidarText = document.getElementById('lidarText'); // // Get the LIDAR text element from the HTML page
const compassDisplay = document.getElementById('compassDisplay');
const rgbDisplay = document.getElementById('rgbDisplay');


// Check if the compassSlider element exists and reset it to 0
if (compassSlider) {
	const middleValue = 0;
	compassSlider.value = middleValue;	// Set slider to the middle position (0 degrees)
	updateCompass(middleValue);			// Set the compass value to initial value of 0
}


setInterval(fetchLidarData, 500); // Fetch LIDAR data every half second
setInterval(fetchCompassData, 500); // fetch cmpsVal every 0.5 second'
setInterval(fetchWarning, 300);
setInterval(fetchRGB, 200);

// Function to update the color display based on which RGB component is dominant
function updateColorDisplay(rgbString) {
  // Parse the RGB string (format: "r;g;b")
  const rgbValues = rgbString.split(';');
  if (rgbValues.length === 3) {
    const r = parseInt(rgbValues[0]);
    const g = parseInt(rgbValues[1]);
    const b = parseInt(rgbValues[2]);

    const colorDisplay = document.getElementById('colorDisplay');
    const colorText = document.getElementById('colorText');

    // Determine dominant color or special cases
    let dominantColor = "Black";
    let backgroundColor = "rgb(0, 0, 0)";

    const max = Math.max(r, g, b);

    if (max === r) {
      dominantColor = "Red";
      backgroundColor = "rgb(255, 0, 0)";
    } else if (max === g) {
      dominantColor = "Green";
      backgroundColor = "rgb(0, 255, 0)";
    } else if (max === b) {
      dominantColor = "Blue";
      backgroundColor = "rgb(0, 0, 255)";
    }

    // Update the display
    colorDisplay.style.backgroundColor = backgroundColor;
    colorText.innerText = "Detected: " + dominantColor;
  }
}

// Add this to your existing code where you update the RGB display
// For example, if you have:
// document.getElementById('rgbDisplay').innerText = rgbData;
// Add this after:
// updateColorDisplay(rgbData);

// For testing purposes, you can also parse the existing RGB text when the page loads
document.addEventListener('DOMContentLoaded', function() {
  const rgbText = document.getElementById('rgbDisplay').innerText;
  updateColorDisplay(rgbText);
});


async function fetchWarning() {
    try {
        const response = await fetch('/warning');
        const data = await response.text();

        const warningElement = document.getElementById('warningMessage');

        if (data.trim() === 'true') {
            // Add fade-in effect
            warningElement.style.opacity = '0';
            warningElement.style.display = 'block';
            setTimeout(() => {
                warningElement.style.opacity = '1';
            }, 10);
        } else {
            // Add fade-out effect
            warningElement.style.opacity = '0';
            setTimeout(() => {
                warningElement.style.display = 'none';
            }, 300); // Wait for fade out to complete
        }
    } catch (error) {
        console.error('error fetching warning: ', error);
    }
}

function logout() {
  fetch('/logout');
  console.log("logout");
}

// Function to update the compass display (show the current compass position)
function updateCompass(pos) {
	document.getElementById('compassValue').innerText = `${pos}°`;// Update the text value for compassValue when value is changing on page element with the current position and add "°" for degrees
}


// Function to send the current compass value to the server
function sendCompassValue(pos) {
	fetch(`/compass?value=${pos}`);		// Send the compass value to the server using a fetch request
	console.log("Compass value", pos);	// Log the compass value to the console for debugging
}


// Functions for moving the motor forward and backward with specific distances
function forwards5() { move('forwards', 5); } 		// Calling move function with separated parameters
function forwards20() { move('forwards', 20); }		// Calling move function with separated parameters
function backwards5() { move('backwards', 5); }		// Calling move function with separated parameters
function backwards20() { move('backwards', 20); }	// Calling move function with separated parameters


// This is a general-purpose function that can handle different combinations of direction and distance
function move(dir, dis) {
	fetch(`/${dir}${dis}`);			// Sends a request to the server with a URL constructed from the received direction and distance (e.g., /forwards5)
	console.log("Drive", dir, dis);	// Log the movement command to the console for debugging
}


// This function fetches asynchronously LIDAR data from the server and updates the display.
async function fetchLidarData() {
  try {
    const response = await fetch('/lidar');		// Fetch the LIDAR data from the server
    const data = await response.text();			// Get the response text
    const lidarValue = parseInt(data, 10); 		// Parse the LIDAR data as an integer

    if (lidarText) lidarText.innerText = `${lidarValue} cm`;	// Update the LIDAR text element with the fetched data
    console.log(`${lidarValue} cm`);
  } catch (error) {
    console.error('Error fetching LIDAR data:', error);		 	// Log any errors to the console
  }
}

async function fetchCompassData() {
  try {
    const response = await fetch('/compass_value');
    const data = await response.text();
    const compassVal = parseInt(data, 10);

    if (compassDisplay) compassDisplay.innerText = `${compassVal}`;
    console.log(`${compassVal} degree`)
  } catch (error) {
    console.error('Error fetching COMPASS data:', error)
  }
}

async function fetchRGB() {
  try {
    const response = await fetch('/rgb');
    const data = await response.text();

    console.log("Raw RGB data:", data);

    if (rgbDisplay) {
      rgbDisplay.innerText = data;
      updateColorDisplay(data);

      // Debug what values are being parsed
      const values = data.split(';');
      console.log("R:", values[0], "G:", values[1], "B:", values[2]);
    }
  } catch (error) {
    console.error('Error fetching RGB data:', error);
  }
}