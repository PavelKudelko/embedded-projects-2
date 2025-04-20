// DOM Element References
const compassSlider = document.getElementById('compassSlider');
const lidarText = document.getElementById('lidarText');
const compassDisplay = document.getElementById('compassDisplay');
const rgbDisplay = document.getElementById('rgbDisplay');

// Improved logout function
function logout() {
    fetch('/logout', {
        method: 'GET',
        credentials: 'include'
    })
    .then(response => {
        if (response.status === 401) {
            // Clear any stored credentials
            localStorage.removeItem('credentials');

            // Redirect to logout page
            window.location.href = '/logged-out';
        } else {
            // Fallback redirect
            window.location.href = '/logged-out';
        }
    })
    .catch(error => {
        console.error('Logout error:', error);
        window.location.href = '/logged-out';
    });
}

// Function to send authenticated requests
async function fetchWithAuth(url, options = {}) {
    // Use Basic Authentication
    const authHeader = 'Basic ' + btoa('username:pwd');

    const defaultOptions = {
        headers: {
            'Authorization': authHeader
        },
        credentials: 'include'
    };

    // Merge default options with any passed options
    const mergedOptions = { ...defaultOptions, ...options };

    try {
        const response = await fetch(url, mergedOptions);

        if (response.status === 401) {
            // Unauthorized - clear any stored credentials and redirect
            localStorage.removeItem('credentials');
            window.location.href = "/logout";
            return null;
        }

        return response;
    } catch (error) {
        console.error('Fetch error:', error);
        alert('Network error. Please try again.');
        return null;
    }
}

// Reset compass slider to middle
if (compassSlider) {
    const middleValue = 0;
    compassSlider.value = middleValue;
    updateCompass(middleValue);
}

// Periodic data fetching
setInterval(fetchLidarData, 500);
setInterval(fetchCompassData, 500);
setInterval(fetchWarning, 300);
setInterval(fetchRGB, 200);

// Color display update function
// Color display update function
function updateColorDisplay(rgbString) {
    const rgbValues = rgbString.split(';');
    if (rgbValues.length === 3) {
        const r = parseInt(rgbValues[0]);
        const g = parseInt(rgbValues[1]);
        const b = parseInt(rgbValues[2]);

        const colorDisplay = document.getElementById('colorDisplay');
        const colorText = document.getElementById('colorText');
        const rgbDisplay = document.getElementById('rgbDisplay');

        // Use the actual RGB values for the background color
        const backgroundColor = `rgb(${r}, ${g}, ${b})`;

        // Determine color name based on RGB values
        let colorName = "Unknown";

        // Simple color detection thresholds
        const threshold = 50;
        const isRed = r > threshold && g < threshold && b < threshold;
        const isGreen = g > threshold && r < threshold && b < threshold;
        const isBlue = b > threshold && r < threshold && g < threshold;
        const isYellow = r > threshold && g > threshold && b < threshold;
        const isWhite = r > threshold && g > threshold && b > threshold;
        const isBlack = r < threshold && g < threshold && b < threshold;

        if (isRed) colorName = "Red";
        else if (isGreen) colorName = "Green";
        else if (isBlue) colorName = "Blue";
        else if (isYellow) colorName = "Yellow";
        else if (isWhite) colorName = "White";
        else if (isBlack) colorName = "Black";

        // Update the color display with actual RGB values
        colorDisplay.style.backgroundColor = backgroundColor;
        colorText.innerText = `Detected: ${colorName}`;

        // Format RGB values for display
        rgbDisplay.innerText = `${r};${g};${b}`;
    }
}

// Fetch and display warning status
async function fetchWarning() {
    try {
        const response = await fetchWithAuth('/warning');
        if (!response) return;

        const data = await response.text();
        const warningElement = document.getElementById('warningMessage');

        if (data.trim() === 'true') {
            warningElement.style.opacity = '0';
            warningElement.style.display = 'block';
            setTimeout(() => {
                warningElement.style.opacity = '1';
            }, 10);
        } else {
            warningElement.style.opacity = '0';
            setTimeout(() => {
                warningElement.style.display = 'none';
            }, 300);
        }
    } catch (error) {
        console.error('error fetching warning: ', error);
    }
}

// Optional: Add an event listener for authentication failures
window.addEventListener('load', () => {
    window.addEventListener('error', (event) => {
        if (event.target.status === 401) {
            window.location.href = '/logout';
        }
    }, true);
});

// Update compass display
function updateCompass(pos) {
    document.getElementById('compassValue').innerText = `${pos}°`;
}

// Send compass value to server
function sendCompassValue(pos) {
    fetchWithAuth(`/compass?value=${pos}`);
    console.log("Compass value", pos);
}

// Movement functions
function forwards5() { move('forwards', 5); }
function forwards20() { move('forwards', 20); }
function backwards5() { move('backwards', 5); }
function backwards20() { move('backwards', 20); }
function Drive50() { DriveGoalDist(50); }
function DriveGoal() { DriveGoal(); }


// General movement function
function move(dir, dis) {
    fetchWithAuth(`/${dir}${dis}`);
    console.log("Drive", dir, dis);
}

// Drive to goal function
function DriveGoal() {
    fetchWithAuth(`/DriveGoal`);
    console.log("Drive to goal");
}

// Drive to goal distance function
function DriveGoalDist(dist) {
    fetchWithAuth(`/DriveGoalDist${dist}`);
    console.log("Drive to goal distance");
}

// Fetch LIDAR data
async function fetchLidarData() {
    try {
        const response = await fetchWithAuth('/lidar');
        if (!response) return;

        const data = await response.text();
        const lidarValue = parseInt(data, 10);

        if (lidarText) lidarText.innerText = `${lidarValue} cm`;
        console.log(`${lidarValue} cm`);
    } catch (error) {
        console.error('Error fetching LIDAR data:', error);
    }
}

// Fetch compass data
async function fetchCompassData() {
    try {
        const response = await fetchWithAuth('/compass_value');
        if (!response) return;

        const data = await response.text();
        const compassVal = parseInt(data, 10);

        if (compassDisplay) compassDisplay.innerText = `${compassVal}`;
        console.log(`${compassVal} degree`);
    } catch (error) {
        console.error('Error fetching COMPASS data:', error);
    }
}

// Fetch RGB data
async function fetchRGB() {
    try {
        const response = await fetchWithAuth('/rgb');
        if (response) {
            const data = await response.text();

            if (rgbDisplay) {
                rgbDisplay.innerText = data;
            }

            // Update color display with the new RGB values
            updateColorDisplay(data);
        }
    } catch (error) {
        console.error('Error fetching RGB data:', error);
    }
}

// Encoder calibration function
async function calibrateEncoder() {
    try {
        const response = await fetchWithAuth(`/calibrateEncoder`);
        if (!response) return;

        const data = await response.text();
        console.log("Encoder calibration data:", data);

        const encoderDisplay = document.getElementById('encoderDisplay');
        if (encoderDisplay) {
            encoderDisplay.innerText = data;
        }
    } catch (error) {
        console.error('Error calibrating encoder:', error);
    }
}

// Initial color display update when page loads
document.addEventListener('DOMContentLoaded', function() {
    const rgbText = document.getElementById('rgbDisplay').innerText;
    updateColorDisplay(rgbText);
});