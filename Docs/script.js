// Theme Toggle
const themeToggleBtn = document.getElementById('theme-toggle');
const body = document.body;
const iconSun = '<svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="5"></circle><line x1="12" y1="1" x2="12" y2="3"></line><line x1="12" y1="21" x2="12" y2="23"></line><line x1="4.22" y1="4.22" x2="5.64" y2="5.64"></line><line x1="18.36" y1="18.36" x2="19.78" y2="19.78"></line><line x1="1" y1="12" x2="3" y2="12"></line><line x1="21" y1="12" x2="23" y2="12"></line><line x1="4.22" y1="19.78" x2="5.64" y2="18.36"></line><line x1="18.36" y1="5.64" x2="19.78" y2="4.22"></line></svg>';
const iconMoon = '<svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"></path></svg>';

// Check local storage or system preference
const currentTheme = localStorage.getItem('theme') || (window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
document.documentElement.setAttribute('data-theme', currentTheme);
if (themeToggleBtn) updateThemeIcon(currentTheme);

if (themeToggleBtn) {
    themeToggleBtn.addEventListener('click', () => {
        let theme = document.documentElement.getAttribute('data-theme');
        let newTheme = theme === 'light' ? 'dark' : 'light';
        document.documentElement.setAttribute('data-theme', newTheme);
        localStorage.setItem('theme', newTheme);
        updateThemeIcon(newTheme);
    });
}

function updateThemeIcon(theme) {
    if (theme === 'dark') {
        themeToggleBtn.innerHTML = iconSun;
    } else {
        themeToggleBtn.innerHTML = iconMoon;
    }
}

// Mobile Menu Toggle
const menuToggle = document.getElementById('mobile-menu-toggle');
const sidebar = document.querySelector('.sidebar');
if (menuToggle && sidebar) {
    menuToggle.addEventListener('click', () => {
        sidebar.classList.toggle('open');
    });

    document.addEventListener('click', (e) => {
        if (window.innerWidth <= 768) {
            if (!sidebar.contains(e.target) && !menuToggle.contains(e.target) && sidebar.classList.contains('open')) {
                sidebar.classList.remove('open');
            }
        }
    });
}

// API Card Expansion Logic
function initApiCards() {
    const headers = document.querySelectorAll('.api-header');
    headers.forEach(header => {
        header.addEventListener('click', () => {
            const card = header.parentElement;

            // Toggle active class
            const isExpanded = card.classList.contains('expanded');

            // Optional: Close others? For now we allow multiple open
            // document.querySelectorAll('.api-card').forEach(c => c.classList.remove('expanded'));

            if (isExpanded) {
                card.classList.remove('expanded');
            } else {
                card.classList.add('expanded');
            }
        });
    });
}
// Run init after DOM load
document.addEventListener('DOMContentLoaded', initApiCards);


// Global Copy to Clipboard & Syntax Highlighting
document.querySelectorAll('pre').forEach(pre => {
    if (pre.parentNode.classList.contains('code-wrapper')) return;

    const wrapper = document.createElement('div');
    wrapper.className = 'code-wrapper';
    pre.parentNode.insertBefore(wrapper, pre);
    wrapper.appendChild(pre);

    const button = document.createElement('button');
    button.className = 'copy-btn';
    button.innerText = 'Copy';

    button.addEventListener('click', (e) => {
        // Prevent triggering parent expanding click if inside api-details
        e.stopPropagation();

        const codeBlock = pre.querySelector('code');
        if (!codeBlock) return;

        navigator.clipboard.writeText(codeBlock.innerText).then(() => {
            button.innerText = 'Copied!';
            setTimeout(() => { button.innerText = 'Copy'; }, 2000);
        }).catch(err => {
            console.error('Failed to copy!', err);
        });
    });

    wrapper.appendChild(button);

    const code = pre.querySelector('code');
    if (code) {
        highlightCode(code);
    }
});

function highlightCode(element) {
    let html = element.textContent;
    // Fix: Ensure no leading/trailing whitespace impacts the rendering
    if (html) html = html.trim();

    // Fix: Escape HTML entities to prevent browser from interpreting <UNI.h> as a tag
    html = html.replace(/</g, '&lt;').replace(/>/g, '&gt;');

    const keywords = /\b(void|int|float|double|bool|char|long|unsigned|const|static|return|if|else|for|while|struct|class|public|private|new|delete|break|continue|switch|case|#define|#include)\b/g;
    const types = /\b(String|UniBase|UniBaseControl|UniDev|Adafruit_SSD1306|OdometryData)\b/g;
    const comments = /(\/\/.*)/g;
    const strings = /("[^"]*")/g;
    const numbers = /\b(\d+(\.\d+)?)\b/g;
    const functions = /\b([a-zA-Z_]\w*)(?=\()/g;

    const tokens = [];
    const saveToken = (match) => { items = tokens.push(match); return `___TOKEN${items - 1}___`; };

    html = html.replace(comments, saveToken);
    html = html.replace(strings, saveToken);

    html = html.replace(keywords, '<span class="token keyword">$1</span>');
    html = html.replace(types, '<span class="token type">$1</span>');
    html = html.replace(functions, '<span class="token function">$1</span>');
    html = html.replace(numbers, '<span class="token number">$1</span>');

    html = html.replace(/___TOKEN(\d+)___/g, (match, id) => {
        const token = tokens[id];
        if (token.startsWith('//')) return `<span class="token comment">${token}</span>`;
        if (token.startsWith('"')) return `<span class="token string">${token}</span>`;
        return token;
    });

    element.innerHTML = html;
}

// Sidebar Active Link & TOC
const currentPath = window.location.pathname.split('/').pop() || 'index.html';
const sidebarLinks = document.querySelectorAll('.sidebar-nav a');

sidebarLinks.forEach(link => {
    if (link.getAttribute('href') === currentPath) {
        link.classList.add('active');
        // TOC generation removed to prevent duplication with manual sidebar-subnav
        // if (currentPath === 'unibase.html' || currentPath === 'unidev.html') { ... }
    }
});

// Global Search
// Global Search
const searchIndex = [
    {
        "page": "index.html",
        "title": "UNI Library",
        "id": "",
        "text": "A clean, efficient Arduino library for controlling educational robotic platforms based on ESP32."
    },
    {
        "page": "unibase.html",
        "title": "UniBase Reference",
        "id": "",
        "text": "Core class for controlling the robot chassis, motors, and display."
    },
    {
        "page": "unibase.html",
        "title": "Constants",
        "id": "constants",
        "text": "Public constants available for control functions."
    },
    {
        "page": "unibase.html",
        "title": "Initialization",
        "id": "initialization",
        "text": "These functions block execution until the movement is complete. They use encoder feedback for\n                    precision."
    },
    {
        "page": "unibase.html",
        "title": "Motor Control",
        "id": "motor-control",
        "text": "These functions block execution until the movement is complete. They use encoder feedback for\n                    precision."
    },
    {
        "page": "unibase.html",
        "title": "Movement (Blocking)",
        "id": "movement",
        "text": "These functions block execution until the movement is complete. They use encoder feedback for\n                    precision."
    },
    {
        "page": "unibase.html",
        "title": "Odometry",
        "id": "odometry",
        "text": "Functions for tracking the robot's position relative to its starting point."
    },
    {
        "page": "unibase.html",
        "title": "Display",
        "id": "display",
        "text": ""
    },
    {
        "page": "unibase.html",
        "title": "Utility",
        "id": "utility",
        "text": ""
    },
    {
        "page": "unibase.html",
        "title": "begin",
        "id": "initialization",
        "text": "Initializes all robot components (Motors, Encoders, OLED, Sensors). This function starts the\n                            background tasks for odometry"
    },
    {
        "page": "unibase.html",
        "title": "motors",
        "id": "motor-control",
        "text": "Sets the power for both motors directly. Positive values match forward direction, negative\n                            match backward."
    },
    {
        "page": "unibase.html",
        "title": "motorLeft",
        "id": "motor-control",
        "text": "Sets power for the left motor only."
    },
    {
        "page": "unibase.html",
        "title": "motorRight",
        "id": "motor-control",
        "text": "Sets power for the right motor only."
    },
    {
        "page": "unibase.html",
        "title": "motorsArc",
        "id": "motor-control",
        "text": "Moves the robot along an arc with a specific turn intensity. This is a non-blocking control\n                            function."
    },
    {
        "page": "unibase.html",
        "title": "stop",
        "id": "motor-control",
        "text": "Stops both motors. Can perform a smooth coast to stop or a hard brake."
    },
    {
        "page": "unibase.html",
        "title": "stopLeft",
        "id": "motor-control",
        "text": "Stops the left motor only."
    },
    {
        "page": "unibase.html",
        "title": "stopRight",
        "id": "motor-control",
        "text": "Stops the right motor only."
    },
    {
        "page": "unibase.html",
        "title": "moveDist",
        "id": "movement",
        "text": "Moves the blind robot specific distance using PID control on encoders."
    },
    {
        "page": "unibase.html",
        "title": "moveTime",
        "id": "movement",
        "text": "Moves the motor for a specific duration of time."
    },
    {
        "page": "unibase.html",
        "title": "moveArcDist",
        "id": "movement",
        "text": "Moves along a curved path for a specific distance."
    },
    {
        "page": "unibase.html",
        "title": "moveArcTime",
        "id": "movement",
        "text": "Moves along a curved path for a specific duration."
    },
    {
        "page": "unibase.html",
        "title": "rotate",
        "id": "movement",
        "text": "Rotates the robot in place by a specific angle."
    },
    {
        "page": "unibase.html",
        "title": "getOdometry",
        "id": "odometry",
        "text": "Returns the current estimated position and orientation struct."
    },
    {
        "page": "unibase.html",
        "title": "getAbsX",
        "id": "odometry",
        "text": "Returns the absolute X coordinate in millimeters."
    },
    {
        "page": "unibase.html",
        "title": "getAbsY",
        "id": "odometry",
        "text": "Returns the absolute Y coordinate in millimeters."
    },
    {
        "page": "unibase.html",
        "title": "getAbsAngle",
        "id": "odometry",
        "text": "Returns the absolute heading angle in degrees."
    },
    {
        "page": "unibase.html",
        "title": "resetDistance",
        "id": "odometry",
        "text": "Resets the accumulated distance counter to 0. Used for relative measurements."
    },
    {
        "page": "unibase.html",
        "title": "getDistance",
        "id": "odometry",
        "text": "Returns the distance traveled since the last reset in millimeters."
    },
    {
        "page": "unibase.html",
        "title": "resetAngle",
        "id": "odometry",
        "text": "Resets the accumulated angle counter to 0."
    },
    {
        "page": "unibase.html",
        "title": "getAngle",
        "id": "odometry",
        "text": "Returns the angle turned since the last reset in degrees."
    },
    {
        "page": "unibase.html",
        "title": "getLeftTicks / getRightTicks",
        "id": "odometry",
        "text": "Returns the raw encoder tick count for the respective motor."
    },
    {
        "page": "unibase.html",
        "title": "printOdometry",
        "id": "odometry",
        "text": "Prints the current X, Y, and Theta values to the Serial Monitor. Useful for debugging."
    },
    {
        "page": "unibase.html",
        "title": "displayPrint (Simple)",
        "id": "display",
        "text": "Clears the OLED screen and prints the provided value in the center. Supports multiple types."
    },
    {
        "page": "unibase.html",
        "title": "displayPrint (Named)",
        "id": "display",
        "text": "Prints a name label at the top of the screen and a large value in the center. Supports all\n                            simple types as the value."
    },
    {
        "page": "unibase.html",
        "title": "displayClear",
        "id": "display",
        "text": "Clears the OLED screen."
    },
    {
        "page": "unibase.html",
        "title": "getBatteryPower",
        "id": "utility",
        "text": "Returns current battery level percentage (0-100). Returns -1 if offline or error."
    },
    {
        "page": "unibase.html",
        "title": "blinkLED",
        "id": "utility",
        "text": "Sets the background task to blink the onboard LED."
    },
    {
        "page": "unibase.html",
        "title": "UniBaseControl",
        "id": "utility",
        "text": "Activates the UART control loop. This is a blocking function calling `ctrlReceiveUART()`\n                            repeatedly."
    },
    {
        "page": "unidev.html",
        "title": "UniDev Reference",
        "id": "",
        "text": "Class for controlling sensors, addressable LEDs (Light Ring), and other peripherals."
    },
    {
        "page": "unidev.html",
        "title": "Port Definitions",
        "id": "constants",
        "text": "Pre-defined constants for connecting modules to correct ports."
    },
    {
        "page": "unidev.html",
        "title": "Initialization",
        "id": "dev-init",
        "text": "These are blocking animations."
    },
    {
        "page": "unidev.html",
        "title": "Sensors",
        "id": "sensors",
        "text": "These are blocking animations."
    },
    {
        "page": "unidev.html",
        "title": "Inputs",
        "id": "inputs",
        "text": "These are blocking animations."
    },
    {
        "page": "unidev.html",
        "title": "Light Ring (Basic)",
        "id": "light-ring-basic",
        "text": "These are blocking animations."
    },
    {
        "page": "unidev.html",
        "title": "Light Ring (Effects)",
        "id": "light-ring-effects",
        "text": "These are blocking animations."
    },
    {
        "page": "unidev.html",
        "title": "Traffic Light",
        "id": "traffic-light",
        "text": ""
    },
    {
        "page": "unidev.html",
        "title": "begin",
        "id": "dev-init",
        "text": "Initializes the UniDev instance, creating the NeoPixel object and setting pin modes."
    },
    {
        "page": "unidev.html",
        "title": "ultraSonic",
        "id": "sensors",
        "text": "Reads distance from an ultrasonic sensor (HC-SR04)."
    },
    {
        "page": "unidev.html",
        "title": "lineSensor",
        "id": "sensors",
        "text": "Reads an analog value from a line sensor."
    },
    {
        "page": "unidev.html",
        "title": "digitalSensor",
        "id": "sensors",
        "text": "Reads a generic digital value from a port."
    },
    {
        "page": "unidev.html",
        "title": "analogSensor",
        "id": "sensors",
        "text": "Reads a generic analog value from a port."
    },
    {
        "page": "unidev.html",
        "title": "getPinMode",
        "id": "sensors",
        "text": "Returns the current configured mode of a GPIO pin."
    },
    {
        "page": "unidev.html",
        "title": "waitButton",
        "id": "inputs",
        "text": "Blocking function that halts program execution until the specified button is pressed and\n                            released."
    },
    {
        "page": "unidev.html",
        "title": "getButtonState",
        "id": "inputs",
        "text": "Checks if a button is currently pressed."
    },
    {
        "page": "unidev.html",
        "title": "pixel",
        "id": "light-ring-basic",
        "text": "Sets the color of a single pixel in the memory buffer. RequirespixelsShow()to\n                            take effect."
    },
    {
        "page": "unidev.html",
        "title": "pixelsAll",
        "id": "light-ring-basic",
        "text": "Sets all LEDs in the ring to the specified RGB color. Implicitly callsshow()."
    },
    {
        "page": "unidev.html",
        "title": "pixelsClear",
        "id": "light-ring-basic",
        "text": "Turns off all LEDs."
    },
    {
        "page": "unidev.html",
        "title": "pixelsShow",
        "id": "light-ring-basic",
        "text": "Sends the current buffer to the LED strip. Must be called afterpixel()."
    },
    {
        "page": "unidev.html",
        "title": "pixelsBrightness",
        "id": "light-ring-basic",
        "text": "Sets the global brightness scaling."
    },
    {
        "page": "unidev.html",
        "title": "pixelsRainbow",
        "id": "light-ring-effects",
        "text": "Displays a rotating rainbow animation."
    },
    {
        "page": "unidev.html",
        "title": "pixelsRunning",
        "id": "light-ring-effects",
        "text": "Running light effect around the ring."
    },
    {
        "page": "unidev.html",
        "title": "pixelsBreathing",
        "id": "light-ring-effects",
        "text": "Pulse/Breathing effect with the specified color."
    },
    {
        "page": "unidev.html",
        "title": "pixelsFill",
        "id": "light-ring-effects",
        "text": "Gradually fills the ring with the color."
    },
    {
        "page": "unidev.html",
        "title": "pixelsSparkle",
        "id": "light-ring-effects",
        "text": "Randomly flashes pixels."
    },
    {
        "page": "unidev.html",
        "title": "pixelsRotating",
        "id": "light-ring-effects",
        "text": "Rotates a segment of LEDs around the ring."
    },
    {
        "page": "unidev.html",
        "title": "pixelsSpinner",
        "id": "light-ring-effects",
        "text": "Shows a spinner animation."
    },
    {
        "page": "unidev.html",
        "title": "setTrafficLight",
        "id": "traffic-light",
        "text": "Sets the traffic light module state."
    },
    {
        "page": "unidev.html",
        "title": "trafficLightSequence",
        "id": "traffic-light",
        "text": "Runs a standard standard traffic light sequence (Green -> Yellow -> Red -> Yellow -> Green)."
    },
    {
        "page": "getting-started.html",
        "title": "Getting Started",
        "id": "",
        "text": "Follow these steps to set up the UNI library and start programming your robot."
    },
    {
        "page": "getting-started.html",
        "title": "Installation",
        "id": "installation",
        "text": "Here is a minimal example to verify that your environment is set up correctly. This sketch\n                    initializes the robot and prints \"Hello"
    },
    {
        "page": "getting-started.html",
        "title": "Hello Robot",
        "id": "hello-robot",
        "text": "Here is a minimal example to verify that your environment is set up correctly. This sketch\n                    initializes the robot and prints \"Hello"
    },
    {
        "page": "getting-started.html",
        "title": "Uploading to ESP32",
        "id": "uploading",
        "text": "Make sure you have theESP32 Board Packageinstalled in your Arduino IDE."
    },
    {
        "page": "examples.html",
        "title": "Examples",
        "id": "",
        "text": "Learn by example. Here are some common snippets to get you started."
    },
    {
        "page": "examples.html",
        "title": "Basic Initialization",
        "id": "basic",
        "text": "The simplest way to initialize the robot (from Start.ino)."
    },
    {
        "page": "examples.html",
        "title": "Distance Sensors",
        "id": "distance",
        "text": "Reading values from ultrasonic sensors (from DistanceSensors.ino). Includes serial output."
    },
    {
        "page": "examples.html",
        "title": "Line Sensor",
        "id": "line",
        "text": "Reading values from a line sensor (from LineSensor.ino). Includes serial output."
    }
];
const searchInput = document.querySelector('.search-input');
const searchContainer = document.querySelector('.search-container');

if (searchInput) {
    const resultsContainer = document.createElement('div');
    resultsContainer.className = 'search-results';
    searchContainer.appendChild(resultsContainer);

    searchInput.addEventListener('input', (e) => {
        const query = e.target.value.toLowerCase();

        // Removed Local Filtering (it was hiding page content)

        // Global Search Logic
        if (query.length < 2) {
            resultsContainer.style.display = 'none';
            return;
        }

        const matches = searchIndex.filter(item =>
            item.title.toLowerCase().includes(query) ||
            item.text.toLowerCase().includes(query)
        );

        if (matches.length > 0) {
            resultsContainer.innerHTML = '';
            matches.slice(0, 10).forEach(match => {
                const div = document.createElement('div');
                div.className = 'search-result-item';
                div.innerHTML = `
                    <div class="result-title">${match.title}</div>
                    <div class="result-context">${match.page}</div>
                `;
                div.addEventListener('click', () => {
                    const target = match.id ? `${match.page}#${match.id}` : match.page;
                    window.location.href = target;
                });
                resultsContainer.appendChild(div);
            });
            resultsContainer.style.display = 'block';
        } else {
            resultsContainer.innerHTML = '<div class="search-no-results">No results found</div>';
            resultsContainer.style.display = 'block';
        }
    });

    document.addEventListener('click', (e) => {
        if (!searchContainer.contains(e.target)) {
            resultsContainer.style.display = 'none';
        }
    });
}
