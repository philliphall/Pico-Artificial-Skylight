/*
  Project: Skylight LED Controller
  Description: 
    This code simiulates natural daylight using an LED panel (in my case, 
    a free 85" TV with a broken LCD panel removed!), adjusting its brightness based 
    on the solar elevation calculated from the user's geographical location. It 
    supports multiple modes, including manual dimming via a potentiometer and 
    automatic skylight simulation with different brightness curves. The system also 
    includes an OLED display for real-time feedback and a real-time clock for 
    accurate solar position calculations.

  Hardware:
    - RP2040 (I'm using the 16mb black one from AliExpress mimicing the Pico)
    - Real-Time Clock (RTC) DS3231
    - OLED Display (SSD1306)
    - Potentiometer
    - LED Panel (PWM-controlled)
    - Suitable power supply (could be pulled from the TV, but don't overvolt the RP2040)
  
  Features:
    - Dynamic brightness adjustment based on solar elevation
    - Manual and automatic control modes
    - OLED feedback for mode and brightness levels
    - Persistent storage of settings
    - Debugging support with adjustable verbosity

  Modes:
    - Manual Dimming: Control brightness with a potentiometer
    - Skylight Mode 1: Simple brightness curve
    - Skylight Mode 2: Complex, realistic solar simulation
    - Photo Match Mode: Adjusts brightness to match external light (experimental)
    - Demo Mode: Cycles through all brightness levels to test LED functionality

  Libraries Used:
    - Start with the Arduino-Pico board framework
    - RTClib.h for RTC management
    - Adafruit_SSD1306.h for OLED display handling
    - Wire.h for I2C communication
    - LittleFS.h for non-volatile memory storage
    - SolarCalculator.h for calculating solar position

  TO DO:
    - Improve Time Setting Mode. I'd like more granularity in speed of increment, possibly by using modulo of delta section as a delay divisor.
    - Modularize by hardware. Be able to compile without a display, for example. Or without a potentiometer, or button, or RTC module. 
    - Split the code into multiple files for easier maintainability. 
*/

#include <LittleFS.h>
#include <SolarCalculator.h>
#include <RTClib.h>
#include <Wire.h>
#include <Math.h>
#include <Adafruit_SSD1306.h>
#include "hardware/pwm.h"
RTC_DS3231 rtc;

// Configuration Variables
const float latitude = 34.21316116274671;   // Hardcode your current location for skylight mode
const float longitude = -84.54894616203148; // Hardcode your current location for skylight mode
uint16_t daylight_value = 55000;         // Compared to full range of 0-65535 - my light is way too bright for indoors
uint16_t min_illum = 25;                 // Because some displays really don't like being super low on the PWM signal.
float currentPWMFrequency = 244;         // Initial default PWM frequency in Hz
#define PHOTO_OFFSET -50                 // The photoresistors aren't identical, and their connections and cord lengths aren't identical. The external reading will be adjusted by this much.
#define DIMMING_TIME 30000               // OLED dimming time in milliseconds (30 seconds)
#define DIMMED_BRIGHTNESS 5              // OLED dimmed brightness as a percentage (5%)
#define OFF_TIME 300000                  // OLED Off time in milliseconds (5 minutes)
#define SKY_CHECK_INTERVAL 10000         // Used in mode 1&2 - only update skylight every xx milliseconds
#define OLED_UPDATE_INTERVAL 500         // Milliseconds between OLED display updates
#define BUTTON_DEBOUND_DELAY 50          // Debounce time in milliseconds
#define USE_ONBOARD_LED                  // Whether or not to match the onboard LED to the PWM-controlled panel.

// Enabled Modes -  Only uncomment the modes you want available (and that you have the hardware for!).
#define EN_MODE_POTENTIOMETER            // At this time, a potentiometer is still required for time setting (and I have yet to make compiliation without that functionality an option)
#define EN_MODE_SKYLIGHT1                // A piecewise brightness curve that just seemed to match what I wanted. Results in softer daylight brightness.
#define EN_MODE_SKYLIGHT2                // An advanced, mathematical, and closest to reality brightness. 
//#define EN_MODE_PHOTO_MATCH            // This mode leverages photosensors - one viewing the outside sky and another viewing the area under the LED panel and strives to make them match. This is the least tested and developed mode today. 
#define EN_MODE_DEMO                     // A simple slow (but scaled) progression to test all valid PWM values - useful to finding the best min_illum and daylight_value.
#define EN_MODE_SERIAL                   // Mode for Serial Control

// Pin Designations
#define LED_PIN 14                       // 16-bit PWM for LED control
#define ONBOARD_LED 25                   // Optionally, we can duplicate this brightness to the onboard LED. Only used if USE_ONBOARD_LED is also defined. 
#define MODE_SW 16                       // Mode switch button
#define PHOTO_INT_PIN 28                 // Internal photoresistor
#define PHOTO_EXT_PIN 27                 // External photoresistor
#define POT_PIN 26                       // Potentiometer wiper input

// OLED Setup
#define SCREEN_WIDTH 128                 // OLED display width, in pixels
#define SCREEN_HEIGHT 32                 // OLED display height, in pixels
#define OLED_RESET    -1                 // Reset pin # (-1 is holdover from sharing with Arduino reset pin)
// SDA 4 (default pin)
// SCL 5 (default pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Debugging setup
#define DEBUG_NONE 0
#define DEBUG_ERROR 1
#define DEBUG_WARNING 2
#define DEBUG_INFO 3
#define DEBUG_VERBOSE 4
#define DEBUG_NEVER 9 // This is really only a placeholder. If you want to see this information, change the debug level on the specific line of code from NEVER to something else.

#define DEBUG_LEVEL DEBUG_INFO        // Set the debug level here

#define DEBUG_PRINT(level, message) \
    do { if (DEBUG_LEVEL >= level) Serial.print(message); } while (0)
#define DEBUG_PRINTLN(level, message) \
    do { if (DEBUG_LEVEL >= level) Serial.println(message); } while (0)

// Modes numbering
#define MODE_POTENTIOMETER 0
#define MODE_SKYLIGHT1 1
#define MODE_SKYLIGHT2 2
#define MODE_PHOTO_MATCH 3
#define MODE_DEMO 4
#define MODE_SERIAL 5

// Modes Display Names
#define MODE_POTENTIOMETER_NAME "Manual Dimming"
#define MODE_SKYLIGHT1_NAME "Skylight Mode 1"
#define MODE_SKYLIGHT2_NAME "Skylight Mode 2"
#define MODE_PHOTO_MATCH_NAME "Brightness Match"
#define MODE_DEMO_NAME "Demo Mode"
#define MODE_SERIAL_NAME "Serial Control Mode"

// Global Scope Declarations
unsigned long currentMillis = 0;         // stores the value of millis() in each iteration of loop()
unsigned long previousMillis = 0;        // for various timings - mode dependent
unsigned long lastInteractionMillis = 0; // Global variable to track the last interaction time for OLED dimming
uint8_t currentMode = 0;                 // Initial mode. 0-Manual, 1-Skylight1, 2-Skylight2, 3-PhotoMatch, 4-Demo, 5-Serial
uint16_t led_value = min_illum;          // 16-bit LED brightness control
bool isDimmed = false;                   // Variable to track the current display brightness state
bool isOff = false;                      // Variable to track the current display state
#define MAX_INPUT_SIZE 32                // Limit for serial command input

// Global variables for Manual potentiometer dimming
#ifdef EN_MODE_POTENTIOMETER
  int potval;                            // Used in mode 0 - value read from the potentiometer on pot_pin
  int previousInteractionPotval = 0;     // Used to change to manual mode from other modes
  int previousModePotval = 0;            // New variable to track potentiometer value for mode changes
  #define AVERAGE_BUFFER_SIZE 100        // Number of readings to average
  int potReadings[AVERAGE_BUFFER_SIZE];  // Array to store the potentiometer readings
  int readIndex = 0;                     // Index for the next reading
  long total = 0;                        // Running total of the readings
  int average = 0;                       // Average of the readings
#endif

// Global variables used in Skylight Mode 1
#ifdef EN_MODE_SKYLIGHT1
  float relative_brightness;
#endif

// Global variables used in Skylight Mode 2
#ifdef EN_MODE_SKYLIGHT2
  #define MATCH_ELEVATION 1              // Elevation angle at which we switch between twilight and daylight models. Don't recommend you change it.
  #define ASTRONOMICAL_DECAY_CONSTANT 0.15 // For how slow the twilight transformation goes
  #define DIFFUSE_SCALING_FACTOR 1       // Will be multiplied. 1 means no change. Higher emphasises the impact of ambient reflection of light
  float match_factor;
  float max_combined_level = 0;
#endif

// Global variables used in Photo Match Mode
#ifdef EN_MODE_PHOTO_MATCH
  int photo_int;                         // Used in mode 3 - value read from the indoor photoresistor
  int photo_ext;                         // Used in mode 3 - value read from the outdoor photoresistor
#endif

// Global variables used in Demo Mode
#ifdef EN_MODE_DEMO
  boolean demoDirectionUp = true;        // Used in mode 4 - state variable
  uint8_t demoCounter = 255;             // Used in mode 4 - increment the 8-bit PWM value once every 256 iterations of the 16-bit loop
#endif

// Function Prototypes - 
// NOTE: All mode handlers need to be defined regardless of inclusion 
//  so that button presses can still call them and result in cycling to the next mode.
//  Their supporting functions can be conditional though. 

// FUnction Prototypes - Potentiometer Mode
void handlePotentiometerMode(bool forceUpdate = false);  
#ifdef EN_MODE_POTENTIOMETER
  // Support funcions sepecific to this mode entered here
#endif

// Function Prototypes - Skylight Mode 1
void handleSkylightMode1(bool forceUpdate = false);
#ifdef EN_MODE_SKYLIGHT1
  // Support funcions sepecific to this mode entered here
#endif

// Function Prototypes - Skylight Mode 2
void handleSkylightMode2(bool forceUpdate = false);
#ifdef EN_MODE_SKYLIGHT2
  float direct_sunlight_brightness(float elevation, float I0 = 1361, float k = 0.2);
  float diffuse_sky_brightness(float elevation, float a = -1, float b = -0.32, float c = 0.43, float d = 1.25, float e = -0.35);
  float astronomical_twilight_brightness(float elevation);
  float combined_daytime_brightness(float elevation);
  float combined_brightness(float elevation);
#endif

// Function Prototypes - Photo Match Mode
void handlePhotoresistorMatchMode(bool forceUpdate = false);
#ifdef EN_MODE_POTENTIOMETER
  // Support funcions sepecific to this mode entered here
#endif

// Function Prototypes - Demo Mode
void handleDemoMode(bool forceUpdate = false);
#ifdef EN_MODE_DEMO 
  unsigned long getNextDemoInterval(uint16_t led_value, bool goingUp);
#endif

// Function Prototypes - Serial Mode
void handleSerialControlMode(bool forceUpdate = false);
#ifdef EN_MODE_SERIAL
  void process_serial();
  void setPWMFrequency(float frequency);
  void setPWMDutyCycle(uint16_t dutyCycle);
#endif

// Function Prototypes - Utility functions
void saveSetting(); // Using LittleFS library for wear-leveled persistent settings storage
bool loadSetting();
void writePWM(uint16_t led_value);
void pwm_init();
uint8_t gammaCorrection8(uint8_t led_value, float gamma = 2.2); // No more 8-bit PWM any more, but still need this for 8-bit OLED display contrast correction
uint16_t gammaCorrection16(uint16_t led_value, float gamma = 2.2);
const char* getModeName(uint8_t currentMode); 

// Function Prototypes - Button and mode cycling
bool readDebouncedButton(int pin);
void readButton();
void cycleModes();
void executeMode(bool forceUpdate=false);

// Function Prototypes - Time setting functions
void enterDateTimeSettingMode();
void waitForMiddle();
void adjustDate(DateTime &now);
void addDate(DateTime &date, int daysToAdd, int monthsToAdd, int yearsToAdd);
void adjustTime(DateTime &now);

// Function Prototypes - Display
void updateDisplay(float elevation, uint16_t led_value, bool forceUpdate = false);
void displayDateSetting(const DateTime &now);
void displayTimeSetting(const DateTime &now);


// *** SETUP ***
void setup() {
  Serial.begin(115200);
  // Wait for the serial port to connect to capture setup messages (with a timeout to avoid getting stuck)
  if (DEBUG_LEVEL >= DEBUG_VERBOSE) {
    unsigned long startMillis = millis();
    while (!Serial && (millis() - startMillis < 5000)) {
      delay(10);
    }
  }
  Serial.println("Serial started");
  DEBUG_PRINTLN(DEBUG_INFO, "Setup started");

  // Initialize PWM for LED pins
  pwm_init();
  
  // Mode switch setup
  pinMode(MODE_SW, INPUT_PULLUP);
  
  // Restore last settings from LittleFS
  if (!LittleFS.begin()) {
    DEBUG_PRINTLN(DEBUG_WARNING, "Failed to mount LittleFS");
    if (!LittleFS.format()) {
      DEBUG_PRINTLN(DEBUG_WARNING, "Failed to format LittleFS.");
    }
    if (!LittleFS.begin()) {
      DEBUG_PRINTLN(DEBUG_ERROR, "Failed to mount LittleFS after attempting to format.");
    }
  }
  if (!loadSettings()) {
    DEBUG_PRINTLN(DEBUG_WARNING, "No settings found, using default values.");
  }
  
  // OLED Begin
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    DEBUG_PRINTLN(DEBUG_ERROR, "SSD1306 allocation failed");
    Serial.flush();
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.cp437(true);
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(255); // Set initial brightness to 100%

  // Time Initialization
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC, using compile time as default.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to compile time if RTC is not found
  }
  else if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to compile time if RTC lost power
  }
  DateTime now = rtc.now();
  if (now.year() < 2021 || now.year() > 2124) { // Check if the date seems reasonable
    DEBUG_PRINTLN(DEBUG_ERROR, "RTC is returning a value but the time is not set correctly");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to the compile time
    //enterDateTimeSettingMode();
  }
  
  // Used in EN_MODE_POTENTIOMETER
  #ifdef EN_MODE_POTENTIOMETER
    for (int thisReading = 0; thisReading < AVERAGE_BUFFER_SIZE; thisReading++) { // Init Debounce Array
      potReadings[thisReading] = 0; 
    }
    previousInteractionPotval = analogRead(POT_PIN); // So we don't immediately switch to manual on first run.
    previousModePotval = previousInteractionPotval;
  #endif

  // Used in Skylight Mode 1 - Linear LED Brightness
  #ifdef EN_MODE_SKYLIGHT1
    relative_brightness = (65535 * log10(2)) / log10(65535);
  #endif

  // Used in Skylight Mode 2 - Calculate combined and twilight light intensities at a matching point
  #ifdef EN_MODE_SKYLIGHT2
    // Calculate the match factor - so we have a smooth transition between twilight and daylight modes
    match_factor = combined_daytime_brightness(MATCH_ELEVATION + 0.001) / astronomical_twilight_brightness(MATCH_ELEVATION - 0.001);
    
    // Calculate the max and min values of the output of our curve
    for (float elevation = -20; elevation <= 90; elevation += 1) {
        float combined_level = combined_brightness(elevation);
        if (combined_level > max_combined_level) {
            max_combined_level = combined_level;
        }
    }
  #endif

  // Track initial interaction time
  lastInteractionMillis = millis();

  Serial.println("Ending Setup()");
  Serial.flush();
}

// *** MAIN LOOP ***
void loop() {  
  currentMillis = millis();              // capture the latest value of millis()
  potval = analogRead(POT_PIN);          // Read current potentiometer value only once per loop
  
  // Start by looking for a mode change
  readButton();                          // Checks and handles button press
  
  // Check for larger potentiometer changes to detect change
  #ifdef EN_MODE_POTENTIOMETER
    if (currentMode != MODE_POTENTIOMETER && abs(potval - previousModePotval) > 50) {
      currentMode = MODE_POTENTIOMETER;  // Dimmer moved - switch to manual dimming mode
      previousMillis = 0;                // Always reset on mode change
      DEBUG_PRINTLN(DEBUG_WARNING, "Manual Override - Entering mode 0 - Manual Dimming");
      saveSettings();                    // Save new mode to persistent storage whenever it changes
      previousModePotval = potval;       // Reset  
      lastInteractionMillis = currentMillis; // Reset interaction timer
    }
  #endif 

  // Check smaller potentiometer changes to reset interaction timer
  if (abs(potval - previousInteractionPotval) > 25) { // Threshold for considering it an interaction
    if (isDimmed || isOff) { // Re-activate the display if it was dimmed or turned off
      display.ssd1306_command(SSD1306_SETCONTRAST);
      display.ssd1306_command(255);
      isDimmed = false;
      isOff = false;
      updateDisplay(-1, led_value, true);
    }
    previousInteractionPotval = potval;     // Reset
    lastInteractionMillis = currentMillis;  // Reset last interaction timer
  }
  
  // Dimming logic
  if ((currentMillis - lastInteractionMillis >= DIMMING_TIME) && !isDimmed) {
    DEBUG_PRINTLN(DEBUG_INFO, "Dimming OLED now.");
    uint8_t correctedDimmedBrightness = gammaCorrection8(map(DIMMED_BRIGHTNESS, 0, 100, 0, 255));
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(correctedDimmedBrightness);
    isDimmed = true;
  }

  // Off timer logic
  if ((currentMillis - lastInteractionMillis >= OFF_TIME) && !isOff) {
    DEBUG_PRINTLN(DEBUG_INFO, "Turning off OLED now.");
    display.clearDisplay();
    display.display();
    isOff = true;
  }
  
  // Process any incoming Serial data
  #ifdef EN_MODE_SERIAL
    process_serial();
  #endif

  // Execute the current mode’s function
  executeMode(false);
}

// Basic manual dimmer mode with gamma correction
void handlePotentiometerMode(bool forceUpdate) {
  #ifdef EN_MODE_POTENTIOMETER
    // Debounce
    total = total - potReadings[readIndex];            // Subtract the oldest reading from the total
    potReadings[readIndex] = potval;                   // Add the current potentiometer value and store it into the array
    total = total + potReadings[readIndex];            // Add the new reading to the total
    readIndex = (readIndex + 1) % AVERAGE_BUFFER_SIZE; // Advance to the next position in the array
    average = total / AVERAGE_BUFFER_SIZE;             // Calculate the average
    
    // Map the average potentiometer value to the 16-bit range
    uint16_t mapped_value = map(average, 0, 1023, 0, 65535);
    
    // Apply gamma correction
    led_value = gammaCorrection16(mapped_value);

    // Write the corrected values to the LED pins
    writePWM(led_value);
    updateDisplay(-1, led_value, forceUpdate);

    // Update debug, based on timing
    if (currentMillis - previousMillis > 500 || forceUpdate == true) { // Only debug every half second
      previousMillis = currentMillis;
      
      // Consolidated debug output
      DEBUG_PRINT(DEBUG_VERBOSE, "Potval: ");
      DEBUG_PRINT(DEBUG_VERBOSE, potval);
      DEBUG_PRINT(DEBUG_VERBOSE, " | Avg: ");
      DEBUG_PRINT(DEBUG_VERBOSE, average);
      DEBUG_PRINT(DEBUG_VERBOSE, " | Mapped 16-bit: ");
      DEBUG_PRINT(DEBUG_VERBOSE, mapped_value);
      DEBUG_PRINT(DEBUG_VERBOSE, " | LED 16-bit: ");
      DEBUG_PRINTLN(DEBUG_VERBOSE, led_value);  
    }
  #else
    cycleModes();
  #endif
}

// *** Artificial Skylight Mode 1 - Modeled
void handleSkylightMode1(bool forceUpdate) {
  #ifdef EN_MODE_SKYLIGHT1
    if (currentMillis - previousMillis >= SKY_CHECK_INTERVAL || forceUpdate == true) {
      previousMillis = currentMillis;
      DEBUG_PRINTLN(DEBUG_INFO, "In Skylight Mode 1");

      // Get the sun's current position
      double azimuth, elevation;
      DateTime now = rtc.now();
      calcHorizontalCoordinates(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), latitude, longitude, azimuth, elevation);

      DEBUG_PRINT(DEBUG_INFO, "Elevation value: ");
      DEBUG_PRINTLN(DEBUG_INFO, elevation);

      // Non-linear brightness growth of the sun based on elevation angle
      if (elevation >= 40) {
        led_value = daylight_value;
        led_value = pow(2, (led_value / relative_brightness));
      } else if (elevation >= 15) {
        led_value = map(elevation * 100, 15 * 100, 40 * 100, .85 * daylight_value, daylight_value);
        led_value = pow(2, (led_value / relative_brightness));
      } else if (elevation >= 3) {
        led_value = map(elevation * 100, 3 * 100, 15 * 100, .76 * daylight_value, .85 * daylight_value);
        led_value = pow(2, (led_value / relative_brightness));
      } else if (elevation >= 0) {
        led_value = map(elevation * 100, 0 * 100, 3 * 100, .7 * daylight_value, .76 * daylight_value);
        led_value = pow(2, (led_value / relative_brightness));
      } else if (elevation >= -3) {
        led_value = map(elevation * 100, -3 * 100, 0 * 100, .6 * daylight_value, .7 * daylight_value);
        led_value = pow(2, (led_value / relative_brightness));
        led_value += map(elevation * 100, -3 * 100, 0 * 100, min_illum, 0);
      } else if (elevation >= -12) {
        led_value = map(elevation * 100, -12 * 100, -3 * 100, 0, .6 * daylight_value);
        led_value = pow(2, (led_value / relative_brightness));
        led_value += min_illum;
      } else {
        led_value = 0;
      }

      // Guard rails
      if (led_value > 0 && led_value < min_illum / 2) led_value = 0; // LED stays off until halfway to min_illum
      else if (led_value >= min_illum / 2 && led_value < min_illum) led_value = min_illum / 2; // LED turns on halfway
      if (led_value > daylight_value) led_value = daylight_value;

      // Write it
      writePWM(led_value);
      updateDisplay(elevation, led_value, forceUpdate);
      DEBUG_PRINT(DEBUG_VERBOSE, "LED Value: ");
      DEBUG_PRINTLN(DEBUG_VERBOSE, led_value);
    }
  #else
    cycleModes();
  #endif
}

// Artificial Skylight Mode 2
#ifdef EN_MODE_SKYLIGHT2
  // Function to calculate the intensity of direct sunlight based on solar elevation using the Solar Air Mass model
  float direct_sunlight_brightness(float elevation, float I0, float k) {
      if (elevation > MATCH_ELEVATION) {
          float zenith_angle = 90 - elevation;
          float AM = 1 / (cos(radians(zenith_angle)) + 0.50572 * pow((96.07995 - zenith_angle), -1.6364));
          return I0 * exp(-k * AM);
      } else {
          return 0;
      }
  }

  // Function to calculate diffuse sky light using the simplified Perez model
  float diffuse_sky_brightness(float elevation, float a, float b, float c, float d, float e) {
      if (elevation < MATCH_ELEVATION) {
          return 0;
      } else {
          float zenith_luminance = 100; // Assumed zenith luminance for a typical clear day
          return zenith_luminance * (1 + a * exp(b / (cos(radians(90 - elevation)) + 0.01)));
      }
  }

  // Function to calculate sky brightness during astronomical twilight using an exponential decay model
  float astronomical_twilight_brightness(float elevation) {
      if (elevation > MATCH_ELEVATION) {
          return 0; // Daylight, no twilight effect
      } else {
          // Exponential decay of light intensity from sunset to end of astronomical twilight
          return exp(ASTRONOMICAL_DECAY_CONSTANT * elevation); // Adjust the decay rate to fit observational data
      }
  }

  // Function to scale and combine direct and diffuse daytime light components
  float combined_daytime_brightness(float elevation) {
      float direct_component = direct_sunlight_brightness(elevation);
      float diffuse_component = diffuse_sky_brightness(elevation);
      return direct_component + (diffuse_component * DIFFUSE_SCALING_FACTOR);
  }

  // Function to scale and combine the twilight and daylight components
  float combined_brightness(float elevation) {
      float combined_daytime_level = combined_daytime_brightness(elevation);
      float astronomical_twilight_level = astronomical_twilight_brightness(elevation);
      return combined_daytime_level + (astronomical_twilight_level * match_factor); // Match factor is determined in the setup() function.
  }
#endif

// And the main handler for this mode
void handleSkylightMode2(bool forceUpdate) {
  #ifdef EN_MODE_SKYLIGHT2
    if (currentMillis - previousMillis >= SKY_CHECK_INTERVAL || forceUpdate == true) {
      previousMillis = currentMillis;
      double azimuth, elevation;
      DEBUG_PRINTLN(DEBUG_INFO, "In Skylight Mode 2");

      // Get the sun's current position
      DateTime now = rtc.now();
      calcHorizontalCoordinates(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), latitude, longitude, azimuth, elevation);
      DEBUG_PRINT(DEBUG_INFO, "Elevation value: ");
      DEBUG_PRINTLN(DEBUG_INFO, elevation);

      // Using a whole series of nifty functions above to create a pretty great curve, I think!
      float combined_level = combined_brightness(elevation);

      // Map the result to our dimmer range
      led_value = map(combined_level, 0, max_combined_level, 0, 65535); // map algorithm value to 16-bit value

      // Gamma Correction
      led_value = gammaCorrection16(led_value);

      // Guard rails
      if (led_value > 0 && led_value < min_illum / 2) led_value = 0; // LED stays off until halfway to min_illum
      else if (led_value >= min_illum / 2 && led_value < min_illum) led_value = min_illum / 2; // LED turns on halfway
      if (led_value > daylight_value) led_value = daylight_value;
      
      // Write it
      writePWM(led_value);
      updateDisplay(elevation, led_value, forceUpdate);
      DEBUG_PRINT(DEBUG_VERBOSE, "LED Value: ");
      DEBUG_PRINTLN(DEBUG_VERBOSE, led_value);
    }
  #else
    cycleModes();
  #endif
}

// Photo Matching Mode
void handlePhotoresistorMatchMode(bool forceUpdate) {
  #ifdef EN_MODE_PHOTO_MATCH
    int photo_int = analogRead(PHOTO_INT_PIN);
    int photo_ext = analogRead(PHOTO_EXT_PIN) + PHOTO_OFFSET;
    if (photo_int > photo_ext) {
      //float dif;
      //dif = ((float)photo_int - (float)photo_ext) / 1023;
      //led_value -= ((int)((float)led_value * dif * .05))+1;
      led_value = max(led_value - 1, 0);
      //if ((led_value > 0) && (led_value < min_illum)) {led_value = min_illum;}
    } else if (photo_int < photo_ext) {
      //float dif;
      //dif = ((float)photo_ext - (float)photo_int) / 1023;
      //led_value += ((int)((float)led_value * dif *.05)+1);
      led_value = min(led_value + 1, 65535);
      if (led_value < min_illum) led_value = min_illum;
    }
    /*    else if (photo_int == photo_ext) {
      DEBUG_PRINT(DEBUG_VERBOSE, "Matched values! led_value: ");
      DEBUG_PRINT(DEBUG_VERBOSE, led_value);
      DEBUG_PRINT(DEBUG_VERBOSE, " - photo_ext & photo_int: ");
      DEBUG_PRINTLN(DEBUG_VERBOSE, photo_int);
    }
    else {
      DEBUG_PRINT(DEBUG_ERROR, "Reached the else clause in photoresistor match mode, which should be impossible! led_value: ");
      DEBUG_PRINT(DEBUG_ERROR, led_value);
    }
    */    
    writePWM(led_value);
    updateDisplay(-1, led_value, forceUpdate);

    // Write status
    if (currentMillis - previousMillis > 250 || forceUpdate == true) { // We don't want to print to serial/display at max speed, 4x per second should be fine.
      previousMillis = currentMillis;
      DEBUG_PRINT(DEBUG_VERBOSE, "---Status--- led_value: ");
      DEBUG_PRINT(DEBUG_VERBOSE, led_value);
      DEBUG_PRINT(DEBUG_VERBOSE, ", photo_ext: ");
      DEBUG_PRINT(DEBUG_VERBOSE, photo_ext);
      DEBUG_PRINT(DEBUG_VERBOSE, ", photo_int: ");
      DEBUG_PRINTLN(DEBUG_VERBOSE, photo_int);
    }
  #else
    cycleModes();
  #endif
}

void handleDemoMode(bool forceUpdate) {
  #ifdef EN_MODE_DEMO
    if (forceUpdate == true) {
      demoDirectionUp = true;
      led_value = 0;
      demoCounter = 255;
    }
    
    // Manage the LED intensity and update timing
    if (demoDirectionUp) {
      if (led_value < 65535 && currentMillis - previousMillis >= getNextDemoInterval(led_value, true)) {
        previousMillis = currentMillis;
        led_value++;
      } else if (led_value >= 65535) {
        demoDirectionUp = false;
        led_value--;
      }
    } else {
      if (led_value > 0 && currentMillis - previousMillis >= getNextDemoInterval(led_value, false)) {
        previousMillis = currentMillis;
        led_value--;
      } else if (led_value <= 0) {
        demoDirectionUp = true;
        led_value++;
      }
    }

    // Update PWM outputs to LED
    writePWM(led_value);
    updateDisplay(-1, led_value, forceUpdate);
  #else
    cycleModes();
  #endif
} // End Demo Mode

// Helper function to determine the next interval based on the current brightness and direction
unsigned long getNextDemoInterval(uint16_t led_value, bool goingUp) {
  #ifdef EN_MODE_DEMO
    if (goingUp) {
      if (led_value < 60) return 100;
      else if (led_value < 255) return 40;
      else if (led_value < 511) return 20;
      else return 10;
    } else {
      if (led_value > 511) return 10;
      else if (led_value > 255) return 20;
      else if (led_value > 40) return 40;
      else return 100;
    }
  #endif
  return -1; // Should not reach this
}

// Serial Control Mode
void handleSerialControlMode(bool forceUpdate) {
  // process_serial() really does all the handling, and is called every iteration of loop regardless of mode.
  #ifdef EN_MODE_SERIAL
    DEBUG_PRINTLN(DEBUG_NEVER, "Serial Control Mode active. Awaiting commands.");
    updateDisplay(-1, led_value, forceUpdate);
  #else
    // This should never occur since the only way to enter Serial mode is to hear a serial command which requires the listener to be enabled :-)
    cycleModes();
  #endif
}

// Process incoming serial commands for PWM adjustments
void process_serial() {
  static char inputBuffer[MAX_INPUT_SIZE];  // Buffer to hold input
  static byte index = 0;                    // Index for buffer
  
  // Check if there are any bytes available to read
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();      // Read incoming byte
    DEBUG_PRINT(DEBUG_VERBOSE, "Char received: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, incomingByte);

    // If newline character is received, process the input
    if (incomingByte == '\n') {
      inputBuffer[index] = '\0';            // Null-terminate the string

      // Switch to Serial Control Mode if not already in it
      if (currentMode != MODE_SERIAL) {
        currentMode = MODE_SERIAL;
        DEBUG_PRINTLN(DEBUG_INFO, "Entering Serial Control Mode");
      }

      // Debug: Show the received input
      DEBUG_PRINTLN(DEBUG_INFO, "Received serial command.");

      // Parse the command
      if (inputBuffer[0] == 'F') {
        float frequency = atof(&inputBuffer[1]); // Convert string to float
        if (frequency > 0) {
          setPWMFrequency(frequency);            // Valid frequency
          saveSettings();
        } else {
          DEBUG_PRINTLN(DEBUG_ERROR, "Invalid frequency value.");
        }
      }
      else if (inputBuffer[0] == 'D') {
        long dutyCycle = atol(&inputBuffer[1]);  // Convert string to integer
        if (dutyCycle >= 0 && dutyCycle <= 65535) {
          setPWMDutyCycle((uint16_t)dutyCycle);  // Valid duty cycle
          // NOT saved, because this is not intended to be a persistent setting, and I don't want to wear out flash by writing often.
        } else {
          DEBUG_PRINTLN(DEBUG_ERROR, "Invalid duty cycle value. Must be between 0 and 65535.");
        }
      }
      else if (inputBuffer[0] == 'I') {
        int newMinIllum = atoi(&inputBuffer[1]);
        if (newMinIllum >= 0 && newMinIllum <= 65535) {
          min_illum = newMinIllum;
          DEBUG_PRINT(DEBUG_INFO, "min_illum set to: ");
          DEBUG_PRINTLN(DEBUG_INFO, min_illum);
          saveSettings();
        } else {
          DEBUG_PRINTLN(DEBUG_ERROR, "Invalid min_illum value. Must be between 0 and 65535.");
        }
      }
      else if (inputBuffer[0] == 'L') {
        int newDaylightValue = atoi(&inputBuffer[1]);
        if (newDaylightValue >= 0 && newDaylightValue <= 65535) {
          daylight_value = newDaylightValue;
          DEBUG_PRINT(DEBUG_INFO, "daylight_value set to: ");
          DEBUG_PRINTLN(DEBUG_INFO, daylight_value);   
          saveSettings();       
        } else {
          DEBUG_PRINTLN(DEBUG_ERROR, "Invalid daylight_value. Must be between 0 and 65535.");
        }
      }
      else if (inputBuffer[0] == 'M') {
        int mode = atoi(&inputBuffer[1]);
        if (mode >= 0 && mode <= 5) {
          currentMode = mode;
          executeMode(true);
          DEBUG_PRINT(DEBUG_INFO, "Mode set to: ");
          DEBUG_PRINTLN(DEBUG_INFO, getModeName(currentMode));
          saveSettings();
        } else {
          DEBUG_PRINTLN(DEBUG_ERROR, "Invalid mode value. Must be between 0 and 5.");
        }
      }
      else if (inputBuffer[0] == 'P') { // Show current settings
        DEBUG_PRINT(  DEBUG_INFO, "Current Settings: \n");
        DEBUG_PRINT(  DEBUG_INFO, " - PWM Frequency:    ");
        DEBUG_PRINTLN(DEBUG_INFO,                       currentPWMFrequency);
        DEBUG_PRINT(  DEBUG_INFO, " - Min Illumination: ");
        DEBUG_PRINTLN(DEBUG_INFO,                       min_illum);
        DEBUG_PRINT(  DEBUG_INFO, " - Daylight Value:   ");
        DEBUG_PRINTLN(DEBUG_INFO,                       daylight_value);
        DEBUG_PRINT(  DEBUG_INFO, " - Current Mode:     ");
        DEBUG_PRINTLN(DEBUG_INFO,                       getModeName(currentMode));
      } else {
        DEBUG_PRINTLN(DEBUG_ERROR, "Invalid command. Use 'F' for frequency, 'D' for duty cycle, 'I' for min_illum, 'L' for daylight_value, 'M' for currentMode, 'P' to show settings.");
      }
      
      // Finalize and reset after processing input
      index = 0;
      updateDisplay(-1, led_value, true); // Force update display with new settings
    } 
    else {
      // Only add to the buffer if we haven't reached the maximum buffer size
      if (index < MAX_INPUT_SIZE - 1) {
        inputBuffer[index++] = incomingByte;   // Add incoming byte to buffer
      } else {
        // If buffer overflows, reset index and print an error message
        DEBUG_PRINTLN(DEBUG_ERROR, "Input buffer overflow. Please enter shorter commands.");
        index = 0;
      }
    } // End else (not a newline)
  } // End while loop
} // End process_serial definition

// Set PWM frequency based on user input
void setPWMFrequency(float frequency) {
  // Update the global variable
  currentPWMFrequency = frequency;

  // Make changes
  uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
  float clkdiv = (float) clock_get_hz(clk_sys) / (currentPWMFrequency * 65535.0); // Calculate the clock divider
  pwm_set_clkdiv(slice_num, clkdiv);

  // Feedback
  DEBUG_PRINT(DEBUG_INFO, "PWM Frequency set to: ");
  DEBUG_PRINT(DEBUG_INFO, currentPWMFrequency);
  DEBUG_PRINT(DEBUG_VERBOSE, ", with clkdiv value: ");
  DEBUG_PRINT(DEBUG_VERBOSE, clkdiv); 
  DEBUG_PRINTLN(DEBUG_INFO, ".");
}

// Set PWM duty cycle based on user input
void setPWMDutyCycle(uint16_t dutyCycle) {
  led_value = dutyCycle;  // Update global led_value
  writePWM(led_value);    // Use existing function to apply the change
  DEBUG_PRINT(DEBUG_INFO, "PWM Duty Cycle set to: ");
  DEBUG_PRINTLN(DEBUG_INFO, dutyCycle);
}

// Persisten settings storage using LittleFS
void saveSettings() {
  File file = LittleFS.open("/settings.txt", "w");
  if (!file) {
    DEBUG_PRINTLN(DEBUG_ERROR, "Failed to open file for writing");
    return;
  }
  file.printf("PWMFrequency=%f\n", currentPWMFrequency);
  file.printf("MinIllum=%u\n", min_illum);
  file.printf("DaylightValue=%u\n", daylight_value);
  file.printf("CurrentMode=%u\n", currentMode);
  file.close();
  DEBUG_PRINTLN(DEBUG_INFO, "Settings saved to LittleFS");
  DEBUG_PRINT(  DEBUG_INFO, " - PWMFrequency =  ");
  DEBUG_PRINTLN(DEBUG_INFO,                     currentPWMFrequency);
  DEBUG_PRINT(  DEBUG_INFO, " - MinIllum =      ");
  DEBUG_PRINTLN(DEBUG_INFO,                     min_illum);
  DEBUG_PRINT(  DEBUG_INFO, " - DaylightValue = ");
  DEBUG_PRINTLN(DEBUG_INFO,                     daylight_value);
  DEBUG_PRINT(  DEBUG_INFO, " - CurrentMode =   ");
  DEBUG_PRINTLN(DEBUG_INFO,                     currentMode);
}

bool loadSettings() {
  File file = LittleFS.open("/settings.txt", "r");
  if (!file) {
    DEBUG_PRINTLN(DEBUG_WARNING, "Failed to open file for reading");
    return false;
  }
  DEBUG_PRINTLN(DEBUG_INFO, "Reading settings from LittleFS");
  while (file.available()) {
    String line = file.readStringUntil('\n');
    int idx = line.indexOf('=');
    String key = line.substring(0, idx);
    String value = line.substring(idx + 1);

    if (key == "PWMFrequency") {
      currentPWMFrequency = value.toFloat();
      DEBUG_PRINT(  DEBUG_INFO, " - PWMFrequency =  ");
      DEBUG_PRINTLN(DEBUG_INFO,                     currentPWMFrequency);
    } else if (key == "MinIllum") {
      min_illum = value.toInt();
      DEBUG_PRINT(  DEBUG_INFO, " - MinIllum =      ");
      DEBUG_PRINTLN(DEBUG_INFO,                     min_illum);
    } else if (key == "DaylightValue") {
      daylight_value = value.toInt();
      DEBUG_PRINT(  DEBUG_INFO, " - DaylightValue = ");
      DEBUG_PRINTLN(DEBUG_INFO,                     daylight_value);
    } else if (key == "CurrentMode") {
      currentMode = value.toInt();
      DEBUG_PRINT(  DEBUG_INFO, " - CurrentMode =   ");
      DEBUG_PRINTLN(DEBUG_INFO,                     currentMode);
    }
  }
  file.close();
  DEBUG_PRINTLN(DEBUG_INFO, "Settings loaded successfully");
  return true;
}

// For writing LED_VALUE to the PWM pin, and optionally to the 8-bit onboard LED. 
void writePWM(uint16_t led_value) {
  pwm_set_gpio_level(LED_PIN, led_value);
  #ifdef USE_ONBOARD_LED
    analogWrite(ONBOARD_LED, led_value >> 8);
  #endif
}

// Initialize PWM
void pwm_init() {
  // Pin function
  gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
  
  // Because PWM settings are not per pin, but per 'slice'
  uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
  
  // Frequency & resolution settings
  float clkdiv = (float) clock_get_hz(clk_sys) / (currentPWMFrequency * 65535.0); // 16-bit resolution
  pwm_set_clkdiv(slice_num, clkdiv);
  pwm_set_wrap(slice_num, 65535); // 16-bit resolution
  pwm_set_enabled(slice_num, true);
  #ifdef USE_ONBOARD_LED
    pinMode(ONBOARD_LED, OUTPUT);
  #endif
  writePWM(0); // Initial duty cycle (0%)
}

// LED Gamma Correction Curve - 8-bit
uint8_t gammaCorrection8(uint8_t led_value, float gamma) {
  // Normalize the input
  float normalized_input = (float)led_value / 255.0;

  // Apply gamma correction
  float corrected = pow(normalized_input, gamma);

  // Scale back to 0-255
  return (uint8_t)(255 * corrected);
}

// LED Gamma Correction Curve - 16-bit
uint16_t gammaCorrection16(uint16_t led_value, float gamma) {
  // Normalize the input
  float normalized_input = (float)led_value / 65535.0;

  // Apply gamma correction
  float corrected = pow(normalized_input, gamma);

  // Scale back to 0-65535
  return (uint16_t)(65535 * corrected);
}

// Function to return the mode name based on current mode
const char* getModeName(uint8_t currentMode) {
  switch (currentMode) {
    case MODE_POTENTIOMETER:
      return MODE_POTENTIOMETER_NAME;
    case MODE_SKYLIGHT1:
      return MODE_SKYLIGHT1_NAME;
    case MODE_SKYLIGHT2:
      return MODE_SKYLIGHT2_NAME;
    case MODE_PHOTO_MATCH:
      return MODE_PHOTO_MATCH_NAME;
    case MODE_DEMO:
      return MODE_DEMO_NAME;
    case MODE_SERIAL:
      return MODE_SERIAL_NAME;
    default:
      return "Unknown Mode";  // Fallback for undefined modes
  }
}

// Function to debounce button presses
bool readDebouncedButton(int pin) {
    static int lastStableState = HIGH;   // Last stable state of the button (HIGH by default due to INPUT_PULLUP)
    static int lastReading = HIGH;       // Last reading of the button state
    static unsigned long lastDebounceTime = 0; // Last time the button state was toggled

    int currentReading = digitalRead(pin);

    if (currentReading != lastReading) { // If the reading has changed
        lastDebounceTime = millis();     // Reset the debouncing timer
    }

    if ((millis() - lastDebounceTime) > BUTTON_DEBOUND_DELAY) {
        // If the button state has been stable for the debounce period, consider it as a stable state
        if (currentReading != lastStableState) {
            lastStableState = currentReading; // Update stable state
        }
    }

    lastReading = currentReading;        // Save the current reading for the next loop
    return lastStableState;              // Return the stable state
}


// Read the momentary mode switch button
void readButton() {
  static unsigned long buttonPressedTime = 0; // Track when the button was initially pressed

  if (readDebouncedButton(MODE_SW) == LOW) { // Button is pressed
    
    if (buttonPressedTime == 0) {        // First detection of the button being pressed
      buttonPressedTime = currentMillis; // Record the time the button was pressed
        } else if ((currentMillis - buttonPressedTime > 4000)) { // Check for long press
          enterDateTimeSettingMode();    // Function to handle long press, enter time setting mode
          buttonPressedTime = 0;         // Reset the timer after handling long press
          return;                        // Exit the function to avoid further processing in the same press
    }
  } else if (buttonPressedTime != 0) {   // Button is not presed, but WAS pressed
    if ((currentMillis - buttonPressedTime > 25) && (currentMillis - buttonPressedTime <= 4000)) {
      // Handle normal button press if it was not a long press
      if (isDimmed || isOff) {
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command(255);    // Restore full brightness
        isDimmed = false;
        isOff = false;
        updateDisplay(-1, led_value, true);
      } else {
        cycleModes();                    // Only cycle modes if not just waking up the display
      }
      lastInteractionMillis = currentMillis; // Reset the dimming timer
    }
    buttonPressedTime = 0;               // Reset variables for next button press
  }
}

void cycleModes() {
  previousMillis = 0;                    // Reset timing to ensure immediate mode update        

  // Cycle through modes
  currentMode = (currentMode + 1) % 5;   // Cycles only between 0 and 4 (excludes mode 5 - serial input)
  saveSettings();                        // Save new mode to persistent storage whenever it changes
  
  // Log mode entry
  DEBUG_PRINT(DEBUG_WARNING, "Switched to mode: ");
  DEBUG_PRINTLN(DEBUG_WARNING, currentMode);
  
  // Perform initial actions for the new mode
  executeMode(true);
} // End cycleModes definition

// Perform initial actions for the new mode
void executeMode(bool forceUpdate) {
  switch (currentMode) {
    case MODE_POTENTIOMETER:
      handlePotentiometerMode(forceUpdate);      // Force update
      break;
    case MODE_SKYLIGHT1:
      handleSkylightMode1(forceUpdate);          // Force update
      break;
    case MODE_SKYLIGHT2:
      handleSkylightMode2(forceUpdate);          // Force update
      break;
    case MODE_PHOTO_MATCH:
      handlePhotoresistorMatchMode(forceUpdate); // Force update
      break;
    case MODE_DEMO:
      handleDemoMode(forceUpdate);               // Force update
      break;
  }
}

// When entering time setting mode, wait for user to set the potentiometer to approximate center before proceeding. 
void enterDateTimeSettingMode() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Set Time Mode");
  display.println("Release button please.");
  display.display();

  // Wait for button release
  while (readDebouncedButton(MODE_SW) == LOW) delay(50);

  // Get currently set date and time
  DateTime now = rtc.now();  // Assuming RTC is set up

  // Adjust time
  adjustTime(now);

  // Wait for button release
  while (readDebouncedButton(MODE_SW) == LOW) delay(50);

  // Adjust date
  adjustDate(now);

  // Finalize
  rtc.adjust(now);                       // Save the new time to RTC
  previousMillis = 0;                    // Reset previousMillis so whatever mode we are returning to updates the display.
  while (readDebouncedButton(MODE_SW) == LOW) delay(50); // Ensure button released before returning
}

// Wait for user to adjust potentiometer to middle
void waitForMiddle() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Set Time Mode");
  display.println("Adjust to middle to begin.");
  display.display();

  int potValue = 0;
  do {
    potValue = analogRead(POT_PIN);
    delay(100);
  } while (abs(potValue - 512) > 25);    // Assume middle is around 512 in a 0-1023 range
}

// Adjust date in a few different speeds based on movement up or down fron center
void adjustDate(DateTime &now) {
  waitForMiddle();
  displayDateSetting(now);
  while (readDebouncedButton(MODE_SW) == LOW) delay(50); // Double check that button is released

  while (readDebouncedButton(MODE_SW) == HIGH) {
    int potValue = analogRead(POT_PIN);
    int delta = potValue - 512;

    if (abs(delta) > 400) {              // Adjust year
      int yearAdjust = delta / 400;
      addDate(now, 0, 0, yearAdjust);
    } else if (abs(delta) > 250) {       // Adjust month
      int monthAdjust = delta / 250;
      addDate(now, 0, monthAdjust, 0);
    } else if (abs(delta) > 100) {       // Adjust day
      int dayAdjust = delta / 100;
      addDate(now, dayAdjust, 0, 0);
    }
    displayDateSetting(now);
    delay(100);
  } // Button has been pressed again
}

// Function to add days, months, and years to a DateTime object
void addDate(DateTime &date, int daysToAdd, int monthsToAdd, int yearsToAdd) {
  // Days in each month
  const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  // Extract current day, month, and year
  int day = date.day();
  int month = date.month();
  int year = date.year();

  // Check for leap year
  auto isLeap = [year]() {
    return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  };

  // Adjust years
  year += yearsToAdd;

  // Adjust months and handle year overflow/underflow
  month += monthsToAdd;
  while (month > 12) {
    month -= 12;
    year++;
  }
  while (month < 1) {
    month += 12;
    year--;
  }

  // Adjust days and handle month overflow/underflow
  day += daysToAdd;
  while (day > daysInMonth[month - 1] + (month == 2 && isLeap())) {
    day -= daysInMonth[month - 1] + (month == 2 && isLeap());
    month++;
    if (month > 12) {
      month -= 12;
      year++;
    }
  }
  while (day < 1) {
    month--;
    if (month < 1) {
      month += 12;
      year--;
    }
    day += daysInMonth[month - 1] + (month == 2 && isLeap());
  }

  // Create new DateTime object with the adjusted values, preserving the time
  date = DateTime(year, month, day, date.hour(), date.minute(), date.second());
}

// Adjust time in a few different speeds based on movement up or down from center
void adjustTime(DateTime &now) {
  waitForMiddle();
  displayTimeSetting(now);
  while (readDebouncedButton(MODE_SW) == LOW) delay(50); // Double check that button is released

  while (readDebouncedButton(MODE_SW) == HIGH) { // Exit loop when button is pressed again
    int potValue = analogRead(POT_PIN);
    int delta = potValue - 512;

    if (abs(delta) > 30 && abs(delta) < 60) {            // To avoid too sensitive adjustments
      now = now + TimeSpan(0, 0, 0, delta/5);               // 6-12 seconds per iteration
    } else if (abs(delta) >= 60 && abs(delta) < 240) {   // lets move a little quicker
      now = now + TimeSpan(0, 0, delta / 60, 0);            // 1-4 minutes per iteration
    } else if (abs(delta) >= 240 && abs(delta) < 480) {  // even quicker
      now = now + TimeSpan(0, 0, delta / 24, 0);            // 10-20 minutes per iteration
    } else if (abs(delta) >= 480) {                      // Fly - this is at the extremes of our analog read values - some POTs may not even be able to produce this value.
      now = now + TimeSpan(0, delta / 480, 0, 0);           // an hour per iteration
    }
    displayTimeSetting(now);
    delay(100);
  } // Button has been pressed again
}

// Updates to the OLED display
void updateDisplay(float elevation, uint16_t led_value, bool forceUpdate) {
  static unsigned long lastOledUpdateMillis = 0;  // Used to limit OLED updates

  // If the display is supposed to be off, or it has already been updated recently, skip updating the display
  if (isOff || (currentMillis - lastOledUpdateMillis) < OLED_UPDATE_INTERVAL) {
    if (forceUpdate == false) { // Unless force update was selected
      return;
    }
  }
  lastOledUpdateMillis = currentMillis; // Update the last update time

  // Start with the current mode on the first row
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(getModeName(currentMode));
  display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines

  // Calculate brightness percentage from led_value
  int percent = map(led_value, 0, 65535, 0, 100);
  display.print("Brightness: ");
  display.print(percent);
  display.println("%");
  display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines

  // Display elevation in Skylight modes
  if (currentMode == MODE_SKYLIGHT1 || currentMode == MODE_SKYLIGHT2) {
    display.print("Ele: ");
    display.print(elevation, 1); // Print elevation with one decimal place
    display.write(248); // Degree symbol
    display.println();
    display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines
  }
  
  // Add raw led_value in Demo mode
  if (currentMode == MODE_DEMO) {
    display.print("Raw 16-bit: ");
    display.println(led_value);
  }

  if (currentMode == MODE_SERIAL) {
    display.println("Awaiting Commands...");
  }

  // Display current time bottom-right aligned
  DateTime now = rtc.now();
  char timeString[9]; // Buffer for time string
  int hour = now.hour();
  char am_pm[3] = "AM";
  if (hour == 0) {
    hour = 12; // Midnight case
  } else if (hour == 12) {
    am_pm[0] = 'P'; // Noon case
  } else if (hour > 12) {
      hour -= 12;
    am_pm[0] = 'P';
  }

  // Construct the time string manually
  int idx = 0;
  if (hour < 10) timeString[idx++] = '0'; // Add leading zero if necessary
  itoa(hour, &timeString[idx], 10);
  idx += strlen(&timeString[idx]);

  timeString[idx++] = ':';

  if (now.minute() < 10) timeString[idx++] = '0'; // Add leading zero if necessary
  itoa(now.minute(), &timeString[idx], 10);
  idx += strlen(&timeString[idx]);

  timeString[idx++] = ' ';
  timeString[idx++] = am_pm[0];
  timeString[idx++] = am_pm[1];
  timeString[idx] = '\0'; // Null-terminate the string

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(timeString, 0, 0, &x1, &y1, &w, &h);  // Calculate the width and height of the time string
  display.setCursor(SCREEN_WIDTH - w - 1, SCREEN_HEIGHT - h); // Position the cursor for right alignment at the bottom
  display.print(timeString);
  
  // Done
  display.display();
}

// Function to display the current date in a specific format on the OLED
void displayDateSetting(const DateTime &now) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Set Date Mode");

  display.print("Day: ");
  if (now.day() < 10) display.print('0'); // Add leading zero if necessary
  display.println(now.day());

  display.print("Month: ");
  if (now.month() < 10) display.print('0'); // Add leading zero if necessary
  display.println(now.month());

  display.print("Year: ");
  display.println(now.year());

  display.display();
}

// Function to display the current time in a specific format on the OLED
void displayTimeSetting(const DateTime &now) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Set Time Mode");

  int hour = now.hour();
  char am_pm[3] = "AM";
  if (hour == 0) {
    hour = 12; // Midnight case
  } else if (hour == 12) {
    am_pm[0] = 'P'; // Noon case
  } else if (hour > 12) {
    hour -= 12;
    am_pm[0] = 'P';
  }

  if (hour < 10) display.print('0'); // Add leading zero if necessary
  display.print(hour);
  display.print(':');
  if (now.minute() < 10) display.print('0'); // Add leading zero if necessary
  display.print(now.minute());
  display.print(' ');
  display.println(am_pm);

  display.display();
}

