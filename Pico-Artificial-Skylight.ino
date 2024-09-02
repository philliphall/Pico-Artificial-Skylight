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
    - EEPROM memory management for storing settings
    - Debugging support with adjustable verbosity

  Modes:
    - Manual Dimming: Control brightness with a potentiometer
    - Skylight Mode 1: Simple brightness curve
    - Skylight Mode 2: Complex, realistic solar simulation
    - Photo Match Mode: Adjusts brightness to match external light (experimental)
    - Demo Mode: Cycles through all brightness levels to test LED functionality

  Libraries Used:
    - RTClib.h for RTC management
    - Adafruit_SSD1306.h for OLED display handling
    - Wire.h for I2C communication
    - EEPROM.h for non-volatile memory storage
    - SolarCalculator.h for calculating solar position

  TO DO:
    - Get tested and working on the RP2040 platform
    - Modularize by hardware. Be able to compile without a display, for example. Or without a potentiometer, or button, or RTC module. 
    - Split the code into multiple files for easier maintainability. 
*/

#include <EEPROM.h>
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
#define DAYLIGHT_VALUE16 55000           // Compared to full range of 0-65535 - my light is way too bright for indoors
#define MIN_ILLUM16 25                   // Because some displays really don't like being super low on the PWM signal.
#define PHOTO_OFFSET -50                 // The photoresistors aren't identical, and their connections and cord lengths aren't identical. The external reading will be adjusted by this much.
#define DIMMING_TIME 30000               // OLED dimming time in milliseconds (30 seconds)
#define DIMMED_BRIGHTNESS 5              // OLED dimmed brightness as a percentage (5%)
#define OFF_TIME 300000                  // OLED Off time in milliseconds (5 minutes)
#define SKY_CHECK_INTERVAL 10000         // Used in mode 1&2 - only update skylight every xx milliseconds
#define OLED_UPDATE_INTERVAL 500         // Milliseconds between OLED display updates
#define BUTTON_DEBOUND_DELAY 50          // Debounce time in milliseconds
#define USE_ONBOARD_LED                  // Whether or not to match the onboard LED to the PWM-controlled panel.

// Enabled Modes -  Only uncomment the modes you want available (and that you have the hardware for!)
#define MODE_POTENTIOMETER               // At this time, a potentiometer is still required for time setting (and I have yet to make compiliation without that functionality an option)
#define MODE_SKYLIGHT1                   // A piecewise brightness curve that just seemed to match what I wanted. Results in softer daylight brightness.
#define MODE_SKYLIGHT2                   // An advanced, mathematical, and closest to reality brightness. 
//#define MODE_PHOTO_MATCH               // This mode leverages photosensors - one viewing the outside sky and another viewing the area under the LED panel and strives to make them match. This is the least tested and developed mode today. 
#define MODE_DEMO                        // A simple slow (but scaled) progression to test all valid PWM values - useful to finding the best MIN_ILLUM and DAYLIGHT values.

// Pin Designations
#define LED_PIN16 14        // 16-bit PWM for LED control
#define ONBOARD_LED 25      // Optionally, we can duplicate this brightness to the onboard LED. Only used if USE_ONBOARD_LED is also defined. 
#define MODE_SW 16          // Mode switch button
#define PHOTO_INT_PIN 28    // Internal photoresistor
#define PHOTO_EXT_PIN 27    // External photoresistor
#define POT_PIN 26          // Potentiometer wiper input

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

#define DEBUG_LEVEL DEBUG_INFO           // Set the debug level here

#define DEBUG_PRINT(level, message) \
    do { if (DEBUG_LEVEL >= level) Serial.print(message); } while (0)
#define DEBUG_PRINTLN(level, message) \
    do { if (DEBUG_LEVEL >= level) Serial.println(message); } while (0)

// EEPROM addresses
#define ADDR_SIGNATURE 0                 // Address for signature
#define ADDR_MODE 1                      // Address for the mode
#define EEPROM_SIGNATURE 0xD4            // Signature byte

// Global Scope Declarations
unsigned long currentMillis = 0;         // stores the value of millis() in each iteration of loop()
unsigned long previousMillis = 0;        // for various timings - mode dependent
unsigned long lastInteractionMillis = 0; // Global variable to track the last interaction time for OLED dimming
uint8_t currentMode = 0;                 // 0-Manual, 1-Skylight1, 2-Skylight2, 3-PhotoMatch, 4-Demo
uint16_t led_value16 = MIN_ILLUM16;      // 16-bit LED brightness control
bool isDimmed = false;                   // Variable to track the current display brightness state
bool isOff = false;                      // Variable to track the current display state

// Global variables for MODE_POTENTIOMETER
#if defined(MODE_POTENTIOMETER)
int potval;                              // Used in mode 0 - value read from the potentiometer on pot_pin
int previousInteractionPotval = 0;       // Used to change to manual mode from other modes
int previousModePotval = 0;              // New variable to track potentiometer value for mode changes
#define AVERAGE_BUFFER_SIZE 100          // Number of readings to average
int potReadings[AVERAGE_BUFFER_SIZE];    // Array to store the potentiometer readings
int readIndex = 0;                       // Index for the next reading
long total = 0;                          // Running total of the readings
int average = 0;                         // Average of the readings
#endif

// Global variables used in Skylight Mode 1
#if defined(MODE_SKYLIGHT1)
float relative_brightness16;
#endif

// Global variables used in Skylight Mode 2
#if defined(MODE_SKYLIGHT2)
#define MATCH_ELEVATION 1                // Elevation angle at which we switch between twilight and daylight models. Don't recommend you change it.
#define ASTRONOMICAL_DECAY_CONSTANT 0.15 // For how slow the twilight transformation goes
#define DIFFUSE_SCALING_FACTOR 1         // Will be multiplied. 1 means no change. Higher emphasises the impact of ambient reflection of light
float match_factor;
float max_combined_level = 0;
#endif

// Global variables used in Photo Match Mode
#if defined(MODE_PHOTO_MATCH)
int photo_int;                           // Used in mode 3 - value read from the indoor photoresistor
int photo_ext;                           // Used in mode 3 - value read from the outdoor photoresistor
#endif

// Global variables used in Demo Mode
#if defined(MODE_DEMO)
boolean demoDirectionUp = true;          // Used in mode 4 - state variable
uint8_t demoCounter = 255;               // Used in mode 4 - increment the 8-bit PWM value once every 256 iterations of the 16-bit loop
#endif

// Function Prototypes - All mode handlers need to be defined regardless of inclusion so that button presses can still call them and result in cycling to the next mode.
void handlePotentiometerMode(bool forceUpdate = false);  
void handleSkylightMode1(bool forceUpdate = false);
#ifdef MODE_SKYLIGHT2 // These helper functions are not required though.
float direct_sunlight_brightness(float elevation, float I0 = 1361, float k = 0.2);
float diffuse_sky_brightness(float elevation, float a = -1, float b = -0.32, float c = 0.43, float d = 1.25, float e = -0.35);
float astronomical_twilight_brightness(float elevation);
float combined_daytime_brightness(float elevation);
float combined_brightness(float elevation);
#endif
void handleSkylightMode2(bool forceUpdate = false);
void handlePhotoresistorMatchMode(bool forceUpdate = false);
void handleDemoMode(bool forceUpdate = false);
#ifdef MODE_DEMO // These helper functions are not required though.
unsigned long getNextDemoInterval(uint16_t led_value16, bool goingUp);
#endif

// Function Prototypes - Utility functions
void initializeEEPROM();
void writePWM(uint16_t led_value16);
uint8_t gammaCorrection(uint8_t led_value, float gamma = 2.2); // No more 8-bit PWM any more, but still need this for 8-bit OLED display contrast correction
uint16_t gammaCorrection16(uint16_t led_value16, float gamma = 2.2);

// Function Prototypes - Button and mode cycling
bool readDebouncedButton(int pin);
void readButton();
void cycleModes();

// Function Prototypes - Time setting functions
void enterDateTimeSettingMode();
void waitForMiddle();
void adjustDate(DateTime &now);
void addDate(DateTime &date, int daysToAdd, int monthsToAdd, int yearsToAdd);
void adjustTime(DateTime &now);

// Function Prototypes - Display
void updateDisplay(const char* mode, float elevation, uint16_t led_value16, bool forceUpdate = false);
void displayDateSetting(const DateTime &now);
void displayTimeSetting(const DateTime &now);


// *** SETUP ***
void setup() {
  Serial.begin(115200);
  // Wait for the serial port to connect, with a timeout to avoid getting stuck
  if (DEBUG_LEVEL >= DEBUG_INFO) {
    unsigned long startMillis = millis();
    while (!Serial && (millis() - startMillis < 5000)) {
      delay(10);
    }
  }
  Serial.println("Serial started");
  DEBUG_PRINTLN(DEBUG_INFO, "Setup started");

  // Initialize PWM for LED pins
  gpio_set_function(LED_PIN16, GPIO_FUNC_PWM);
  pwm_set_wrap(pwm_gpio_to_slice_num(LED_PIN16), 65535); // 16-bit resolution
  pwm_set_enabled(pwm_gpio_to_slice_num(LED_PIN16), true);
  #ifdef USE_ONBOARD_LED
  pinMode(ONBOARD_LED, OUTPUT);
  #endif
  writePWM(0); // Initial duty cycle (0%)
  
  // Mode switch setup
  pinMode(MODE_SW, INPUT_PULLUP);
  
  // Restore last mode
  initializeEEPROM();
  currentMode = EEPROM.read(ADDR_MODE);
  DEBUG_PRINT(DEBUG_INFO, "Setting initial Mode from EEPROM: ");
  DEBUG_PRINTLN(DEBUG_INFO, currentMode);
  
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
  
  // Used in MODE_POTENTIOMETER
  #if defined(MODE_POTENTIOMETER)
  for (int thisReading = 0; thisReading < AVERAGE_BUFFER_SIZE; thisReading++) { // Init Debounce Array
    potReadings[thisReading] = 0; 
  }
  previousInteractionPotval = analogRead(POT_PIN); // So we don't immediately switch to manual on first run.
  previousModePotval = previousInteractionPotval;
  #endif

  // Used in Skylight Mode 1 - Linear LED Brightness
  #if defined(MODE_SKYLIGHT1)
  relative_brightness16 = (65535 * log10(2)) / log10(65535);
  #endif

  // Used in Skylight Mode 2 - Calculate combined and twilight light intensities at a matching point
  #if defined(MODE_SKYLIGHT2)
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
  #if defined(MODE_POTENTIOMETER)
  if (currentMode != 0 && abs(potval - previousModePotval) > 50) {
    currentMode = 0;                     // Dimmer moved - switch to manual dimming mode
    previousMillis = 0;                  // Always reset on mode change
    DEBUG_PRINTLN(DEBUG_WARNING, "Manual Override - Entering mode 0 - Manual Dimming");
    EEPROM.update(ADDR_MODE, currentMode);  // Save new mode to EEPROM whenever it changes
    previousModePotval = potval;            // Reset  
    lastInteractionMillis = currentMillis;  // Reset interaction timer
  }
  #endif 

  // Check smaller potentiometer changes to reset interaction timer
  if (abs(potval - previousInteractionPotval) > 25) { // Threshold for considering it an interaction
    if (isDimmed || isOff) { // Re-activate the display if it was dimmed or turned off
      display.ssd1306_command(SSD1306_SETCONTRAST);
      display.ssd1306_command(255);
      display.display();
      isDimmed = false;
      isOff = false;
    }
    previousInteractionPotval = potval;     // Reset
    lastInteractionMillis = currentMillis;  // Reset last interaction timer
  }
  
  // Dimming logic
  if ((currentMillis - lastInteractionMillis >= DIMMING_TIME) && !isDimmed) {
    DEBUG_PRINTLN(DEBUG_INFO, "Dimming OLED now.");
    uint8_t correctedDimmedBrightness = gammaCorrection(map(DIMMED_BRIGHTNESS, 0, 100, 0, 255));
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
  
  // Execute the current mode’s function
  switch (currentMode) {
    case 0:
      handlePotentiometerMode();
      break;
    case 1:
      handleSkylightMode1();
      break;
    case 2:
      handleSkylightMode2();
      break;
    case 3:
      handlePhotoresistorMatchMode();
      break;
    case 4:
      handleDemoMode();
      break;
  }
}

// Basic manual dimmer mode with gamma correction
void handlePotentiometerMode(bool forceUpdate) {
  #if defined(MODE_POTENTIOMETER)
  // Debounce
  total = total - potReadings[readIndex];            // Subtract the oldest reading from the total
  potReadings[readIndex] = potval;                   // Add the current potentiometer value and store it into the array
  total = total + potReadings[readIndex];            // Add the new reading to the total
  readIndex = (readIndex + 1) % AVERAGE_BUFFER_SIZE; // Advance to the next position in the array
  average = total / AVERAGE_BUFFER_SIZE;             // Calculate the average
  
  // Map the average potentiometer value to the 16-bit range
  uint16_t mapped_value16 = map(average, 0, 1023, 0, 65535);
  
  // Apply gamma correction
  led_value16 = gammaCorrection16(mapped_value16);

  // Write the corrected values to the LED pins
  writePWM(led_value16);
  updateDisplay("Manual Dimming", -1, led_value16, forceUpdate);

  // Update debug, based on timing
  if (currentMillis - previousMillis > 500 || forceUpdate == true) { // Only debug every half second
    previousMillis = currentMillis;
    
    // Consolidated debug output
    DEBUG_PRINT(DEBUG_VERBOSE, "Potval: ");
    DEBUG_PRINT(DEBUG_VERBOSE, potval);
    DEBUG_PRINT(DEBUG_VERBOSE, " | Avg: ");
    DEBUG_PRINT(DEBUG_VERBOSE, average);
    DEBUG_PRINT(DEBUG_VERBOSE, " | Mapped 16-bit: ");
    DEBUG_PRINT(DEBUG_VERBOSE, mapped_value16);
    DEBUG_PRINT(DEBUG_VERBOSE, " | LED 16-bit: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, led_value16);  
  }
  #else
  cycleModes();
  #endif
}

// *** Artificial Skylight Mode 1 - Modeled
void handleSkylightMode1(bool forceUpdate) {
  #if defined(MODE_SKYLIGHT1)
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
      led_value16 = DAYLIGHT_VALUE16;
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 15) {
      led_value16 = map(elevation * 100, 15 * 100, 40 * 100, .85 * DAYLIGHT_VALUE16, DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 3) {
      led_value16 = map(elevation * 100, 3 * 100, 15 * 100, .76 * DAYLIGHT_VALUE16, .85 * DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 0) {
      led_value16 = map(elevation * 100, 0 * 100, 3 * 100, .7 * DAYLIGHT_VALUE16, .76 * DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= -3) {
      led_value16 = map(elevation * 100, -3 * 100, 0 * 100, .6 * DAYLIGHT_VALUE16, .7 * DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
      led_value16 += map(elevation * 100, -3 * 100, 0 * 100, MIN_ILLUM16, 0);
    } else if (elevation >= -12) {
      led_value16 = map(elevation * 100, -12 * 100, -3 * 100, 0, .6 * DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
      led_value16 += MIN_ILLUM16;
    } else {
      led_value16 = 0;
    }

    // Guard rails
    if (led_value16 > 0 && led_value16 < MIN_ILLUM16 / 2) led_value16 = 0; // LED stays off until halfway to MIN_ILLUM16
    else if (led_value16 >= MIN_ILLUM16 / 2 && led_value16 < MIN_ILLUM16) led_value16 = MIN_ILLUM16 / 2; // LED turns on halfway
    if (led_value16 > DAYLIGHT_VALUE16) led_value16 = DAYLIGHT_VALUE16;

    // Write it
    writePWM(led_value16);
    updateDisplay("Skylight Mode 1", elevation, led_value16, forceUpdate);
    DEBUG_PRINT(DEBUG_VERBOSE, "LED Value16: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, led_value16);
  }
  #else
  cycleModes();
  #endif
}

// Artificial Skylight Mode 2
#if defined(MODE_SKYLIGHT2)
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
  #if defined(MODE_SKYLIGHT2)
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
    led_value16 = map(combined_level, 0, max_combined_level, 0, 65535); // map algorithm value to 16-bit value

    // Gamma Correction
    led_value16 = gammaCorrection16(led_value16);

    // Guard rails
    if (led_value16 > 0 && led_value16 < MIN_ILLUM16 / 2) led_value16 = 0; // LED stays off until halfway to MIN_ILLUM16
    else if (led_value16 >= MIN_ILLUM16 / 2 && led_value16 < MIN_ILLUM16) led_value16 = MIN_ILLUM16 / 2; // LED turns on halfway
    if (led_value16 > DAYLIGHT_VALUE16) led_value16 = DAYLIGHT_VALUE16;
    
    // Write it
    writePWM(led_value16);
    updateDisplay("Skylight Mode 2", elevation, led_value16, forceUpdate);
    DEBUG_PRINT(DEBUG_VERBOSE, "LED Value16: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, led_value16);
  }
  #else
  cycleModes();
  #endif
}

// Photo Matching Mode
void handlePhotoresistorMatchMode(bool forceUpdate) {
  #if defined(MODE_PHOTO_MATCH)
  int photo_int = analogRead(PHOTO_INT_PIN);
  int photo_ext = analogRead(PHOTO_EXT_PIN) + PHOTO_OFFSET;
  if (photo_int > photo_ext) {
    //float dif;
    //dif = ((float)photo_int - (float)photo_ext) / 1023;
    //led_value16 -= ((int)((float)led_value16 * dif * .05))+1;
    led_value16 = max(led_value16 - 1, 0);
    //if ((led_value16 > 0) && (led_value16 < MIN_ILLUM16)) {led_value16 = MIN_ILLUM16;}
  } else if (photo_int < photo_ext) {
    //float dif;
    //dif = ((float)photo_ext - (float)photo_int) / 1023;
    //led_value16 += ((int)((float)led_value16 * dif *.05)+1);
    led_value16 = min(led_value16 + 1, 65535);
    if (led_value16 < MIN_ILLUM16) led_value16 = MIN_ILLUM16;
  }
  /*    else if (photo_int == photo_ext) {
    DEBUG_PRINT(DEBUG_VERBOSE, "Matched values! led_value16: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value16);
    DEBUG_PRINT(DEBUG_VERBOSE, " - photo_ext & photo_int: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, photo_int);
  }
  else {
    DEBUG_PRINT(DEBUG_ERROR, "Reached the else clause in photoresistor match mode, which should be impossible! led_value16: ");
    DEBUG_PRINT(DEBUG_ERROR, led_value16);
  }
  */    
  writePWM(led_value16);
  updateDisplay("Brightness Match", -1, led_value16, forceUpdate);

  // Write status
  if (currentMillis - previousMillis > 250 || forceUpdate == true) { // We don't want to print to serial/display at max speed, 4x per second should be fine.
    previousMillis = currentMillis;
    DEBUG_PRINT(DEBUG_VERBOSE, "---Status--- led_value16: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value16);
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
  #if defined(MODE_DEMO)
  if (forceUpdate == true) {
    demoDirectionUp = true;
    led_value16 = 0;
    demoCounter = 255;
  }
  
  // Manage the LED intensity and update timing
  if (demoDirectionUp) {
    if (led_value16 < 65535 && currentMillis - previousMillis >= getNextDemoInterval(led_value16, true)) {
      previousMillis = currentMillis;
      led_value16++;
    } else if (led_value16 >= 65535) {
      demoDirectionUp = false;
      led_value16--;
    }
  } else {
    if (led_value16 > 0 && currentMillis - previousMillis >= getNextDemoInterval(led_value16, false)) {
      previousMillis = currentMillis;
      led_value16--;
    } else if (led_value16 <= 0) {
      demoDirectionUp = true;
      led_value16++;
    }
  }

  // Update PWM outputs to LED
  writePWM(led_value16);
  updateDisplay("Demo Mode", -1, led_value16, forceUpdate);
  #else
  cycleModes();
  #endif
} // End Demo Mode

// Helper function to determine the next interval based on the current brightness and direction
unsigned long getNextDemoInterval(uint16_t led_value16, bool goingUp) {
    #if defined(MODE_DEMO)
    if (goingUp) {
    if (led_value16 < 60) return 100;
    else if (led_value16 < 255) return 40;
    else if (led_value16 < 511) return 20;
    else return 10;
  } else {
    if (led_value16 > 511) return 10;
    else if (led_value16 > 255) return 20;
    else if (led_value16 > 40) return 40;
    else return 100;
  }
  #endif
  return -1; // Should not reach this
}

// Initialize EEPROM so we can retain last mode on resume
void initializeEEPROM() {
  uint8_t sig = EEPROM.read(ADDR_SIGNATURE);
  if (sig != EEPROM_SIGNATURE) {
    // Signature not matched, EEPROM not initialized
    DEBUG_PRINTLN(DEBUG_WARNING, "EEPROM Signature not matched, initializing the EEPROM addresses used by this sketch.");
    EEPROM.write(ADDR_SIGNATURE, EEPROM_SIGNATURE);
    EEPROM.write(ADDR_MODE, 0);  // Default mode
  }
}

// For writing LED_VALUE16 to the PWM pin, and optionally to the 8-bit onboard LED. 
void writePWM(uint16_t led_value16) {
  pwm_set_gpio_level(LED_PIN16, led_value16);
  #ifdef USE_ONBOARD_LED
  analogWrite(ONBOARD_LED, led_value16 >> 8);
  #endif
}

// LED Gamma Correction Curve - 8-bit
uint8_t gammaCorrection(uint8_t led_value, float gamma) {
  // Normalize the input
  float normalized_input = (float)led_value / 255.0;

  // Apply gamma correction
  float corrected = pow(normalized_input, gamma);

  // Scale back to 0-255
  return (uint8_t)(255 * corrected);
}

// LED Gamma Correction Curve - 16-bit
uint16_t gammaCorrection16(uint16_t led_value16, float gamma) {
  // Normalize the input
  float normalized_input = (float)led_value16 / 65535.0;

  // Apply gamma correction
  float corrected = pow(normalized_input, gamma);

  // Scale back to 0-65535
  return (uint16_t)(65535 * corrected);
}

// Function to debounce button presses
bool readDebouncedButton(int pin) {
    static int lastStableState = HIGH; // Last stable state of the button (HIGH by default due to INPUT_PULLUP)
    static int lastReading = HIGH;     // Last reading of the button state
    static unsigned long lastDebounceTime = 0; // Last time the button state was toggled

    int currentReading = digitalRead(pin);

    if (currentReading != lastReading) { // If the reading has changed
        lastDebounceTime = millis(); // Reset the debouncing timer
    }

    if ((millis() - lastDebounceTime) > BUTTON_DEBOUND_DELAY) {
        // If the button state has been stable for the debounce period, consider it as a stable state
        if (currentReading != lastStableState) {
            lastStableState = currentReading; // Update stable state
        }
    }

    lastReading = currentReading; // Save the current reading for the next loop
    return lastStableState; // Return the stable state
}


// Read the momentary mode switch button
void readButton() {
  static unsigned long buttonPressedTime = 0; // Track when the button was initially pressed

  if (readDebouncedButton(MODE_SW) == LOW) { // Button is pressed
    
    if (buttonPressedTime == 0) { // First detection of the button being pressed
      buttonPressedTime = currentMillis; // Record the time the button was pressed
        } else if ((currentMillis - buttonPressedTime > 4000)) { // Check for long press
          enterDateTimeSettingMode(); // Function to handle long press, enter time setting mode
          buttonPressedTime = 0; // Reset the timer after handling long press
          return; // Exit the function to avoid further processing in the same press
    }
  } else if (buttonPressedTime != 0) { // Button is not presed, but WAS pressed
    if ((currentMillis - buttonPressedTime > 25) && (currentMillis - buttonPressedTime <= 4000)) {
      // Handle normal button press if it was not a long press
      if (isDimmed || isOff) {
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command(255); // Restore full brightness
        display.display();
        isDimmed = false;
        isOff = false;
      } else {
        cycleModes(); // Only cycle modes if not just waking up the display
      }
      lastInteractionMillis = currentMillis; // Reset the dimming timer
    }
    buttonPressedTime = 0; // Reset variables for next button press
  }
}

void cycleModes() {
  previousMillis = 0; // Reset timing to ensure immediate mode update        

  // Cycle through modes
  currentMode = (currentMode + 1) % 5;
  EEPROM.update(ADDR_MODE, currentMode);  // Save new mode to EEPROM whenever it changes
  
  // Log mode entry
  DEBUG_PRINT(DEBUG_WARNING, "Switched to mode: ");
  DEBUG_PRINTLN(DEBUG_WARNING, currentMode);
  
  // Perform initial actions for the new mode
  switch (currentMode) {
    case 0:
      handlePotentiometerMode(true); // Force update
      break;
    case 1:
      handleSkylightMode1(true); // Force update
      break;
    case 2:
      handleSkylightMode2(true); // Force update
      break;
    case 3:
      handlePhotoresistorMatchMode(true); // Force update
      break;
    case 4:
      handleDemoMode(true); // Force update
      break;
  }
} // End cycleModes definition

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
  rtc.adjust(now); // Save the new time to RTC
  previousMillis = 0; // Reset previousMillis so whatever mode we are returning to updates the display.
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
  } while (abs(potValue - 512) > 25); // Assume middle is around 512 in a 0-1023 range
}

// Adjust date in a few different speeds based on movement up or down fron center
void adjustDate(DateTime &now) {
  waitForMiddle();
  displayDateSetting(now);
  while (readDebouncedButton(MODE_SW) == LOW) delay(50); // Double check that button is released

  while (readDebouncedButton(MODE_SW) == HIGH) {
    int potValue = analogRead(POT_PIN);
    int delta = potValue - 512;

    if (abs(delta) > 400) {               // Adjust year
      int yearAdjust = delta / 400;
      addDate(now, 0, 0, yearAdjust);
    } else if (abs(delta) > 250) {        // Adjust month
      int monthAdjust = delta / 250;
      addDate(now, 0, monthAdjust, 0);
    } else if (abs(delta) > 100) {        // Adjust day
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
void updateDisplay(const char* mode, float elevation, uint16_t led_value16, bool forceUpdate) {
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
  display.println(mode);
  display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines

  // Calculate brightness percentage from led_value16
  int percent = map(led_value16, 0, 65535, 0, 100);
  display.print("Brightness: ");
  display.print(percent);
  display.println("%");
  display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines

  // Display elevation in Skylight modes
  if (currentMode == 1 || currentMode == 2) {
    display.print("Ele: ");
    display.print(elevation, 1); // Print elevation with one decimal place
    display.write(248); // Degree symbol
    display.println();
    display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines
  }
  
  // Add raw led_value16 in Demo mode
  if (currentMode == 4) {
    display.print("Raw 16-bit: ");
    display.println(led_value16);
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

