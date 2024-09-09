# Raspberry PI Pico Artificial LED Skylight Controller

## Project Overview
This project is designed to simulate natural daylight using an LED panel. The system adjusts the brightness of the panel based on the calculated solar elevation relative to the user's geographic location. With multiple operational modes, users can manually control the LED panel or allow it to mimic natural daylight throughout the day.

The Original Arduino Artificial LED Skylight Controller ourgrew the Arduino Uno's storage and processing capacity, and the RP2040 can be found less expensive anyway! 

## Features
- **Dynamic Brightness Adjustment**: Automatically adjusts the LED brightness based on the solar elevation.
- **Multiple Control Modes**:
  - **Manual Dimming**: Control brightness using a potentiometer.
  - **Skylight Mode 1**: Simple brightness curve for daylight simulation.
  - **Skylight Mode 2**: Advanced algorithm for realistic solar illumination.
  - **Photo Match Mode**: Adjusts LED brightness to match external light conditions (experimental).
  - **Demo Mode**: Sequentially tests all brightness levels for LED calibration.
- **Real-Time Feedback**: OLED display shows current mode, brightness levels, and time.
- **Memory Management**: (Currently broken) Saves last used settings in EEPROM for persistence across power cycles.
- **Debugging Support**: Multiple levels of debug output to assist in development and troubleshooting.

## Hardware Requirements
- RP2040
- DS3231 Real-Time Clock (RTC)
- SSD1306 OLED Display
- Potentiometer
- LED Panel capable of 8-bit or 16-bit PWM
- Suitable power supply (could potentially be pulled from the TV Panel)
- Momentary pushbutton (if you want multiple modes without access to the Pico's built-in USR button)

## Setup Instructions
### 1. Installing Arduino IDE and Arduino-Pico Framework

To use this project, you'll need to set up your development environment with the Arduino IDE and the Arduino-Pico core for RP2040. Follow these steps:

1. Install [Arduino IDE](https://www.arduino.cc/en/software).
2. Open Arduino IDE and navigate to **File > Preferences**.
3. In the "Additional Boards Manager URLs" field, enter:  
   `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`.
4. Go to **Tools > Board > Boards Manager**, search for "Pico", and install Raspberry Pi Pico/RP2040 by Earle F. Philhower, III.
5. Select your board, "Select Other Board", and pick Raspberry Pi Pico (The Raspberry Pi Pico/RP2040 version, **not** the Arduino-Mbed version).

### 2. Selecting the Correct Settings

Now that you are using the correct **Arduino-Pico** framework with this project, ensure you adjust the following settings in **Tools > Board** options:

- **Board**: Double check that the previous step selected **Raspberry Pi Pico**.
- **Flash Size**: Change this setting to match the flash on your RP2040 board including a small flash. Ensure **DO NOT** use `2MB (no flash)` as this disables flash storage and breaks setting persistence over power cycles.

### 3. Uploading the Code

Once you have selected the correct settings:
1. Open the project in the Arduino IDE.
2. Select the appropriate port under **Tools > Port**.
3. Press **Upload** to flash the code onto your RP2040.

## Required Configuration
### 1. Setting Geographical Coordinates
To simulate natural daylight, you need to set the latitude and longitude of your location in the code. These values are crucial for accurate solar elevation calculations.

- **Latitude and Longitude**: Update the following constants in the code with your location's latitude and longitude:
  ```cpp
  const float latitude = [Your Latitude];   // e.g., 34.21316116274671
  const float longitude = [Your Longitude]; // e.g., -84.54894616203148
  ```
- **Finding Your Coordinates**: You can find your latitude and longitude by searching for your address on Google Maps. Right-click on your location and select "What's here?" to view the coordinates.

### 2. Configuring Brightness Levels
The system allows you to configure the minimum illumination values for both 8-bit and 16-bit PWM, which are important for ensuring your LED panel operates correctly at lower brightness levels.

- **Initial Setup**: Start with the following values:
  ```cpp
  #define MIN_ILLUM16 1           // Minimum brightness for 16-bit PWM
  #define DAYLIGHT_VALUE16 65535  // Maximum daylight brightness for 16-bit PWM
  ```

- **Calibrating Brightness**:
  - **Option 1a - Use Demo Mode**: Use demo mode to cycle through all brightness levels. This will help identify the optimal minimum brightness (`MIN_ILLUM16`) for your LED panel. However, demo mode may run too quickly at higher brightness levels, so it might not be ideal for determining maximum brightness. 
  - **Option 1b - Manual Dimming Mode**: Switch to manual dimming mode and adjust the potentiometer to find a comfortable daylight brightness. Note the percentage shown on the OLED display, which can help you approximate and set the `DAYLIGHT_VALUE16` variables. This does no limit max manual dimming brightness, but does limit how bright your panel is at high noon. Mine seems brigher than the sun :-)
  - **Option 2 - Use Serial Commands**: While connected to Arduino IDE, with "newline" set as the line ending and baud rate set to 115200, you can send the serial commands (documented below) to configure these settings. 

### 3. Enabling/Disabling Modes
Ensure that only the modes you intend to use are enabled. This can be controlled by commenting/uncommenting the mode definitions in the code:

```cpp
// Enabled Modes -  Only uncomment the modes you want available (and that you have the hardware for!).
#define EN_MODE_POTENTIOMETER            // At this time, a potentiometer is still required for time setting (and I have yet to make compiliation without that functionality an option)
#define EN_MODE_SKYLIGHT1                // A piecewise brightness curve that just seemed to match what I wanted. Results in softer daylight brightness.
#define EN_MODE_SKYLIGHT2                // An advanced, mathematical, and closest to reality brightness. 
//#define EN_MODE_PHOTO_MATCH            // This mode leverages photosensors - one viewing the outside sky and another viewing the area under the LED panel and strives to make them match. This is the least tested and developed mode today. 
#define EN_MODE_DEMO                     // A simple slow (but scaled) progression to test all valid PWM values - useful to finding the best min_illum and daylight_value.
#define EN_MODE_SERIAL                   // Mode for Serial Control               
```

**Reminder:** Properly configuring the enabled modes and ensuring the correct hardware is connected will prevent unexpected behavior and errors during operation.

## Getting Started
1. **Assemble Hardware**: Maybe one day I'll write some good guides on hardware wiring... but not today. At least pin assignment notes are below.
2. **Install Required Libraries**: Use the Arduino Library Manager to install all the necessary libraries (`RTClib`, `Adafruit_SSD1306`, `Wire`, `EEPROM`, `SolarCalculator`).
3. **Upload the Sketch**: Modify the configuration variables as outlined above, then upload the sketch to your Arduino Uno.
4. **Test and Calibrate**: Use the demo and manual modes to calibrate brightness levels and test the system.

### Default configuration pin assignments:
| **Component**            | **Name**      | **RP2040 Pin** | **Description**                           |
|--------------------------|---------------|----------------|-------------------------------------------|
| **Power Supply In**      | **VCC**       | 39 (Vin/VSYS)  | 1.8 to 5.5V Supply                        |
|                          | **GND**       | 38 (GND)       | Ground                                    |
| **OLED Display**         | **SDA**       | 4 (GP2)        | I2C Data Line                             |
|                          | **SCL**       | 5 (GP3)        | I2C Clock Line                            |
|                          | **VCC**       | 36 (3V3)       | Pico Regulated 3.3V Out                   |
|                          | **GND**       | 3 (GND)        | Ground                                    |
| **RTC (DS3231)**         | **SDA**       | 4 (GP2)        | Shared (parallel) with OLED               |
|                          | **SCL**       | 5 (GP3)        | Shared (parallel) with OLED               |
|                          | **VCC**       | 36 (3V3)       | Shared (parallel) with OLED               |
|                          | **GND**       | 3 (GND)        | Shared (parallel) with OLED               |
| **Potentiometer**        | **Analog In** | 31 (GP26/ADC0) | Potentiometer's wiper pin                 |
|                          | **VCC**       | 36 (3V3)       | Pico Regulated 3.3V Out (2 in 1 hole)     |
|                          | **GND**       | 33 (GND)       | Ground                                    |
| **Mode Button**          | **Input**     | 21 (GP16)      | Digital input for mode switching          |
|                          | **GND**       | 23 (GND)       | Ground                                    |
| **Internal Photosensor** | **Analog In** | 32 (GP27/ADC1) | Analog input for internal light sensing   |
|                          | **GND**       | 28 (GND)       | Ground                                    |
| **External Photosensor** | **Analog In** | 34 (GP28/ADC2) | Analog input for external light sensing   |
|                          | **GND**       | 28 (GND)       | Ground                                    |
| **LED Panel (PWM control)**| **PWM**     | 19 (GP14)      | PWM output to LED panel control           |
|                          | **GND**       | 18 (GND)       | Ground                                    |


## Serial Commands

This project supports various serial commands for adjusting settings on the fly. Commands must be sent with a newline character at the end (`\n`). Below are the available commands:

### 1. Adjust PWM Frequency

- **Command**: `F<value>`
- **Description**: Sets the PWM frequency.
- **Valid Values**: Any positive float value. The default value for the RP2040 is 488.28 Hz.
- **Example**:  
  `F500.00` sets the PWM frequency to 500 Hz.

### 2. Adjust PWM Duty Cycle

- **Command**: `D<value>`
- **Description**: Sets the PWM duty cycle for the LED panel.
- **Valid Values**: 0 to 65535 (16-bit PWM).
- **Example**:  
  `D32768` sets the PWM duty cycle to 50% (32768 out of 65535).

### 3. Adjust Minimum Illumination (`minIllum`)

- **Command**: `I<value>`
- **Description**: Sets the minimum illumination level for the LED panel.
- **Valid Values**: 0 to 65535.
- **Example**:  
  `I100` sets the minimum illumination to 100.

### 4. Adjust Daylight Value (`daylight_value`)

- **Command**: `L<value>`
- **Description**: Sets the maximum daylight brightness for the LED panel.
- **Valid Values**: 0 to 65535.
- **Example**:  
  `L60000` sets the daylight brightness value to 60000.

### 5. Adjust Current Mode (`currentMode`)

- **Command**: `M<value>`
- **Description**: Sets the current mode.
- **Valid Values**: 0 to 5.
  - `0`: Manual Dimming
  - `1`: Skylight Mode 1
  - `2`: Skylight Mode 2
  - `3`: Brightness Match (if enabled)
  - `4`: Demo Mode
  - `5`: Serial Control Mode
- **Example**:  
  `M2` sets the current mode to Skylight Mode 2.

### 6. Print All Settings

- **Command**: `P`
- **Description**: Prints all current settings to the serial output, including PWM frequency, duty cycle, minimum illumination, daylight value, and the current mode.
- **Example**:  
  `P` prints all settings to the serial monitor.


## TO DO
- **Modularize by Hardware**: Enable the system to compile without certain hardware components like the display, potentiometer, button, or RTC module.
- **Split the Code into Multiple Files**: Improve maintainability by separating the code into different files based on functionality.
- **Improve Time Setting Mode**: I'd like more granularity in speed of increment, possibly by using modulo of delta section as a delay divisor.
- **Brightness Setting Mode**: Add a way to set minIllum and daylightValue using the Potentiometer.

## License
This project is released under the Creative Commons Attribution-NonCommercial 4.0 International license. See the LICENSE file for more details.
