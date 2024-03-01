// REG NO: ENM221-0062/2020
// NAME: ONAMI OMARIBA COLLINS

// Include necessary libraries for LCD display, SPI and I2C communication
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Wire.h>

// Initialize the LCD display with I2C address, width and height
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// Define the Chip Select Pin for MCP3204
const int CS_PIN = 10; 

// Define actuator pins
#define MOTOR_PIN_1 3 // Pin for wheel 1
#define MOTOR_PIN_2 5 // Pin for wheel 2
#define MOTOR_PIN_3 6 // Pin for wheel 3
#define PARKING_PIN A0 // Pin for the parking brake
#define BRAKE_ACTUATOR_PIN A1 // Analog pin for the brake pedal potentiometer
#define PRESSURE_PIN_1 A2 // Pin for pressure sensor 1
#define PRESSURE_PIN_2 A3 // Pin for pressure sensor 2
#define PRESSURE_PIN_3 A6 // Pin for pressure sensor 3
#define ANTI_SKID_PIN A7 // Pin for the anti skid button
#define FAN_ON_INDICATOR_PIN 9 // Pin for the fan on indicator

// Define interface pins
#define DECELERATION_LOW_PIN 2 // Pin for low deceleration
#define DECELERATION_MEDIUM_PIN 4 // Pin for medium deceleration
#define DECELERATION_HIGH_PIN 7 // Pin for high deceleration
#define LANDING_GEAR_STATUS_PIN 8 // Pin for landing gear status

// Define global variables
bool landingGearStatus[3] = {false, false, false}; // Array to hold status of landing gears, initially set to up
bool yellowHydraulicStatus = false; // Status of yellow hydraulic system, initially set to false
bool antiSkidOn = false; // Status of anti skid, initially set to false
bool autoBrakeOn = false; // Status of auto brake, initially set to false
int brakePosition = 0; // Variable to hold brake position, initially set to 0
bool greenSystemStatus = true; // Status of green system, initially set to working

// Setup function runs once when the program starts
void setup() {
  // Initialize the lcd and turn on the backlight
  lcd.init();
  lcd.backlight();

  // Start serial communication at 9600 baud rate
  Serial.begin(9600);

  // Set pin modes for various pins
  pinMode(CS_PIN, OUTPUT);
  pinMode(BRAKE_ACTUATOR_PIN, INPUT);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(MOTOR_PIN_3, OUTPUT);
  pinMode(PARKING_PIN, OUTPUT);

  // Initialize SPI and set SPI clock frequency (1 MHz)
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Set pin modes for deceleration and landing gear status pins
  pinMode(DECELERATION_LOW_PIN, INPUT_PULLUP);
  pinMode(DECELERATION_MEDIUM_PIN, INPUT_PULLUP);
  pinMode(DECELERATION_HIGH_PIN, INPUT_PULLUP);
  pinMode(LANDING_GEAR_STATUS_PIN, OUTPUT);
  pinMode(FAN_ON_INDICATOR_PIN, OUTPUT);
  pinMode(ANTI_SKID_PIN, INPUT_PULLUP);

  // Initialize I2C communication
  Wire.begin(); 
}

// Loop function runs continuously after setup function
void loop() {
  // Update landing gear status
  for (int i = 0; i < 3; i++) {
    landingGearStatus[i] = digitalRead(LANDING_GEAR_STATUS_PIN + i);
  }

  // Check if all landing gears are down
  if (!landingGearStatus[0] || !landingGearStatus[1] || !landingGearStatus[2]) {
    return;
  }

  // Check if green system has failed
  if (!greenSystemStatus) {
    yellowHydraulicStatus = true;
    antiSkidOn = false; // Anti skid off when yellow system is active
  } else {
    yellowHydraulicStatus = false;
    antiSkidOn = true; // Anti skid on when green system is active
  }
  
  // Clear the display buffer
  lcd.clear(); 
  // Set cursor (Column, Row)
  lcd.setCursor(0, 0);
  // print "Hydraulic System: " at (0, 0)   
  lcd.print("Hydraulic System: "); 
  // Set cursor (Column, Row)
  lcd.setCursor(0,1);
  // print the status of the hydraulic system at (0, 1)
  lcd.print(yellowHydraulicStatus ? "YELLOW" : "GREEN");

  // Read brake position
  brakePosition = analogRead(BRAKE_ACTUATOR_PIN);
  Serial.println("Brake position");
  Serial.println(brakePosition);

  // Map brake position to motor speed (assuming brake position is in the range 0 to 1023)
  int motorSpeed = map(brakePosition, 0, 1023, 0, 255);
  Serial.println("Motor Speed");
  Serial.println(motorSpeed);

  // Control motor speeds based on brake position
  analogWrite(MOTOR_PIN_1, motorSpeed);
  analogWrite(MOTOR_PIN_2, motorSpeed);
  analogWrite(MOTOR_PIN_3, motorSpeed);

  // Update interface
  updateInterface();

  // Check status of deceleration buttons
  if (digitalRead(DECELERATION_LOW_PIN) || digitalRead(DECELERATION_MEDIUM_PIN) || digitalRead(DECELERATION_HIGH_PIN)) {
    autoBrakeOn = true;
  } else {
    autoBrakeOn = false;
  }
  // Control motor speeds based on deceleration buttons
  if (digitalRead(DECELERATION_LOW_PIN)) {
    // If the low deceleration button is pressed, set all motor speeds to 0
    analogWrite(MOTOR_PIN_1, 0);
    analogWrite(MOTOR_PIN_2, 0);
    analogWrite(MOTOR_PIN_3, 0);
  } else if (digitalRead(DECELERATION_MEDIUM_PIN)) {
    // If the medium deceleration button is pressed, set all motor speeds to 128
    analogWrite(MOTOR_PIN_1, 128);
    analogWrite(MOTOR_PIN_2, 128);
    analogWrite(MOTOR_PIN_3, 128);
  } else if (digitalRead(DECELERATION_HIGH_PIN)) {
    // If the high deceleration button is pressed, set all motor speeds to 255
    analogWrite(MOTOR_PIN_1, 255);
    analogWrite(MOTOR_PIN_2, 255);
    analogWrite(MOTOR_PIN_3, 255);
  }

  // Read MCP3204 channels for brake temperature and display
  for (int channel = 0; channel < 3; channel++) {
    // Read the sensor value from the MCP3204
    int sensorValue = readMCP3204(channel);
    Serial.print("sensors value: ");
    Serial.println(sensorValue);
    // Convert the sensor value to a temperature
    float temperature = map(sensorValue, 1665, 2355, 0, 150);

    // If the temperature exceeds 100, turn on the fan
    if (temperature > 100) {
      digitalWrite(FAN_ON_INDICATOR_PIN, HIGH);
    } else {
      digitalWrite(FAN_ON_INDICATOR_PIN, LOW);
    }

    // Display the brake temperature on the LCD and Serial Monitor
    lcd.setCursor(0, channel + 2);
    lcd.print("Temp ");
    lcd.print(channel + 1);
    lcd.print(": ");
    lcd.print(temperature);
    lcd.print("C");

    Serial.print("Brake ");
    Serial.print(channel + 1);
    Serial.print(" Temp: ");
    Serial.print(temperature);
    Serial.println(" °C");
    delay(500);
  }

  // If the yellow hydraulic system is active, display the pressure on the LCD and Serial Monitor
  if (yellowHydraulicStatus) {
    int pressure1 = analogRead(PRESSURE_PIN_1);
    int pressure2 = analogRead(PRESSURE_PIN_2);
    int pressure3 = analogRead(PRESSURE_PIN_3);

    lcd.setCursor(0, 5);
    lcd.print("Pressure 1: ");
    lcd.print(pressure1);
    lcd.print(" PSI");

    lcd.setCursor(0, 6);
    lcd.print("Pressure 2: ");
    lcd.print(pressure2);
    lcd.print(" PSI");

    lcd.setCursor(0, 7);
    lcd.print("Pressure 3: ");
    lcd.print(pressure3);
    lcd.print(" PSI");

    Serial.print("Pressure 1: ");
    Serial.print(pressure1);
    Serial.println(" PSI");

    Serial.print("Pressure 2: ");
    Serial.print(pressure2);
    Serial.println(" PSI");

    Serial.print("Pressure 3: ");
    Serial.print(pressure3);
    Serial.println(" PSI");
  }

  // Print the status of the Anti-Skid system
  Serial.print("Anti-Skid: ");
  Serial.println(antiSkidOn ? "ON" : "OFF");

  // Print the status of the Auto Brake system
  Serial.print("Auto Brake: ");
  Serial.println(autoBrakeOn ? "ON" : "OFF");

  // Delay for stability
  delay(500);
  }

  void updateInterface() {
    // Update landing gear status indicators
    for (int i = 0; i < 3; i++) {
      digitalWrite(LANDING_GEAR_STATUS_PIN + i, landingGearStatus[i] ? HIGH : LOW);
    }

    // Update yellow hydraulic system status indicator
    if (!greenSystemStatus) {
      yellowHydraulicStatus = true;
      antiSkidOn = false; // Anti skid off when yellow system is active
    } else {
      yellowHydraulicStatus = false;
      antiSkidOn = true; // Anti skid on when green system is active
    }
  }

  int readMCP3204(int channel) {
    // Send command to MCP3204 to read from specified channel
    byte command = B00000001 | (channel << 4); // Construct command byte
    digitalWrite(CS_PIN, LOW); // Select MCP3204
    delayMicroseconds(1); // Wait for MCP3204 to settle
    SPI.transfer(command); // Send command
    int result = SPI.transfer16(0); // Read ADC value (12 bits)
    digitalWrite(CS_PIN, HIGH); // Deselect MCP3204
    return result & 0xFFF; // Mask off unwanted bits
  }