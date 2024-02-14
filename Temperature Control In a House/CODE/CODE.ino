// REG NO: ENM221-0062/2020
// NAME: ONAMI OMARIBA COLLINS
#include <AutoPID.h>

// Pins
#define HEATER_PIN 9     // Pin to control the heater
#define FAN_PIN 10       // Pin to control the fan
#define TEMP_SENSOR_PIN A1 // Pin to read the temperature sensor

// PID settings and gains
#define KP 10 // Proportional gain
#define KI 0.1 // Integral gain
#define KD 1 // Derivative gain

// Minimum and maximum analog values observed from the temperature sensor
const int analogMin = 508;
const int analogMax = 774;

// Define the corresponding temperature range
const float tempMin = -25.0;
const float tempMax = 105.0;

// Temperature setpoint
#define SETPOINT 22.0  // Setpoint in Celsius

// Define temperature range for fan and heater control
#define TEMP_THRESHOLD 1.0  // Hysteresis threshold for fan and heater control

double temperature, setPoint, outputVal;

// Input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&temperature, &setPoint, &outputVal, tempMin, tempMax, KP, KI, KD);

// Define the size of the moving average filter
#define FILTER_SIZE 10

// Create an array to store the last FILTER_SIZE temperature readings
float temperatureReadings[FILTER_SIZE] = {0};

// Create a variable to keep track of the sum of the temperature readings
float temperatureSum = 0;

// Create a variable to keep track of the current index in the temperatureReadings array
int temperatureIndex = 0;

void setup() {
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize PID controller
  myPID.setBangBang(TEMP_THRESHOLD);

  // Set PID update interval to 1000ms (1 second)
  myPID.setTimeStep(1000);
}

// Boolean flags to track the state of the heater and fan
bool isHeaterOn = false;
bool isFanOn = false;

void loop() {
  // Read temperature sensor
  float rawTemperature = map(analogRead(TEMP_SENSOR_PIN), analogMin, analogMax, tempMin, tempMax);

  // Subtract the oldest temperature reading from the sum
  temperatureSum -= temperatureReadings[temperatureIndex];

  // Replace the oldest temperature reading with the new reading
  temperatureReadings[temperatureIndex] = rawTemperature;

  // Add the new reading to the sum
  temperatureSum += rawTemperature;

  // Calculate the average temperature
  temperature = temperatureSum / FILTER_SIZE;

  // Increment the index, wrapping back to 0 if it exceeds FILTER_SIZE
  temperatureIndex = (temperatureIndex + 1) % FILTER_SIZE;

  // Update PID setpoint from potentiometer
  setPoint = SETPOINT;

  // Run PID controller
  myPID.run();

  // Control heater and fan based on temperature
  if (temperature > setPoint + TEMP_THRESHOLD && !isFanOn) {
    // Temperature is above setpoint + threshold, turn on fan
    digitalWrite(FAN_PIN, HIGH);
    isFanOn = true;
    // Turn off heater only if it's on
    if (isHeaterOn) {
      analogWrite(HEATER_PIN, 0);  // Turn off heater
      isHeaterOn = false;
    }
  } else if (temperature < setPoint - TEMP_THRESHOLD && !isHeaterOn) {
    // Temperature is below setpoint - threshold, turn on heater
    // Turn off fan only if it's on
    if (isFanOn) {
      digitalWrite(FAN_PIN, LOW);
      isFanOn = false;
    }
    analogWrite(HEATER_PIN, outputVal);  // Control heater based on PID output
    isHeaterOn = true;
  }

  // Send temperature and control information to serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Heater Output: ");
  Serial.print(outputVal);
  Serial.print(", Fan Status: ");
  Serial.println(digitalRead(FAN_PIN));

  // Delay for stability
  delay(100);
}


