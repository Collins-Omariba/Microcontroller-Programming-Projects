// Include necessary libraries
#include <Keypad.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Define pin numbers for various components
const int InletPumpPin = 11; // Inlet pump pin
const int OutletPumpPin = 13; // Outlet pump pin
const int buzzerPin = 12; // Buzzer pin
const int potPins[] = {A0, A1, A2}; // Potentiometer pins for the three tanks
const int emergencyButtonPin = 3; // Emergency button pin

// Define keypad layout
const byte ROWS = 4; // four rows
const byte COLS = 3; // three columns
char keys[ROWS][COLS] = {  // keypad layout
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// Define pin numbers for keypad rows and columns
byte rowPins[ROWS] = {6, 5, 4, 2}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {9, 8, 7};    // connect to the column pinouts of the keypad

// Define flags for emergency stop and pump running status
volatile bool emergencyStop = false; // Emergency stop flag
bool pumpRunning = false; // Flag to track if a pump is running

// Initialize keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Setup function to initialize serial communication, pin modes and interrupt
void setup() {
  Serial.begin(9600);
  pinMode(InletPumpPin, OUTPUT);
  pinMode(OutletPumpPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Set potentiometer pins as input
  for (int i = 0; i < 3; i++) {
    pinMode(potPins[i], INPUT);
  }

  // Setup interrupt for emergency button
  pinMode(emergencyButtonPin, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(emergencyButtonPin), emergencyStopInterrupt, RISING);
}


// Main loop function to handle keypad input and control pumps
void loop() {
  if (!emergencyStop) { // Proceed only if emergency is not triggered
    char key = keypad.getKey();

    // Check if a key is pressed
    if (key) {
      handleKey(key);
    }

    // Read and process potentiometer values
    for (int i = 0; i < 3; i++) {
      int potValue = analogRead(potPins[i]);

      // Map the potentiometer value to a range of 0 to 5
      float mappedValue = map(potValue, 0, 1023, 0, 5);


      // If tank is not full, allow pump operation
      if (key == keys[i][0]) {
        if (keypad.getState() == HOLD && keypad.isPressed('*')) {
          Serial.print("Start filling pump for Tank ");
          Serial.println(key);
          startPump(InletPumpPin);
          digitalWrite(OutletPumpPin, LOW); // Turn off outlet pump
        } else if (keypad.getState() == HOLD && keypad.isPressed('#')) {
          Serial.print("Start emptying pump for Tank ");
          Serial.println(key);
          startPump(OutletPumpPin);
          digitalWrite(InletPumpPin, LOW); // Turn off inlet pump
        }
      }
    }
  }
}



// Function to handle keypad input
void handleKey(char key) {
  static char selectedTank = '\0'; // Variable to store the selected tank number

  switch (key) {
    case '0':
      if (pumpRunning) {
        Serial.println("Stop pump");
        turnOffPump();
      } else {
        Serial.println("No pump running!");
      }
      break;

    case '1':
    case '2':
    case '3':
      if (!pumpRunning) {
        Serial.print("Selected Tank: ");
        Serial.println(key);
        selectedTank = key; // Record the selected tank
      } else {
        Serial.println("Another pump is already running!");
      }
      break;

    case '*':
      if (selectedTank != '\0') {
        Serial.println("Start filling pump");
        startPump(InletPumpPin);
        digitalWrite(OutletPumpPin, LOW); // Turn off outlet pump
      } else {
        Serial.println("No tank selected!");
      }
      selectedTank = '\0'; // Reset selected tank after operation
      break;

    case '#':
      if (selectedTank != '\0') {
        Serial.println("Start emptying pump");
        startPump(OutletPumpPin);
        digitalWrite(InletPumpPin, LOW); // Turn off inlet pump
      } else {
        Serial.println("No tank selected!");
      }
      selectedTank = '\0'; // Reset selected tank after operation
      break;
  }
}

// Function to start a pump
void startPump(int pumpPin) {
  turnOffPump(); // Turn off all pumps before starting a new one
  digitalWrite(pumpPin, HIGH);
  pumpRunning = true;
}

// Function to turn off all pumps
void turnOffPump() {
  digitalWrite(InletPumpPin, LOW);
  digitalWrite(OutletPumpPin, LOW);
  pumpRunning = false;
}

// Interrupt service routine for emergency stop
void emergencyStopInterrupt() {
  emergencyStop = true;
  turnOffPump(); // Turn off all pumps when emergency is triggered

  int count = 0;

  // Sound the alarm with the buzzer 40 times
  while (emergencyStop and count < 40) {
    digitalWrite(buzzerPin, HIGH);
    Serial.println("Emergency stop triggered!");
    delay(100); 
    count += 1;
    digitalWrite(buzzerPin, LOW);
  }

}

