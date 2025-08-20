// Hybrid Inverter with Solar Charging - 50Hz Output
// Uses Arduino Uno with timer interrupts for precise 50Hz generation

// Pin definitions
#define SOLAR_INPUT_PIN A0        // Solar panel voltage monitoring
#define BATTERY_LEVEL_PIN A1      // Battery voltage monitoring
#define AC_OUTPUT_PIN 9           // PWM output for inverter (pin 9 supports timer1)
#define CHARGE_CONTROL_PIN 10     // MOSFET for solar charging control
#define BUZZER_PIN 8              // For alarm/notifications
#define LED_INDICATOR_PIN 13      // System status LED

// System parameters
#define FREQUENCY 50              // Output frequency (Hz)
#define SAMPLE_COUNT 40           // Number of samples per sine wave cycle
#define BATTERY_FULL_VOLTAGE 13.8 // Full charge voltage for 12V battery
#define BATTERY_LOW_VOLTAGE 11.0  // Low battery voltage cutoff
#define SOLAR_MIN_VOLTAGE 10.0    // Minimum solar voltage to begin charging

// Sine wave lookup table (pre-calculated for 50Hz)
const int sineTable[SAMPLE_COUNT] = {
  128, 152, 176, 198, 218, 234, 246, 254, 
  255, 254, 246, 234, 218, 198, 176, 152,
  128, 103, 79, 57, 37, 21, 9, 1,
  0, 1, 9, 21, 37, 57, 79, 103,
  128, 152, 176, 198, 218, 234, 246, 254
};

// Global variables
volatile int sampleIndex = 0;
unsigned long lastUpdateTime = 0;
float batteryVoltage = 0;
float solarVoltage = 0;
bool chargingActive = false;
bool inverterEnabled = true;

void setup() {
  // Initialize pins
  pinMode(AC_OUTPUT_PIN, OUTPUT);
  pinMode(CHARGE_CONTROL_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_INDICATOR_PIN, OUTPUT);
  
  // Initialize serial communication for monitoring
  Serial.begin(9600);
  
  // Set up Timer1 for 50Hz interrupt
  noInterrupts();           // Disable all interrupts
  TCCR1A = 0;               // Clear timer registers
  TCCR1B = 0;
  TCNT1 = 0;                // Initialize counter value
  
  // Set compare match register for 50Hz
  // CPU frequency 16MHz, prescaler 8, target 50Hz
  // Formula: OCR1A = (16,000,000 / (prescaler * frequency * samples)) - 1
  OCR1A = 999;              // = (16000000 / (8 * 50 * 40)) - 1
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt
  interrupts();             // Enable all interrupts
  
  Serial.println("Hybrid Inverter System Started");
  Serial.println("50Hz Output | Solar Charging Enabled");
}

// Timer1 interrupt service routine for 50Hz sine wave generation
ISR(TIMER1_COMPA_vect) {
  if (inverterEnabled) {
    analogWrite(AC_OUTPUT_PIN, sineTable[sampleIndex]);
    sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
  } else {
    analogWrite(AC_OUTPUT_PIN, 0); // Turn off output if disabled
  }
}

void loop() {
  // Read system voltages every second
  if (millis() - lastUpdateTime > 1000) {
    lastUpdateTime = millis();
    
    readVoltages();
    manageCharging();
    systemCheck();
    displayStatus();
  }
}

// Read battery and solar voltages
void readVoltages() {
  // Read analog values and convert to voltage (12V system assumption)
  int batteryValue = analogRead(BATTERY_LEVEL_PIN);
  batteryVoltage = (batteryValue / 1023.0) * 5.0 * 3.0; // Voltage divider ratio 3:1
  
  int solarValue = analogRead(SOLAR_INPUT_PIN);
  solarVoltage = (solarValue / 1023.0) * 5.0 * 3.0; // Voltage divider ratio 3:1
}

// Manage solar charging based on conditions
void manageCharging() {
  if (solarVoltage > SOLAR_MIN_VOLTAGE && batteryVoltage < BATTERY_FULL_VOLTAGE) {
    // Enable charging if solar power available and battery not full
    digitalWrite(CHARGE_CONTROL_PIN, HIGH);
    chargingActive = true;
    
    // Pulse LED to indicate charging
    digitalWrite(LED_INDICATOR_PIN, !digitalRead(LED_INDICATOR_PIN));
  } else {
    // Disable charging
    digitalWrite(CHARGE_CONTROL_PIN, LOW);
    chargingActive = false;
    
    // Solid LED when not charging
    digitalWrite(LED_INDICATOR_PIN, batteryVoltage > BATTERY_LOW_VOLTAGE);
  }
}

// System safety checks
void systemCheck() {
  // Low battery protection
  if (batteryVoltage < BATTERY_LOW_VOLTAGE) {
    inverterEnabled = false;
    tone(BUZZER_PIN, 1000, 500); // Alert sound
    Serial.println("WARNING: Battery voltage low! Inverter disabled.");
  } else if (!inverterEnabled && batteryVoltage > BATTERY_LOW_VOLTAGE + 0.5) {
    // Re-enable when voltage recovers
    inverterEnabled = true;
    Serial.println("Battery recovered. Inverter enabled.");
  }
}

// Display system status on serial monitor
void displayStatus() {
  Serial.print("Battery: ");
  Serial.print(batteryVoltage);
  Serial.print("V | Solar: ");
  Serial.print(solarVoltage);
  Serial.print("V | Charging: ");
  Serial.print(chargingActive ? "YES" : "NO");
  Serial.print(" | Inverter: ");
  Serial.println(inverterEnabled ? "ON" : "OFF");
}

// Manual control functions (could be expanded with buttons or communication)
void enableInverter(bool enable) {
  inverterEnabled = enable;
}

void setCharging(bool enable) {
  if (enable && solarVoltage > SOLAR_MIN_VOLTAGE) {
    digitalWrite(CHARGE_CONTROL_PIN, HIGH);
    chargingActive = true;
  } else {
    digitalWrite(CHARGE_CONTROL_PIN, LOW);
    chargingActive = false;
  }
}
