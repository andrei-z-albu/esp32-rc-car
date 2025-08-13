#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

// OLED CONFIG
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_I2C_ADDRESS 0x3C
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// PCA9685 Configuration
#define PWM_FREQ 50 // Standard servo frequency
#define PCA9685_I2C_ADDRESS 0x40 // Default address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS);
#define SERVO_MIN_PULSE 150 // Minimum pulse length out of 4096
#define SERVO_MAX_PULSE 600 // Maximum pulse length out of 4096
#define STEERING_CHANNEL 0 // First channel on PCA9685

// Pin Configuration
#define STEERING_SERVO_PIN 13 // Changed to GPIO13 - safer choice for ESP32-WROOM
#define I2C_SDA 21 // Standard I2C pins for ESP32-WROOM
#define I2C_SCL 22 // Standard I2C pins for ESP32-WROOM

// Motor pins
// Left motor
constexpr int PWM_L = 18;
constexpr int INA_L = 19;
constexpr int INB_L = 13;

// Right motor
constexpr int PWM_R = 5;
constexpr int INA_R = 23;
constexpr int INB_R = 12;   // keep LOW at boot

// Encoders - right encoder pins swapped to fix backwards reading
constexpr int ENC_L_A = 25;
constexpr int ENC_L_B = 26;
constexpr int ENC_R_A = 14;  // Swapped from 27 to 14
constexpr int ENC_R_B = 27;  // Swapped from 14 to 27

// Servo Configuration
#define SERVO_CENTER 90
#define SERVO_MAX_LEFT 180
#define SERVO_MAX_RIGHT 0
#define JOYSTICK_DEADZONE 30

// Motor parameters
#define MAX_PWM 255
#define MIN_PWM 50  // Increased from 50 to overcome static friction
#define STALL_THRESHOLD 180
#define COUNTS_PER_REV 3638  // Calibrated: new = old * (displayed/actual) = 14550 * (0.5/2.0)
#define WHEEL_DIAMETER_M 0.120  // 120mm wheel diameter in meters (Pololu Wild Thumper)
#define WHEEL_CIRCUMFERENCE_M (PI * WHEEL_DIAMETER_M)
#define ODOMETRY_SCALE_FACTOR 1.0  // Removed 10x multiplier now that encoders work correctly

// PID constants - reduced for better stability
#define KP 0.8  // Reduced from 1.0 for less aggressive response
#define KI 0.05 // Reduced from 0.1 for less oscillation
#define KD 0.02 // Reduced from 0.05 for smoother operation

// Global variables
bool servoInitialized = false;
bool isControllerConnected = false;

// Motor control variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long lastLeftCount = 0;
volatile long lastRightCount = 0;

// PID variables
float leftRPMTarget = 0;
float rightRPMTarget = 0;
float leftRPMActual = 0;
float rightRPMActual = 0;
float leftPIDError = 0, leftPIDPrev = 0, leftPIDIntegral = 0;
float rightPIDError = 0, rightPIDPrev = 0, rightPIDIntegral = 0;

// Timing variables
unsigned long lastRPMTime = 0;
unsigned long lastOdometryTime = 0;

// Odometry variables
float totalDistance = 0.0;  // meters
float lastLeftDistance = 0.0;
float lastRightDistance = 0.0;

// NVS persistence
Preferences odomPrefs;
const char* ODOM_NAMESPACE = "odom";
const char* ODOM_KEY = "total_m";
unsigned long lastOdomSaveMs = 0;
float lastSavedDistanceM = 0.0f;

// Top image dimensions
#define TOP_IMAGE_WIDTH 73
#define TOP_IMAGE_HEIGHT 30

// Top bitmap image (73x30 pixels)
static const unsigned char PROGMEM top_image[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x60, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc0, 0x00, 0x00, 0x00, 0x01, 0xfc, 0x00,
0x00, 0x00, 0x18, 0x60, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xe0, 0x00, 0x00, 0x1c, 0x38, 0x00, 0x00,
0x0f, 0xf9, 0x0d, 0xfc, 0x00, 0x00, 0x3c, 0x3e, 0x00, 0x00, 0xff, 0x80, 0x8c, 0x9f, 0x00, 0x00,
0x3c, 0x3f, 0xe0, 0x3f, 0xfb, 0xc0, 0xd9, 0x0f, 0x80, 0x00, 0x1e, 0x3f, 0xff, 0xff, 0x81, 0xf0,
0xd9, 0x6c, 0xe0, 0x00, 0x1f, 0x7e, 0x02, 0x03, 0xc0, 0xf8, 0x4b, 0x68, 0xf0, 0x00, 0x1f, 0xff,
0x03, 0x0f, 0xe0, 0x4c, 0x42, 0x0a, 0xb0, 0x00, 0x0f, 0xff, 0xc7, 0x1e, 0x70, 0x46, 0x42, 0x70,
0xb8, 0x00, 0x0f, 0xff, 0xe7, 0x33, 0x38, 0x66, 0x52, 0x73, 0x38, 0x00, 0x07, 0xff, 0xe7, 0x21,
0x9c, 0x64, 0xda, 0x77, 0x64, 0x00, 0x03, 0xe3, 0xf2, 0x64, 0x86, 0x60, 0x9a, 0x11, 0x40, 0x00,
0x00, 0x01, 0xf0, 0x4c, 0x8e, 0x40, 0x92, 0x31, 0x6e, 0x00, 0x01, 0x80, 0xe0, 0x4c, 0x9f, 0x01,
0x93, 0xff, 0x36, 0x00, 0x00, 0xf1, 0xc6, 0xcc, 0x9f, 0x11, 0x37, 0xe7, 0xc6, 0x00, 0x00, 0x7f,
0xce, 0x8c, 0x9f, 0x31, 0x7c, 0x00, 0xcf, 0x00, 0x00, 0x3f, 0x0c, 0x88, 0x86, 0x1b, 0xe0, 0x00,
0x3f, 0x00, 0x00, 0x0e, 0x0c, 0xc9, 0x86, 0x1f, 0x80, 0x00, 0x1f, 0x00, 0x00, 0x03, 0x8c, 0x49,
0x8c, 0x7c, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0xf8, 0x43, 0x9f, 0xe0, 0x00, 0x00, 0x0e, 0x00,
0x00, 0x00, 0x3e, 0x67, 0x9f, 0x80, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x00,
0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// D-pad arrow icons (16x16 pixels)
static const unsigned char PROGMEM up_arrow[] = {
0x00, 0x00,
0x00, 0x00,
0x01, 0x80,
0x03, 0xC0,
0x07, 0xE0,
0x0F, 0xF0,
0x1F, 0xF8,
0x3F, 0xFC,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x00, 0x00
};

static const unsigned char PROGMEM down_arrow[] = {
0x00, 0x00,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x07, 0xE0,
0x3F, 0xFC,
0x1F, 0xF8,
0x0F, 0xF0,
0x07, 0xE0,
0x03, 0xC0,
0x01, 0x80,
0x00, 0x00,
0x00, 0x00
};

static const unsigned char PROGMEM left_arrow[] = {
0x00, 0x00,
0x00, 0xC0,
0x01, 0xC0,
0x03, 0xC0,
0x07, 0xC0,
0x0F, 0xC0,
0x1F, 0xC0,
0x3F, 0xC0,
0x3F, 0xC0,
0x1F, 0xC0,
0x0F, 0xC0,
0x07, 0xC0,
0x03, 0xC0,
0x01, 0xC0,
0x00, 0xC0,
0x00, 0x00
};

static const unsigned char PROGMEM right_arrow[] = {
0x00, 0x00,
0x03, 0x00,
0x03, 0x80,
0x03, 0xC0,
0x03, 0xE0,
0x03, 0xF0,
0x03, 0xF8,
0x03, 0xFC,
0x03, 0xFC,
0x03, 0xF8,
0x03, 0xF0,
0x03, 0xE0,
0x03, 0xC0,
0x03, 0x80,
0x03, 0x00,
0x00, 0x00
};

// Button Icons (16x16 pixels)
static const unsigned char PROGMEM triangle_icon[] = {
0b00000000, 0b00000000,
0b00000000, 0b10000000,
0b00000001, 0b11000000,
0b00000011, 0b11100000,
0b00000111, 0b11110000,
0b00001111, 0b11111000,
0b00011111, 0b11111100,
0b00111111, 0b11111110,
0b01111111, 0b11111111,
0b11111111, 0b11111111,
0b00000000, 0b00000000,
0b00000000, 0b00000000,
0b00000000, 0b00000000,
0b00000000, 0b00000000,
0b00000000, 0b00000000,
0b00000000, 0b00000000
};

static const unsigned char PROGMEM circle_icon[] = {
0b00000111, 0b11100000,
0b00011111, 0b11111000,
0b00111111, 0b11111100,
0b01111111, 0b11111110,
0b01111111, 0b11111110,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b01111111, 0b11111110,
0b01111111, 0b11111110,
0b00111111, 0b11111100,
0b00011111, 0b11111000,
0b00000111, 0b11100000,
0b00000000
};

static const unsigned char PROGMEM square_icon[] = {
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111,
0b11111111, 0b11111111
};

static const unsigned char PROGMEM x_icon[] = {
0b11000000, 0b00000011,
0b11100000, 0b00000111,
0b01110000, 0b00001110,
0b00111000, 0b00011100,
0b00011100, 0b00111000,
0b00001110, 0b01110000,
0b00000111, 0b11100000,
0b00000011, 0b11000000,
0b00000011, 0b11000000,
0b00000111, 0b11100000,
0b00001110, 0b01110000,
0b00011100, 0b00111000,
0b00111000, 0b00011100,
0b01110000, 0b00001110,
0b11100000, 0b00000111,
0b11000000, 0b00000011
};

// Encoder interrupt handlers
void IRAM_ATTR leftEncoderISR() {
    static unsigned char lastEncoded = 0;
    unsigned char encoded = (digitalRead(ENC_L_A) << 1) | digitalRead(ENC_L_B);
    unsigned char sum = (lastEncoded << 2) | encoded;
    
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        leftEncoderCount++;
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        leftEncoderCount--;
    }
    
    lastEncoded = encoded;
}

void IRAM_ATTR rightEncoderISR() {
    static unsigned char lastEncoded = 0;
    unsigned char encoded = (digitalRead(ENC_R_A) << 1) | digitalRead(ENC_R_B);
    unsigned char sum = (lastEncoded << 2) | encoded;
    
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        rightEncoderCount++;
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        rightEncoderCount--;
    }
    
    lastEncoded = encoded;
}

// Motor control function
void setMotorSpeed(int leftPWM, int rightPWM) {
    
    // Add PWM deadband to prevent tiny movements
    const int PWM_DEADBAND = 20;
    
    // Static variables to track previous motor states for reduced debug noise
    static int prevLeftPWM = 0;
    static int prevRightPWM = 0;
    static unsigned long lastMotorDebug = 0;
    
    // Only print debug info if motors changed significantly or every 2 seconds
    bool shouldDebug = (abs(leftPWM - prevLeftPWM) > 20 || abs(rightPWM - prevRightPWM) > 20 || 
                      (millis() - lastMotorDebug > 2000 && (leftPWM != 0 || rightPWM != 0)));
    
    if (shouldDebug) {
        Serial.print("Motor PWM - Left: ");
        Serial.print(leftPWM);
        Serial.print(" Right: ");
        Serial.print(rightPWM);
        Serial.print(" | Left Enc: ");
        Serial.print(leftEncoderCount);
        Serial.print(" Right Enc: ");
        Serial.print(rightEncoderCount);
        Serial.print(" | Left RPM: ");
        Serial.print(leftRPMActual);
        Serial.print(" Right RPM: ");
        Serial.println(rightRPMActual);
        lastMotorDebug = millis();
        prevLeftPWM = leftPWM;
        prevRightPWM = rightPWM;
    }
    
    // Left motor
    if (leftPWM > PWM_DEADBAND) {
        // Forward - ensure minimum PWM for movement
        int actualPWM = max(leftPWM, MIN_PWM);
        digitalWrite(INA_L, HIGH);
        digitalWrite(INB_L, LOW);
        analogWrite(PWM_L, constrain(actualPWM, MIN_PWM, MAX_PWM));
        if (shouldDebug) {
            Serial.print("Left motor FORWARD at PWM: ");
            Serial.print(actualPWM);
            Serial.print(" (INA_L=HIGH, INB_L=LOW)");
            Serial.println();
        }
    } else if (leftPWM < -PWM_DEADBAND) {
        // Reverse - ensure minimum PWM for movement
        int actualPWM = max(-leftPWM, MIN_PWM);
        digitalWrite(INA_L, LOW);
        digitalWrite(INB_L, HIGH);
        analogWrite(PWM_L, constrain(actualPWM, MIN_PWM, MAX_PWM));
        if (shouldDebug) {
            Serial.print("Left motor REVERSE at PWM: ");
            Serial.print(actualPWM);
            Serial.print(" (INA_L=LOW, INB_L=HIGH)");
            Serial.println();
        }
    } else {
        // Stop motor - set both direction pins LOW for brake
        digitalWrite(INA_L, LOW);
        digitalWrite(INB_L, LOW);
        analogWrite(PWM_L, 0);
        if (shouldDebug) {
            Serial.println("Left motor STOPPED (INA_L=LOW, INB_L=LOW)");
        }
    }
    
    // Right motor - ensure identical behavior
    if (rightPWM > PWM_DEADBAND) {
        // Forward - ensure minimum PWM for movement
        int actualPWM = max(rightPWM, MIN_PWM);
        digitalWrite(INA_R, HIGH);
        digitalWrite(INB_R, LOW);
        analogWrite(PWM_R, constrain(actualPWM, MIN_PWM, MAX_PWM));
        if (shouldDebug) {
            Serial.print("Right motor FORWARD at PWM: ");
            Serial.print(actualPWM);
            Serial.print(" (INA_R=HIGH, INB_R=LOW)");
            Serial.println();
        }
    } else if (rightPWM < -PWM_DEADBAND) {
        // Reverse - ensure minimum PWM for movement
        int actualPWM = max(-rightPWM, MIN_PWM);
        digitalWrite(INA_R, LOW);
        digitalWrite(INB_R, HIGH);
        analogWrite(PWM_R, constrain(actualPWM, MIN_PWM, MAX_PWM));
        if (shouldDebug) {
            Serial.print("Right motor REVERSE at PWM: ");
            Serial.print(actualPWM);
            Serial.print(" (INA_R=LOW, INB_R=HIGH)");
            Serial.println();
        }
    } else {
        // Stop motor - set both direction pins LOW for brake
        digitalWrite(INA_R, LOW);
        digitalWrite(INB_R, LOW);
        analogWrite(PWM_R, 0);
        if (shouldDebug) {
            Serial.println("Right motor STOPPED (INA_R=LOW, INB_R=LOW)");
        }
    }
}

// Calculate RPM from encoder counts
void calculateRPM() {
    unsigned long currentTime = millis();
    if (currentTime - lastRPMTime >= 100) {  // Update every 100ms
        float deltaTime = (currentTime - lastRPMTime) / 1000.0;  // seconds
        
        long leftDelta = leftEncoderCount - lastLeftCount;
        long rightDelta = rightEncoderCount - lastRightCount;

        // Calculate RPM - preserve sign for direction
        leftRPMActual = (leftDelta / (float)COUNTS_PER_REV) * (60.0 / deltaTime);
        rightRPMActual = (rightDelta / (float)COUNTS_PER_REV) * (60.0 / deltaTime);
        
        lastLeftCount = leftEncoderCount;
        lastRightCount = rightEncoderCount;
        lastRPMTime = currentTime;
        
        // Debug RPM calculation
        static unsigned long lastRPMDebug = 0;
        if (currentTime - lastRPMDebug >= 1000) {  // Every second
            Serial.print("RPM Debug - Left delta: ");
            Serial.print(leftDelta);
            Serial.print(" Right delta: ");
            Serial.print(rightDelta);
            Serial.print(" Left RPM: ");
            Serial.print(leftRPMActual);
            Serial.print(" Right RPM: ");
            Serial.println(rightRPMActual);
            lastRPMDebug = currentTime;
        }
    }
}

// PID control for motors
void updatePID() {
    static unsigned long lastPIDTime = 0;
    unsigned long currentTime = millis();
    
    // Only update PID every 50ms for stability (was updating every 5ms)
    if (currentTime - lastPIDTime < 50) {
        return;
    }
    lastPIDTime = currentTime;
    
    int leftPWM = 0;
    int rightPWM = 0;
    
    // Only apply PID if we have a target speed
    if (abs(leftRPMTarget) > 1 || abs(rightRPMTarget) > 1) {
        // Left motor PID
        leftPIDError = leftRPMTarget - leftRPMActual;
        leftPIDIntegral += leftPIDError;
        // Limit integral windup - reduce for better stability
        leftPIDIntegral = constrain(leftPIDIntegral, -100, 100);
        float leftPIDDerivative = leftPIDError - leftPIDPrev;
        float leftPIDOutput = KP * leftPIDError + KI * leftPIDIntegral + KD * leftPIDDerivative;
        leftPIDPrev = leftPIDError;
        
        // Right motor PID
        rightPIDError = rightRPMTarget - rightRPMActual;
        rightPIDIntegral += rightPIDError;
        // Limit integral windup - reduce for better stability
        rightPIDIntegral = constrain(rightPIDIntegral, -100, 100);
        float rightPIDDerivative = rightPIDError - rightPIDPrev;
        float rightPIDOutput = KP * rightPIDError + KI * rightPIDIntegral + KD * rightPIDDerivative;
        rightPIDPrev = rightPIDError;
        
        // Calculate base PWM from target RPM - handle forward and reverse consistently
        // Use the target RPM range appropriate for high-speed mode
        int maxRPM = (abs(leftRPMTarget) > 130 || abs(rightRPMTarget) > 130) ? 200 : 130;
        
        // Calculate base PWM preserving direction - use higher minimum for reverse
        int baseMinPWM = MIN_PWM;
        if (leftRPMTarget < 0 || rightRPMTarget < 0) {
            baseMinPWM = MIN_PWM + 20;  // Higher minimum PWM for reverse to match forward speed
        }
        
        leftPWM = map(abs(leftRPMTarget), 0, maxRPM, baseMinPWM, MAX_PWM);
        rightPWM = map(abs(rightRPMTarget), 0, maxRPM, baseMinPWM, MAX_PWM);
        
        // Apply PID correction - handle reverse direction properly
        // For reverse, we need to increase PWM magnitude when actual speed is lower than target
        if (leftRPMTarget < 0) {
            // Reverse: if actual speed is less negative than target, we need more PWM
            leftPWM += abs((int)leftPIDOutput);  // Always add correction for reverse
        } else {
            // Forward: normal PID correction
            leftPWM += (int)leftPIDOutput;
        }
        
        if (rightRPMTarget < 0) {
            // Reverse: if actual speed is less negative than target, we need more PWM
            rightPWM += abs((int)rightPIDOutput);  // Always add correction for reverse
        } else {
            // Forward: normal PID correction
            rightPWM += (int)rightPIDOutput;
        }
        
        // Apply direction AFTER all calculations - this prevents sign confusion
        if (leftRPMTarget < 0) leftPWM = -leftPWM;
        if (rightRPMTarget < 0) rightPWM = -rightPWM;
        
        // CRITICAL: Ensure minimum PWM when target is non-zero to prevent motor stopping
        // This prevents the load imbalance issue where one motor stops completely
        if (leftRPMTarget != 0) {
            if (leftPWM > 0 && leftPWM < MIN_PWM) leftPWM = MIN_PWM;
            if (leftPWM < 0 && leftPWM > -MIN_PWM) leftPWM = -MIN_PWM;
        }
        if (rightRPMTarget != 0) {
            if (rightPWM > 0 && rightPWM < MIN_PWM) rightPWM = MIN_PWM;
            if (rightPWM < 0 && rightPWM > -MIN_PWM) rightPWM = -MIN_PWM;
        }
        
        // Load balancing: prevent extreme PWM differences that cause one motor to stop
        // This happens when one motor has no load (robot lifted) and spins too fast
        // Skip load balancing in high-speed mode to allow maximum performance
        if (leftRPMTarget != 0 && rightRPMTarget != 0 && maxRPM != 300) {
            int pwmDifference = abs(leftPWM - rightPWM);
            const int MAX_PWM_DIFFERENCE = 100;  // Maximum allowed PWM difference
            
            if (pwmDifference > MAX_PWM_DIFFERENCE) {
                // Calculate average PWM and constrain both motors to reasonable range around it
                int avgPWM = (abs(leftPWM) + abs(rightPWM)) / 2;
                int maxAllowedPWM = avgPWM + (MAX_PWM_DIFFERENCE / 2);
                int minAllowedPWM = max(MIN_PWM, avgPWM - (MAX_PWM_DIFFERENCE / 2));
                
                // Apply constraints while preserving direction
                if (leftPWM > 0) {
                    leftPWM = constrain(leftPWM, minAllowedPWM, maxAllowedPWM);
                } else {
                    leftPWM = constrain(leftPWM, -maxAllowedPWM, -minAllowedPWM);
                }
                
                if (rightPWM > 0) {
                    rightPWM = constrain(rightPWM, minAllowedPWM, maxAllowedPWM);
                } else {
                    rightPWM = constrain(rightPWM, -maxAllowedPWM, -minAllowedPWM);
                }
                
                // Reset PID integrals when load balancing kicks in to prevent windup
                if (pwmDifference > MAX_PWM_DIFFERENCE * 1.5) {
                    leftPIDIntegral *= 0.5;  // Reduce but don't completely reset
                    rightPIDIntegral *= 0.5;
                    Serial.println("Load balancing applied - PWM difference too large");
                }
            }
        }
        
        // Constrain PWM values
        leftPWM = constrain(leftPWM, -MAX_PWM, MAX_PWM);
        rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);
        
        // Improved stall detection - reset integral if stalled
        if (abs(leftPWM) > STALL_THRESHOLD && abs(leftRPMActual) < 10) {
            leftPWM = (leftPWM > 0) ? leftPWM * 0.8 : leftPWM * 0.8;  // Reduce PWM by 20%
            leftPIDIntegral = 0;  // Reset integral to prevent windup
            Serial.println("Left motor stall detected - reducing PWM");
        }
        if (abs(rightPWM) > STALL_THRESHOLD && abs(rightRPMActual) < 10) {
            rightPWM = (rightPWM > 0) ? rightPWM * 0.8 : rightPWM * 0.8;  // Reduce PWM by 20%
            rightPIDIntegral = 0;  // Reset integral to prevent windup
            Serial.println("Right motor stall detected - reducing PWM");
        }
        
        // Debug output - show both forward and reverse clearly
        static unsigned long lastDebugTime = 0;
        if (millis() - lastDebugTime > 1000) {  // Every 1000ms
            Serial.print("PID Debug - Target L:");
            Serial.print(leftRPMTarget);
            Serial.print(" R:");
            Serial.print(rightRPMTarget);
            Serial.print(" | Actual L:");
            Serial.print(leftRPMActual);
            Serial.print(" R:");
            Serial.print(rightRPMActual);
            Serial.print(" | PWM L:");
            Serial.print(leftPWM);
            Serial.print(" R:");
            Serial.print(rightPWM);
            Serial.print(" | Error L:");
            Serial.print(leftPIDError);
            Serial.print(" R:");
            Serial.print(rightPIDError);
            Serial.print(" | MinPWM:");
            Serial.print(baseMinPWM);
            Serial.print(" | MaxRPM:");
            Serial.print(maxRPM);
            Serial.print(" | HighSpeed:");
            Serial.print(maxRPM == 300 ? "YES" : "NO");
            
            // Add PWM calculation debugging for high-speed mode
            if (maxRPM == 300) {
                int expectedLeftPWM = map(abs(leftRPMTarget), 0, maxRPM, baseMinPWM, MAX_PWM);
                int expectedRightPWM = map(abs(rightRPMTarget), 0, maxRPM, baseMinPWM, MAX_PWM);
                Serial.print(" | Expected PWM L:");
                Serial.print(expectedLeftPWM);
                Serial.print(" R:");
                Serial.print(expectedRightPWM);
                Serial.print(" | PID Out L:");
                Serial.print((int)leftPIDOutput);
                Serial.print(" R:");
                Serial.print((int)rightPIDOutput);
            }
            Serial.println();
            lastDebugTime = millis();
        }
    } else {
        // Target is zero - stop motors completely and reset PID state
        leftPWM = 0;
        rightPWM = 0;
        leftPIDIntegral = 0;  // Reset integral terms
        rightPIDIntegral = 0;
        leftPIDPrev = 0;
        rightPIDPrev = 0;
        leftPIDError = 0;
        rightPIDError = 0;
    }
    
    setMotorSpeed(leftPWM, rightPWM);
}

// Display odometry in top area (replacing bitmap when controller connected)
void displayOdometry() {
    // Large font for main distance reading
    display.setTextSize(3);  // Large text (~24px height)
    display.setTextColor(SSD1306_WHITE);
    
    String distanceText;
    if (totalDistance >= 1000.0) {
        distanceText = String(totalDistance / 1000.0, 1) + "km";
    } else if (totalDistance >= 100.0) {
        distanceText = String((int)totalDistance) + "m";
    } else {
        distanceText = String(totalDistance, 1) + "m";
    }
    
    // Calculate text width for centering (approximate)
    int textWidth = distanceText.length() * 18;  // ~18 pixels per character at size 3
    int startX = (SCREEN_WIDTH - textWidth) / 2;
    if (startX < 0) startX = 0;
    
    // Position at top center
    display.setCursor(startX, 5);
    display.println(distanceText);
}

// Update odometry
void updateOdometry() {
    unsigned long currentTime = millis();
    if (currentTime - lastOdometryTime >= 50) {  // Update every 50ms
        float leftDistance = (leftEncoderCount / (float)COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M * ODOMETRY_SCALE_FACTOR;
        float rightDistance = (rightEncoderCount / (float)COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M * ODOMETRY_SCALE_FACTOR;
        
        float leftDelta = leftDistance - lastLeftDistance;
        float rightDelta = rightDistance - lastRightDistance;
        float averageDelta = (leftDelta + rightDelta) / 2.0;
        
        totalDistance += abs(averageDelta);
        
        // Persist odometry if enough time or distance has passed
        bool shouldSave = false;
        if (currentTime - lastOdomSaveMs > 5000) shouldSave = true;              // time based
        if ((totalDistance - lastSavedDistanceM) > 1.0f) shouldSave = true;      // distance based
        if (shouldSave) {
            odomPrefs.putFloat(ODOM_KEY, totalDistance);
            lastOdomSaveMs = currentTime;
            lastSavedDistanceM = totalDistance;
            Serial.printf("Odometry saved: %.2f meters\n", totalDistance);
        }
        
        // Debug output every second
        static unsigned long lastDebugTime = 0;
        if (currentTime - lastDebugTime >= 1000) {
            Serial.print("Odometry - Encoder counts: L=");
            Serial.print(leftEncoderCount);
            Serial.print(" R=");
            Serial.print(rightEncoderCount);
            Serial.print(" Distance: L=");
            Serial.print(leftDistance, 3);
            Serial.print("m R=");
            Serial.print(rightDistance, 3);
            Serial.print("m Total=");
            Serial.print(totalDistance, 3);
            Serial.println("m");
            lastDebugTime = currentTime;
        }
        
        lastLeftDistance = leftDistance;
        lastRightDistance = rightDistance;
        lastOdometryTime = currentTime;
    }
}

// Motor control based on controller input
void updateMotorControl(uint16_t throttle, uint16_t brake) {
    // Debug input values
    Serial.print("Motor Control - Throttle: ");
    Serial.print(throttle);
    Serial.print(" Brake: ");
    Serial.print(brake);
    
    // Use lower deadzone to ensure triggers work
    const int TRIGGER_DEADZONE = 20;  // Reduced further
    
    // Apply deadzone to triggers
    int effectiveThrottle = (throttle > TRIGGER_DEADZONE) ? throttle - TRIGGER_DEADZONE : 0;
    int effectiveBrake = (brake > TRIGGER_DEADZONE) ? brake - TRIGGER_DEADZONE : 0;
    
    Serial.print(" Effective - Throttle: ");
    Serial.print(effectiveThrottle);
    Serial.print(" Brake: ");
    Serial.print(effectiveBrake);
    
    // Adjust the mapping range since we removed the deadzone
    const int EFFECTIVE_MAX = 1023 - TRIGGER_DEADZONE;
    
    if (effectiveThrottle > 0 && effectiveBrake <= 0) {
        // Forward movement - capped at motor specification (130 RPM at 6V)
        float speed = map(effectiveThrottle, 0, EFFECTIVE_MAX, 20, 130);  // 20-130 RPM range (motor spec)
        leftRPMTarget = speed;
        rightRPMTarget = speed;
        Serial.print(" -> FORWARD speed: ");
        Serial.println(speed);
    } else if (effectiveBrake > 0 && effectiveThrottle <= 0) {
        // Reverse movement - capped at motor specification (130 RPM at 6V)
        float speed = map(effectiveBrake, 0, EFFECTIVE_MAX, 25, 130);  // 25-130 RPM range (motor spec)
        leftRPMTarget = -speed;  // Negative for reverse
        rightRPMTarget = -speed; // Negative for reverse
        Serial.print(" -> REVERSE speed: ");
        Serial.println(-speed);
    } else if (effectiveThrottle > 0 && effectiveBrake > 0) {
        // Both triggers pressed - stop
        leftRPMTarget = 0;
        rightRPMTarget = 0;
        Serial.println(" -> BOTH TRIGGERS - STOP");
    } else {
        // No triggers pressed - stop
        leftRPMTarget = 0;
        rightRPMTarget = 0;
        leftPIDIntegral = 0;  // Reset integral term
        rightPIDIntegral = 0;
        // Don't spam the serial with stop messages
        static unsigned long lastStopMessage = 0;
        if (millis() - lastStopMessage > 3000) {
            Serial.println(" -> NO TRIGGERS - STOP");
            lastStopMessage = millis();
        }
    }
}

void displayTopImage() {
    // Calculate center position for top image
    int x = (SCREEN_WIDTH - TOP_IMAGE_WIDTH) / 2;
    display.drawBitmap(x, 0, top_image, TOP_IMAGE_WIDTH, TOP_IMAGE_HEIGHT, SSD1306_WHITE);
}

void displayButtonIcon(const unsigned char* icon) {
    display.clearDisplay();
    // Always display the top image first
    displayTopImage();
    // Calculate center position for 16x16 button icon
    // Place it in the lower half of the screen
    int x = (SCREEN_WIDTH - 16) / 2;
    int y = TOP_IMAGE_HEIGHT + ((SCREEN_HEIGHT - TOP_IMAGE_HEIGHT - 16) / 2);
    display.drawBitmap(x, y, icon, 16, 16, SSD1306_WHITE);
    display.display();
}

void printCenteredText(const char* text) {
    display.clearDisplay();
    // Always display the top image first
    displayTopImage();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    // Calculate text width
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    // Center the text in the remaining space below the top image
    int x = (SCREEN_WIDTH - w) / 2;
    int y = TOP_IMAGE_HEIGHT + ((SCREEN_HEIGHT - TOP_IMAGE_HEIGHT - h) / 2);
    display.setCursor(x, y);
    display.println(text);
    display.display();
}

// PIN Connections
const int builtInLed = 2;

// D-pad Buttons
const uint16_t leftButton = 0x08;
const uint16_t rightButton = 0x04;
const uint16_t upButton = 0x01;
const uint16_t downButton = 0x02;

// PS4 Buttons
const uint16_t triangleButton = 0x0008;
const uint16_t squareButton = 0x0004;
const uint16_t circleButton = 0x0002;
const uint16_t xButton = 0x0001;
const uint16_t l1Button = 0x0010; // Left shoulder button
const uint16_t r1Button = 0x0020; // Right shoulder button

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void printOLED(String text) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(text);
    display.display();
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            isControllerConnected = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    Serial.printf("CALLBACK: Controller disconnected!\n");
    isControllerConnected = false;
    // Remove the controller from myControllers
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            display.clearDisplay();
            displayTopImage();
            printCenteredText("Connect Controller");
            break;
        }
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void drawBar(int x, uint16_t value, bool isLeft) {
    const int BAR_WIDTH = 5;
    const int MIN_HEIGHT = 5;  // Minimum height (5x5 square)
    const int MAX_HEIGHT = SCREEN_HEIGHT - 4;  // Use almost full screen height, leave 4px margin at top
    
    // For right bar, adjust x to align to right edge
    if (!isLeft) {
        x = x - BAR_WIDTH + 1;  // Adjusted to align with guide lines
    }
    
    if (value == 0) {
        // Draw 5x5 square at bottom when no pressure
        display.fillRect(x, SCREEN_HEIGHT - MIN_HEIGHT, BAR_WIDTH, MIN_HEIGHT, SSD1306_WHITE);
    } else {
        // Calculate height based on value (0-1023)
        int height = map(value, 0, 1023, MIN_HEIGHT, MAX_HEIGHT);
        int y = SCREEN_HEIGHT - height;
        display.fillRect(x, y, BAR_WIDTH, height, SSD1306_WHITE);
    }
}

void drawJoystickLine(int16_t value, int yPosition) {
    const int LINE_HEIGHT = 5;
    const int DEAD_ZONE = 30;
    const int MARGIN = 6;  // Space for vertical bars
    const int CENTER_X = SCREEN_WIDTH / 2;
    
    // Start with center square
    int width = LINE_HEIGHT;  // 5x5 square when centered
    int startX = CENTER_X - (LINE_HEIGHT / 2);
    
    // Only extend if outside dead zone
    if (abs(value) > DEAD_ZONE) {
        if (value < 0) {  // Moving left
            // Map -508 to -30 to pixels from center to left edge + margin
            width = map(abs(value), DEAD_ZONE, 508, LINE_HEIGHT, CENTER_X - MARGIN);
            startX = CENTER_X - width + (LINE_HEIGHT / 2);
        } else {  // Moving right
            // Map 30 to 512 to pixels from center to right edge - margin
            width = map(value, DEAD_ZONE, 512, LINE_HEIGHT, CENTER_X - MARGIN);
        }
    }
    
    display.fillRect(startX, yPosition, width, LINE_HEIGHT, SSD1306_WHITE);
}

// Function to set servo angle using PCA9685
void setServoAngle(int angle) {
    if (!servoInitialized) return;
    
    angle = constrain(angle, 0, 180);
    uint16_t pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(STEERING_CHANNEL, 0, pulse);
}

void updateSteering(int16_t axisX) {
    if (!servoInitialized) return;
    
    // Simple direct mapping from joystick to servo angle
    int angle;
    
    if (abs(axisX) < JOYSTICK_DEADZONE) {
        angle = SERVO_CENTER;
    } else {
        // Direct linear mapping
        angle = map(axisX, -511, 512, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
    }
    
    setServoAngle(angle);
}

void processGamepad(ControllerPtr ctl) {
    uint16_t currentButtons = ctl->buttons();
    uint16_t currentDPad = ctl->dpad();
    uint16_t currentThrottle = ctl->throttle();
    uint16_t currentBrake = ctl->brake();
    int16_t axisX = ctl->axisX();      // Left joystick X-axis
    int16_t axisRX = ctl->axisRX();    // Right joystick X-axis

    // Debug trigger values
    static unsigned long lastTriggerDebug = 0;
    if (millis() - lastTriggerDebug > 500) {
        if (currentThrottle > 0 || currentBrake > 0) {
            Serial.print("Triggers - Throttle: ");
            Serial.print(currentThrottle);
            Serial.print(" Brake: ");
            Serial.print(currentBrake);
            Serial.print(" Buttons: 0x");
            Serial.println(currentButtons, HEX);
        }
        lastTriggerDebug = millis();
    }

    // Motor diagnostics test - hold L1 + R1 for direct motor tests
    bool motorDiagTest = (currentButtons & l1Button) && (currentButtons & r1Button);
    
    // Direct motor test mode - hold Square button for direct PWM control
    bool directMotorTest = (currentButtons & squareButton);
    
    // Check for high-speed mode (hold Triangle + Circle for max speed override)
    bool highSpeedMode = (currentButtons & xButton);
    if (highSpeedMode) {
        Serial.println("High speed mode detected!");
    }
    
    // Debug which path is taken
    Serial.print("Mode selection - MotorDiag:");
    Serial.print(motorDiagTest ? "YES" : "NO");
    Serial.print(" DirectTest:");
    Serial.print(directMotorTest ? "YES" : "NO");
    Serial.print(" HighSpeed:");
    Serial.print(highSpeedMode ? "YES" : "NO");
    Serial.print(" → Taking path: ");
    
    // Update steering based on left joystick X-axis
    updateSteering(axisX);
    
    // Update motor control based on mode
    if (motorDiagTest) {
        Serial.println("MOTOR_DIAG");
        // Motor diagnostics mode - test each motor individually
        static unsigned long lastDiagSwitch = 0;
        static int diagPhase = 0;
        
        if (millis() - lastDiagSwitch > 3000) {  // Switch every 3 seconds
            diagPhase = (diagPhase + 1) % 6;
            lastDiagSwitch = millis();
            
            // Reset encoder counts at start of each phase for clear comparison
            leftEncoderCount = 0;
            rightEncoderCount = 0;
        }
        
        switch (diagPhase) {
            case 0:
                // Both motors forward
                setMotorSpeed(150, 150);
                Serial.println("DIAG: Both motors FORWARD at PWM 150");
                break;
            case 1:
                // Both motors reverse
                setMotorSpeed(-150, -150);
                Serial.println("DIAG: Both motors REVERSE at PWM -150");
                break;
            case 2:
                // Left motor only forward
                setMotorSpeed(150, 0);
                Serial.println("DIAG: LEFT motor FORWARD at PWM 150, RIGHT stopped");
                break;
            case 3:
                // Right motor only forward
                setMotorSpeed(0, 150);
                Serial.println("DIAG: LEFT stopped, RIGHT motor FORWARD at PWM 150");
                break;
            case 4:
                // Left motor only reverse
                setMotorSpeed(-150, 0);
                Serial.println("DIAG: LEFT motor REVERSE at PWM -150, RIGHT stopped");
                break;
            case 5:
                // Right motor only reverse
                setMotorSpeed(0, -150);
                Serial.println("DIAG: LEFT stopped, RIGHT motor REVERSE at PWM -150");
                break;
        }
        
        // Print encoder changes every second during diagnostics
        static unsigned long lastDiagPrint = 0;
        if (millis() - lastDiagPrint > 1000) {
            Serial.print("Encoders - Left: ");
            Serial.print(leftEncoderCount);
            Serial.print(" Right: ");
            Serial.print(rightEncoderCount);
            Serial.print(" | Expected direction: ");
            if (diagPhase == 0) Serial.println("Both +");
            else if (diagPhase == 1) Serial.println("Both -");
            else if (diagPhase == 2) Serial.println("Left +, Right 0");
            else if (diagPhase == 3) Serial.println("Left 0, Right +");
            else if (diagPhase == 4) Serial.println("Left -, Right 0");
            else if (diagPhase == 5) Serial.println("Left 0, Right -");
            lastDiagPrint = millis();
        }
        return; // Skip all other motor control logic
    } else if (directMotorTest) {
        Serial.println("DIRECT_MOTOR_TEST");
        // Direct motor test - bypass all PID and RPM logic
        if (currentThrottle > 100) {
            // Direct forward PWM
            int pwm = map(currentThrottle, 100, 1023, MIN_PWM, MAX_PWM);
            setMotorSpeed(pwm, pwm);
            Serial.print("DIRECT MOTOR TEST - FORWARD PWM: ");
            Serial.println(pwm);
            return; // Skip all other motor control logic
        } else if (currentBrake > 100) {
            // Direct reverse PWM
            int pwm = map(currentBrake, 100, 1023, MIN_PWM, MAX_PWM);
            setMotorSpeed(-pwm, -pwm);
            Serial.print("DIRECT MOTOR TEST - REVERSE PWM: ");
            Serial.println(-pwm);
            return; // Skip all other motor control logic
        } else {
            setMotorSpeed(0, 0);
            Serial.println("DIRECT MOTOR TEST - STOPPED");
            return; // Skip all other motor control logic
        }
    } else if (highSpeedMode) {
        Serial.println("HIGH_SPEED_MODE");
        // High speed mode - bypass normal control
        Serial.println("HIGH SPEED MODE ACTIVE");
        if (currentThrottle > 100) {
            // Forward at high speed - utilize full battery voltage capability
            leftRPMTarget = 200;  // Achievable target for 8.25V (vs 130 RPM at 6V)
            rightRPMTarget = 200;
            Serial.println("HIGH SPEED FORWARD MODE ACTIVATED!");
        } else if (currentBrake > 100) {
            // Reverse at high speed - utilize full battery voltage capability
            leftRPMTarget = -200;  // Achievable target for 8.25V (vs 130 RPM at 6V)
            rightRPMTarget = -200;
            Serial.println("HIGH SPEED REVERSE MODE ACTIVATED!");
        } else {
            leftRPMTarget = 0;
            rightRPMTarget = 0;
            Serial.println("HIGH SPEED MODE - NO TRIGGERS");
        }
        // Skip normal motor control when in high-speed mode
    } else {
        Serial.println("NORMAL_MODE");
        // Normal mode
        updateMotorControl(currentThrottle, currentBrake);
    }
    
    // Always start with a clear display
    display.clearDisplay();
    
    // Show odometry in top area if controller is connected, otherwise show bitmap
    if (isControllerConnected) {
        displayOdometry();
        
        // Show mode indicators below the large odometry reading
        if (highSpeedMode) {
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(50, 30);  // Centered below odometry
            display.println("HIGH SPEED");
        } else if (directMotorTest) {
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(45, 30);  // Centered below odometry
            display.println("DIRECT TEST");
        } else if (motorDiagTest) {
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(35, 30);  // Centered below odometry
            display.println("DIAGNOSTICS");
        }
    } else {
        displayTopImage();
    }

    // Draw brake and throttle bars
    drawBar(1, currentBrake, true); // Left bar for brake
    drawBar(SCREEN_WIDTH - 1, currentThrottle, false); // Right bar for throttle

    // Create arrays to store active buttons and their icons
    const unsigned char* activeIcons[8] = {nullptr};
    int activeCount = 0;

    // Collect all pressed buttons
    if (currentDPad & upButton) activeIcons[activeCount++] = up_arrow;
    if (currentDPad & downButton) activeIcons[activeCount++] = down_arrow;
    if (currentDPad & leftButton) activeIcons[activeCount++] = left_arrow;
    if (currentDPad & rightButton) activeIcons[activeCount++] = right_arrow;
    if (currentButtons & triangleButton) activeIcons[activeCount++] = triangle_icon;
    if (currentButtons & circleButton) activeIcons[activeCount++] = circle_icon;
    if (currentButtons & squareButton) activeIcons[activeCount++] = square_icon;
    if (currentButtons & xButton) activeIcons[activeCount++] = x_icon;

    if (activeCount > 0) {
        // Calculate total width needed (16px per icon + 2px spacing between icons)
        int totalWidth = (activeCount * 16) + ((activeCount - 1) * 2);
        // Calculate starting X to center the entire group
        int startX = (SCREEN_WIDTH - totalWidth) / 2;
        
        // Calculate Y position to be above the joystick lines
        // Leave 16px (icon height) + 2px margin above the top joystick line
        int y = SCREEN_HEIGHT - 11 - 16 - 2;
        
        // Draw all active icons
        for (int i = 0; i < activeCount; i++) {
            int x = startX + (i * 18); // 16px for icon + 2px spacing
            display.drawBitmap(x, y, activeIcons[i], 16, 16, SSD1306_WHITE);
        }
    }

    // Draw joystick lines at bottom
    // Right joystick line (top)
    drawJoystickLine(axisRX, SCREEN_HEIGHT - 11); // 5px height + 1px space + 5px height from bottom
    // Left joystick line (bottom)
    drawJoystickLine(axisX, SCREEN_HEIGHT - 5);    // 5px from bottom

    // Update the display
    display.display();
}

void processControllers() {
    for (auto controller : myControllers) {
        if (controller && controller->isConnected() && controller->hasData()) {
            if (controller->isGamepad()) {
                processGamepad(controller);
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\nESP32-WROOM Starting Up!");

    // Initialize motor pins
    pinMode(PWM_L, OUTPUT);
    pinMode(INA_L, OUTPUT);
    pinMode(INB_L, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(INA_R, OUTPUT);
    pinMode(INB_R, OUTPUT);
    
    // Initialize encoder pins
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    
    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), rightEncoderISR, CHANGE);
    
    // Ensure motors are stopped
    setMotorSpeed(0, 0);
    Serial.println("Motors initialized");

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // Fast I2C mode
    
    // Scan I2C bus
    Serial.println("Scanning I2C bus...");
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    
    // Initialize PCA9685
    if (!pwm.begin()) {
        Serial.println("PCA9685 not found! Check wiring...");
        while (1);
    }
    
    pwm.setPWMFreq(PWM_FREQ);
    servoInitialized = true;
    Serial.println("PCA9685 initialized successfully");
    
    // center the servo
    setServoAngle(SERVO_CENTER);
    
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
        Serial.println("SSD1306 init failed");
    } else {
        Serial.println("Display initialized on I2C (SDA:21, SCL:22)");
        display.clearDisplay();
        displayTopImage();
        printCenteredText("Connect Controller");
    }

    // Initialize Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
    
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    
    // Initialize NVS for odometry
    if (!odomPrefs.begin("odom", false)) { // false means don't erase old data
        Serial.println("NVS prefs failed to initialize");
    } else {
        Serial.println("NVS prefs initialized successfully");
        totalDistance = odomPrefs.getFloat(ODOM_KEY, 0.0f); // Load total distance
        Serial.printf("Loaded total distance: %.2f meters\n", totalDistance);
    }

    Serial.println("Setup complete");
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // Process controller updates
    BP32.update();
    processControllers();
    
    // Update motor control systems if controller is connected
    if (isControllerConnected) {
        calculateRPM();
        updatePID();
        updateOdometry();

        // Save odometry periodically
        // This block is now handled within updateOdometry
    }
    
    // Small delay to prevent tight loop
    delay(5);
}

