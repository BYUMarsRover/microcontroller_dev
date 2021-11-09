/* Arduino Mega code
    This code implements the necessary setup for receiving information through
    the serial channel and writing that information to the corresponding pins 
    that connect to the motors. It also implements sending information back to 
    the serial channel to provide feedback to whatever device is connected to 
    the serial port.
    A noteworthy feature that's both helpful and dangerous is implemented in the
    checkClearErrorStates() method, which basically "restarts" any motor that 
    threw an error state and stopped working. This is good for wheen the rover 
    runs into errors during operation, so that it can "ignore" the issue that 
    triggered the error state and keep working. The issue is exactly that, this
    code constantly ignores error states which unavoidably takes a toll on the 
    motors' propper functioning over time.
*/

#include "Globals.h"
#include "Wheels.h"

using namespace std;

const byte STOP_WHEELS = byte(0);
const int MILLISECONDS_FOR_TIMEOUT = 2000;

int last_checkpoint;
bool timeout_happened;

Wheels wheels;

void setup() {
    set_pin_modes();
    establish_connection();

    last_checkpoint = millis();
    timeout_happened = false;

    // Write the default (initialization) parameters contained in the Wheels class to the wheels
    wheels.writeParams();
}

void loop() {
    // There are bytes available to be read
    if (Serial.available() >= 12) {
        
        read_bytes(); // Read the incoming bytes and store them in the wheels data members

        wheels.writeParams(); // Write the values of the wheels data members onto the arduino pins

        send_feedback(); // Send feedback trough the serial channel
    }
    // No bytes available to read
    else {
        timeout_check();// No bytes available, check for timeout
    }
    
    checkClearErrorStates();
}

// This method toggles the enable pin which resets the error state in a motor that failed
void checkClearErrorStates() {
    for (int i = 0; i < NUM_WHEELS; i++) {
        if (wheels.wheelList[i].error) {
            digitalWrite(wheels.wheelList[i].enable_pin, false);
            delay(10);
            digitalWrite(wheels.wheelList[i].enable_pin, true);
            delay(10);
        }
    }
}

// This method reads incoming bytes and stores them in the wheels data members
void read_bytes() {
    // Update timeout check variables
    timeout_happened = false;
    last_checkpoint = millis();

    double left_wheel_input = Serial.read();
    double right_wheel_input = Serial.read();
    
    bool left_wheel_dir = left_wheel_input > 0;
    bool right_wheel_dir = right_wheel_input > 0;
    double left_wheel_speed = fabs(left_wheel_input);
    double right_wheel_speed = fabs(right_wheel_input);

    wheels.wheelList[0].set_speed = left_wheel_speed;
    wheels.wheelList[0].dir = left_wheel_dir;
    wheels.wheelList[1].set_speed = left_wheel_speed;
    wheels.wheelList[1].dir = left_wheel_dir;
    wheels.wheelList[2].set_speed = left_wheel_speed;
    wheels.wheelList[2].dir = left_wheel_dir;

    wheels.wheelList[3].set_speed = right_wheel_speed;
    wheels.wheelList[3].dir = right_wheel_dir;
    wheels.wheelList[4].set_speed = right_wheel_speed;
    wheels.wheelList[4].dir = right_wheel_dir;
    wheels.wheelList[5].set_speed = right_wheel_speed;
    wheels.wheelList[5].dir = right_wheel_dir;
    
}

// This method flushes whatever is in the input buffer
void flush_input_buffer() {
    while (Serial.available() > 0) { Serial.read(); }
}

// This method checks if it has been longer than MILLISECONDS_FOR_TIMEOUT since the last command. If so, it's understood that connection with the serial port was lost, so the rover is stopped
void timeout_check() {
    if (!timeout_happened) {
        // If the current millis count exceeds the last checkpoint by X amount of time
        if ((millis() - last_checkpoint) >= MILLISECONDS_FOR_TIMEOUT) {
            timeout_happened = true;
            // Stop the rover
            for (int i=0; i < NUM_WHEELS; i++) {
                wheels.wheelList[i].set_speed = STOP_WHEELS;
                wheels.wheelList[i].dir = STOP_WHEELS;
            }
            
            // Write the updates values (updated to zero) to the pins of the wheels
            wheels.writeParams(); // Write the values of the wheels data members onto the arduino pins
            
            // Send feedback
            send_feedback();
        }
    }
}

// This method writes the information of each wheel to the serial port as feedback
void send_feedback() {
    for (int i=0; i < NUM_WHEELS; i++) {
        // Capture variables
        int wheel_speed = wheels.wheelList[i].set_speed;
        int wheel_dir = wheels.wheelList[i].dir;
        // Send variables to the serial port
        Serial.write(wheel_speed);
        Serial.write(wheel_dir);
    }
}

// This method establishes the serial connection
void establish_connection() {
    Serial.begin(9600);
    Serial.println("Connection established");
}

// This method sets the pins for each motor
void set_pin_modes() {
    //wheels
    pinMode(RIGHT_FRONT_WHEEL_SET_SPEED, OUTPUT);
    pinMode(RIGHT_FRONT_WHEEL_DIR, OUTPUT);
    pinMode(RIGHT_FRONT_WHEEL_ENABLE, OUTPUT);
    pinMode(RIGHT_FRONT_WHEEL_ERROR, INPUT_PULLUP);

    pinMode(RIGHT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
    pinMode(RIGHT_MIDDLE_WHEEL_DIR, OUTPUT);
    pinMode(RIGHT_MIDDLE_WHEEL_ENABLE, OUTPUT);
    pinMode(RIGHT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);

    pinMode(RIGHT_REAR_WHEEL_SET_SPEED, OUTPUT);
    pinMode(RIGHT_REAR_WHEEL_DIR, OUTPUT);
    pinMode(RIGHT_REAR_WHEEL_ENABLE, OUTPUT);
    pinMode(RIGHT_REAR_WHEEL_ERROR, INPUT_PULLUP);

    pinMode(LEFT_FRONT_WHEEL_SET_SPEED, OUTPUT);
    pinMode(LEFT_FRONT_WHEEL_DIR, OUTPUT);
    pinMode(LEFT_FRONT_WHEEL_ENABLE, OUTPUT);
    pinMode(LEFT_FRONT_WHEEL_ERROR, INPUT_PULLUP);

    pinMode(LEFT_MIDDLE_WHEEL_SET_SPEED, OUTPUT);
    pinMode(LEFT_MIDDLE_WHEEL_DIR, OUTPUT);
    pinMode(LEFT_MIDDLE_WHEEL_ENABLE, OUTPUT);
    pinMode(LEFT_MIDDLE_WHEEL_ERROR, INPUT_PULLUP);

    pinMode(LEFT_REAR_WHEEL_SET_SPEED, OUTPUT);
    pinMode(LEFT_REAR_WHEEL_DIR, OUTPUT);
    pinMode(LEFT_REAR_WHEEL_ENABLE, OUTPUT);
    pinMode(LEFT_REAR_WHEEL_ERROR, INPUT_PULLUP);  
}
