#ifndef ARM_ARDUINO_INO
#define ARM_ARDUINO_INO

#include <ros.h>
#include <rover_msgs/GripperElevatorControl.h>
#include <Tic.h>
#define FASTLED_INTERNAL
#include <FastLED.h>


TicI2C tic;

// Elevator and Gripper
#define FLT 2  // D2  fault
#define INA 3   // D3
#define INB 4  // D4
#define PWM 5   // D5

// Battery
#define BATTERY_MONITOR_INPUT_PIN 19 //A0
#define BATTERY_MONITOR_BATTERY_ADC_MIN 729
#define BATTERY_MONITOR_PREAMBLE 0x2


// Indicator
#define STATUS_INDICATOR_LED_PIN     11 // D8
#define STATUS_INDICATOR_NUM_LEDS   32
#define STATUS_INDICATOR_BRIGHTNESS  25
#define STATUS_INDICATOR_PREAMBLE  0x1
#define STATUS_INDICATOR_COUNTER_MAX 250

void control_Callback(const rover_msgs::bat_stat_grip_elev_arduino& control_msg);

int battCounter = 0;
int statCounter = 0;
bool engage = false;
ros::NodeHandle_<ArduinoHardware, 2, 1, 160, 210> node_handle;
rover_msgs::GripperElevatorControl control_msg;
ros::Subscriber<rover_msgs::bat_stat_grip_elev_arduino> control_subscriber("stat_grip_elev_arduino_cmd", &control_Callback);
// Define the array of leds
CRGB leds[STATUS_INDICATOR_NUM_LEDS];
enum led_mode_states {AUTONOMOUS, ARRIVAL, TELEOPERATION, IDLE};
led_mode_states led_mode;


void control_Callback(const rover_msgs::bat_stat_grip_elev_arduino& control_msg) {

  if (control_msg.navigation_state != 1) {
    led_mode = static_cast<led_mode_states>(control_msg.navigation_state);
  }

  if (control_msg.gripper == 0) {

    analogWrite(PWM, 0);
  } else if (control_msg.gripper > 0) {

    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
    analogWrite(PWM, control_msg.gripper);
  } else if (control_msg.gripper < 0) {

    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
    analogWrite(PWM, -255 - control_msg.gripper);
  }
  
  if (engage == true) {

    if (control_msg.elevator == 12) {
      
      tic.deenergize();
      engage = false;
    } else {
      
      tic.setTargetVelocity(control_msg.elevator * 2000000);
    }
  } else if ((engage == false) && (control_msg.elevator != 1)) {

    engage = true;
    // Set up I2C.
    Wire.begin();
    // Give the Tic some time to start up.
    delay(20);
    tic.exitSafeStart();
    tic.setTargetVelocity(0);
    tic.energize();
  }
}

// Sends a "Reset command timeout" command to the Tic.  We must
// call this at least once per second, or else a command timeout
// error will happen.  The Tic's default command timeout period
// is 1000 ms, but it can be changed or disabled in the Tic
// Control Center.
void resetCommandTimeout()
{
  tic.resetCommandTimeout();
}

// Delays for the specified number of milliseconds while
// resetting the Tic's command timeout so that its movement does
// not get interrupted by errors.
void delayWhileResettingCommandTimeout(uint32_t ms)
{
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}

// Arduino specific - called when data is available
void serialEvent()
{
  // Indicator Code
  readSerialData();

}

// //TODO: REPLACE WITH ROSTOPICS
// void readSerialData()
//     {
//         unsigned int data_array[2];
//         int index = 0;
//         while(Serial.available())    // slave may send less than requested
//         {
//             unsigned int c = Serial.read();    // receive a byte as character
//             if (index >=2){
//             index = 1;
//             }
//             data_array[index++] = c;
//         }
//         if (index == 2)
//         {
//             led_mode = static_cast<led_mode_states>(data_array[1]);
//         }
//     }

void setArrayColor(char red, char green, char blue)
    {
        for (int x=0;x<STATUS_INDICATOR_NUM_LEDS;x++)
        {
            leds[x] = CRGB(red,green,blue);
        }
        FastLED.show();
    }

void setup()
{
  // Arduino_rover_status setup
  Serial.begin(9600);
  Serial.println("starting");
  // indicator = StatusIndicator();
  // indicator.setup();
  // battery_monitor = BatteryMonitor();
  // battery_monitor.setup();


  // !! Set pin functionality

  // Gripper and Elevator setup //

  // FAULT - LOW: Overcurrent condition or thermal shutdown
  //         HIGH: Normal Operation
  pinMode(FLT, INPUT);

  // ENABLE - LOW: Enable driver outputs
  //          HIGH: Enable Tri-State ??????????????
  pinMode(INA, OUTPUT);

  // DIRECTION - LOW: Reverse
  //             HIGH: Forward
  pinMode(INB, OUTPUT);

  // PULSE WIDTH MODULATION - 0: 0% Duty Cycle
  //                          255: 100% Duty Cycle
  pinMode(PWM, OUTPUT);

  // !! Set pin states !!
  
  // Sets ENABLE to enable driver outputs
  digitalWrite(INA, LOW);

  // Sets DIRECTION to forward
  digitalWrite(INB, HIGH);

  // Sets PULSE WIDTH MODULATION to 0% duty cycle
  analogWrite(PWM, 0);
  
  // Battery Setup //

  pinMode(BATTERY_MONITOR_INPUT_PIN, INPUT);

  // Inidicator LED setup //

  FastLED.addLeds<WS2812B, STATUS_INDICATOR_LED_PIN, GRB>(leds, STATUS_INDICATOR_NUM_LEDS);
  FastLED.setBrightness(STATUS_INDICATOR_BRIGHTNESS);

  setArrayColor(255, 255, 255);    // Flash white for ok signal
  delay(1000);
  led_mode = IDLE;

  node_handle.initNode();
  node_handle.subscribe(control_subscriber);
}

void statusIndicatorTick()
    {
        if (led_mode == AUTONOMOUS)
            setArrayColor(255,0,0);
        else if (led_mode == ARRIVAL)
        {
            if (statCounter == STATUS_INDICATOR_COUNTER_MAX/2)
                // Need to flash if in arrival state
                setArrayColor(0,255,0);
            else if (statCounter == STATUS_INDICATOR_COUNTER_MAX){
                setArrayColor(0,0,0);
                statCounter = 0;
            }
            statCounter++;
        }
        else if (led_mode == TELEOPERATION)
            setArrayColor(0,0,255);
        else
            setArrayColor(0,0,0);
    }

void batteryTick()
  {
      if (battCounter++ < 1000)
            return;
      int reading = analogRead(BATTERY_MONITOR_INPUT_PIN);

      char low_byte = reading;
      char high_byte = reading >> 8;

      // You can't write to serial correctly without flushing the buffer
      Serial.flush();

      // Data array padded on both ends with 0's to prevent data from coming in out of order.
      // Preamble is at 1, low byte is at 2, high byte is at 3. 0's at indexes 0 and 4.
      char buf[5];
      buf[0] = 0; 
      buf[1] = BATTERY_MONITOR_PREAMBLE;
      buf[2] = low_byte;
      buf[3] = high_byte;
      buf[4] = 0;
        
      Serial.write(buf, 5);
      Serial.flush();
  }

void loop()
{
  // Indicator LED
  statusIndicatorTick();
  // Battery 
  batteryTick();
  // Arm and Elevator
  node_handle.spinOnce();
  delay(1);
}










#endif 