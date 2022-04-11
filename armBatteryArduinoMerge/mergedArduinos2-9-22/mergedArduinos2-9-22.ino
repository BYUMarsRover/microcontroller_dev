#ifndef ARM_ARDUINO_INO
#define ARM_ARDUINO_INO

//#include <ros.h>
//#include <rover_msgs/bat_stat_grip_elev_arduino.h>
#include <Tic.h>
#define FASTLED_INTERNAL
#include <FastLED.h>
#include <ezButton.h>
#define TOP_LIMIT_SWITCH A2


TicI2C tic;
ezButton limitSwitch(A2);  // create ezButton object that attach to pin A2;


// Elevator and Gripper
#define FLT 2  // D2  fault
#define INA 3   // D3
#define INB 4  // D4
#define PWM 5   // D5

// Keyboard Clicker
#define LASR 6 // D6
#define CLIKR 9 // D9

// Battery
#define BATTERY_MONITOR_INPUT_PIN A0 //A0
//#define BATTERY_MONITOR_BATTERY_ADC_MIN 729
#define BATTERY_MONITOR_PREAMBLE 0x2


// Indicator
#define STATUS_INDICATOR_LED_PIN 8 // D8
#define STATUS_INDICATOR_NUM_LEDS   32
#define STATUS_INDICATOR_BRIGHTNESS  25
#define STATUS_INDICATOR_PREAMBLE  0x1
#define STATUS_INDICATOR_COUNTER_MAX 80000

//void control_Callback(const rover_msgs::bat_stat_grip_elev_arduino& control_msg);

int battCounter = 0;
long statCounter = 0;
bool engage = false;
bool hit_top_elevator = false;
//ros::NodeHandle_<ArduinoHardware, 2, 1, 160, 210> node_handle;
//rover_msgs::bat_stat_grip_elev_arduino control_msg;
//ros::Subscriber<rover_msgs::bat_stat_grip_elev_arduino> control_subscriber("stat_grip_elev_arduino_cmd", &control_Callback);
// Define the array of leds
CRGB leds[STATUS_INDICATOR_NUM_LEDS];
enum led_mode_states {AUTONOMOUS, TELEOPERATION, ARRIVAL, IDLE};
led_mode_states led_mode;



//
//void control_Callback(const rover_msgs::bat_stat_grip_elev_arduino& control_msg) {
//  
//  if (control_msg.navigation_state != 1) {
//    led_mode = static_cast<led_mode_states>(control_msg.navigation_state);
//  }
//
//  if (control_msg.gripper == 0) {
//
//    analogWrite(PWM, 0);
//  } else if (control_msg.gripper > 0) {
//
//    digitalWrite(INA, HIGH);
//    digitalWrite(INB, LOW);
//    analogWrite(PWM, control_msg.gripper);
//  } else if (control_msg.gripper < 0) {
//
//    digitalWrite(INA, LOW);
//    digitalWrite(INB, HIGH);
//    analogWrite(PWM, -255 - control_msg.gripper);
//  }
//  
//  if (engage == true) {
//
//    if (control_msg.elevator == 12) {
//      
//      tic.deenergize();
//      engage = false;
//    } else {
//      
//      tic.setTargetVelocity(control_msg.elevator * 2000000);
//    }
//  } else if ((engage == false) && (control_msg.elevator != 1)) {
//
//    engage = true;
//    // Set up I2C.
//    Wire.begin();
//    // Give the Tic some time to start up.
//    delay(20);
//    tic.exitSafeStart();
//    tic.setTargetVelocity(0);
//    tic.energize();
//  }
//}

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

//// Arduino specific - called when data is available
//void serialEvent()
//{
//  // Indicator Code
//  readSerialData();

//}

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
  limitSwitch.setDebounceTime(50); 

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

  // LASER DIODE AND SOLENOID ENABLE PINS
  pinMode(LASR, OUTPUT);
  pinMode(CLIKR, OUTPUT);
  // !! Set pin states !!
  
  // Sets ENABLE to enable driver outputs
  digitalWrite(INA, LOW);

  // Sets DIRECTION to forward
  digitalWrite(INB, HIGH);

  // Sets PULSE WIDTH MODULATION to 0% duty cycle
  analogWrite(PWM, 0);
  
  // Battery Setup //

  pinMode(BATTERY_MONITOR_INPUT_PIN, INPUT);

  // Laser Default State
  digitalWrite(LASR, LOW);
  digitalWrite(CLIKR, LOW);


  // Inidicator LED setup //

  FastLED.addLeds<WS2812B, STATUS_INDICATOR_LED_PIN, GRB>(leds, STATUS_INDICATOR_NUM_LEDS);
  
  FastLED.setBrightness(STATUS_INDICATOR_BRIGHTNESS);
  
  setArrayColor(255, 255, 255);    // Flash white for ok signal

//  delay(2000);
  
  led_mode = IDLE;

  Serial.flush();
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
      battCounter = 0;
      int reading = analogRead(BATTERY_MONITOR_INPUT_PIN);
//      String asciiReading = String(reading);

      // You can't write to serial correctly without flushing the buffer
      Serial.flush();

      Serial.println(reading);
      Serial.flush();
  }
  
char data_array[128];
int stored_data_index = 0;

void decoder()
{
  /////////////////////
  // BUFFER EXTRACTION/READING

  // reply only when you receive data:
  if (Serial.available() > 0) {
    // setup for loop where we put serial buffer bytes into local RAM buffer
    int how_many_bytes = Serial.available();
    bool has_message = false;
    int end_of_message_index = -1;
    
    // each iteration of the loop gets one character
    for (int i = 0; i < how_many_bytes; i++) {
      char read_char = Serial.read();
      // FOR DEBUGGING ONLY, PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
        //Serial.println("read char:");
        //Serial.println(read_char);
      // we defined an interface where a semicolon represents the end of the message, so we look for semicolons
      
      if (!isspace(read_char)) {
        data_array[stored_data_index] = read_char;
      }
      stored_data_index++;

      
      
      if (read_char == ';') {
        // Serial.println("Message complete, setting has_message to true"); 
        has_message = true;
        // we keep track of the end of message,
        end_of_message_index = stored_data_index - 1;
        
        // we break to make sure that we don't store two valid commands. And this helps us to make parsing easier and prevents some edge cases that mess us up 
        // (if you understand buffers, then you can change this, otherwise, don't)
        break;
      }
    }

    /////////////////////////
    // PARSING
    //if we have a message, then we continue with parsing, otherwise, this function is done

    // the format is G [gripper field] : [elevator field] ;
    if (has_message) {

      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%PROBLEM IS HERE: Data_array index isn't getting reset properly, so it's never entering this branch after the first time. 
      //SJC Feb 11
      
      //Serial.println(data_array[0]);
      if(data_array[0] == 'G') {
        // Serial.println("Detected message is of type G");
        // this parses the gripper_elev commands
        // first, we find the index of the colon delimiter
        char *delimiter_index = data_array;
        char *command_index = data_array;
        while (*command_index == 'G') {
          command_index++;
        }
        
        while(*delimiter_index != ':') {
          delimiter_index++;
        }
        //Serial.println(data_array);
        // we found the index of the colon delimiter
        *delimiter_index = '\0';
        // now the delimiter points to the start of the elevator field
        delimiter_index++;
        
        data_array[end_of_message_index] = '\0';
        char *gripper = command_index;
        // Serial.println(command_index);
        //now gripper will point to the start of the integer field (ignoring the 'G' character)
        //gripper++;
        int gripper_int = atoi(gripper);
        int elevator_int = atoi(delimiter_index);
        // FOR DEBUGGING ONLY, PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
         // Serial.println(gripper_int);
         //Serial.println(elevator_int);

        ///////////////////
        // HAVING PARSED, WE NOW DO THE FUNCITONALITY
        if (gripper_int == 0) {
          analogWrite(PWM, 0);
        } else if (gripper_int > 0) {
          //Serial.println(gripper_int);
          digitalWrite(INA, HIGH); // D3 
          digitalWrite(INB, LOW);  // D4
          analogWrite(PWM, abs(gripper_int));
        } else if (gripper_int < 0) {
          // Reversed LOW/HIGH is brokey
          digitalWrite(INA, LOW); // 
          digitalWrite(INB, HIGH); // 
//          digitalWrite(INA, HIGH);
//          digitalWrite(INB, LOW);
          //Serial.println(gripper_int);
          //analogWrite(PWM, gripper_int);
        analogWrite(PWM, -254 - gripper_int);
        }
        
//        Serial.println(hit_top_elevator);
        
        if (engage == true) {
          if (elevator_int == 12) {
            
            tic.deenergize();
            engage = false;
          } else {
            if (!hit_top_elevator) { // if it has not hit the top
              tic.setTargetVelocity(elevator_int * 2000000); // keep going up
            } else {
              if (elevator_int > 0) { // if it is commanded to go down
                tic.setTargetVelocity(elevator_int * 2000000); // set the velocity 
                delay(1000); //and let it go until it releases the button
              }
            }
          }
          
        } else if ((engage == false) && (elevator_int != 1)) {
      
          engage = true;
          // Set up I2C.
          Wire.begin();
          // Give the Tic some time to start up.
          delay(20);
          tic.exitSafeStart();
          tic.setTargetVelocity(0);
          tic.energize();
        }


      } else if(data_array[0] == 'L') {
        // Serial.println("Detected message is of type L");
        ////////////////////
        // PARSING
        // this parses a LED command
        
        data_array[end_of_message_index] = '\0';
        // FOR DEBUGGING ONLY, PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
//        Serial.println("data array:");
//        Serial.println(data_array);
        char* nav_field = data_array;
        nav_field += end_of_message_index - 1;
        int navigation_state = atoi(nav_field);
//        Serial.println(navigation_state);
        ///////////////////
        // HAVING PARSED, THE FOLLOWING IS FUNCTIONALITY
//        if (navigation_state != 1) {
        led_mode = static_cast<led_mode_states>(navigation_state);
//        }
        // FOR DEBUGGING ONLY, PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
//        Serial.println("navigation state:");
//        Serial.println(navigation_state);
         
      } else if(data_array[0] == 'S'){
        // Serial.println("Detected message is of type S");
          // PARSING
          // this parses a Solenoid command
          data_array[end_of_message_index] = '\0';
          if(data_array[1] == '+'){
            digitalWrite(LASR,HIGH);
          }
          if(data_array[1] == '-'){
            digitalWrite(LASR,LOW);
          }
          if(data_array[1] == '!'){
            digitalWrite(CLIKR,HIGH);
            delay(110); // good for one key click
            digitalWrite(CLIKR,LOW);
          }
          
      } else {
        // you done messed up A-Aron, we erase the buffer
        // FOR DEBUGGING ONLY, PRINT STATEMENTS ARE INTERPRETED AS BATTERY VOLTAGE READINGS
        // println("invalid command detected");
        stored_data_index = 0;
      } 
      //Serial.println("Command procedures passed, resetting buffer");
      // we have parsed our message, now we need to reset the buffer
      stored_data_index = 0;
      while(Serial.available()){
        Serial.read();
      }
      // // Serial.print("This is what is left: ");
      // Serial.println(data_array);
      // Serial.print("left in the serial:");
      // Serial.println(Serial.read());
      Serial.flush();
      
    }
  }
}


void limitswitch(){
   // Limit switch
  limitSwitch.loop(); // MUST call the loop() function first
  if(limitSwitch.isPressed()) {
//      Serial.println("The limit switch: UNTOUCHED -> TOUCHED");
      hit_top_elevator = true;
      tic.setTargetVelocity(0);
  }

  if(limitSwitch.isReleased()) {
//      Serial.println("The limit switch: TOUCHED -> UNTOUCHED");
      hit_top_elevator = false;
      
  }

  int state = limitSwitch.getState();
  if(state == HIGH) {
//      Serial.println("The limit switch: UNTOUCHED");
      hit_top_elevator = false;
  }
  else{
//        Serial.println("The limit switch: TOUCHED");
        hit_top_elevator = true;
        tic.setTargetVelocity(0);
    }
}

void loop()
{
  limitswitch();
  // Indicator LED
  decoder();
  statusIndicatorTick();
  // Battery 
  batteryTick();
  // Arm and Elevator
//  node_handle.spinOnce();
//  delay(1);
}

#endif 
