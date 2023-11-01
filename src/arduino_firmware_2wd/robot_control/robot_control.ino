//Name: robot_firmware

//Author: Lentin Joseph

//ROS Arduino code publishing sensor data and subscribing to motor commands 


#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>


/////////////////////////////////////////////////////////////////
//Motor, Encoder and IR sensor pin definition

int encoder_pinA = 2;    
int encoder_pinB = 3;
                             
volatile int pulses1 = 0;  
volatile int pulses2 = 0;      

#define IR_PIN A0

//Motor A
int enableA = 5;
int MotorA1 = 6;
int MotorA2 = 7;
 
//Motor B
int enableB = 11;
int MotorB1 = 10;
int MotorB2 = 9;

/////////////////////////////////////////////////////////////////////////////////////

//ROS Node handle
ros::NodeHandle  nh;
//Left and right speed
int r_speed = 0, l_speed = 0;

//Direction flag for encoder
int left_direction = 1;
int right_direction = 1;


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief callback to set the left speed of the robot
 * 
 * @param msg(std_msgs::Int32) - message containing the left speed
 * 
 * @return void
*/
void left_speed_cb(const std_msgs::Int32& msg)  // cmd_vel callback function definition
{

   digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
   l_speed = msg.data;

}

/**
 * @brief callback to set the right speed of the robot
 * 
 * @param msg(std_msgs::Int32) - message containing the right speed
 * 
 * @return void
 */

void right_speed_cb(const std_msgs::Int32& msg)  // cmd_vel callback function definition
{
   digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
   r_speed = msg.data;

}

/**
 * @brief callback to reset the speed of the robot. This also resets the count of the encoders
 * 
 * @param msg(std_msgs::Bool) - message containing the reset flag
 * 
 * @return void
 */
void reset_cb(const std_msgs::Bool& msg)
{

    l_speed = 0;
    r_speed = 0;

    pulses1 = 0;
    pulses2 = 0;

  
}

////////////////////////////////////////////////////////////////////////////////////////////////
//Mapping function one range to another range

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


////////////////////////////////////////////////////////////////////////////////////////////////
//Publisher for Left and Right encoder

std_msgs::Int32 l_encoder_msg;
ros::Publisher l_enc_pub("left_ticks", &l_encoder_msg);

std_msgs::Int32 r_encoder_msg;
ros::Publisher r_enc_pub("right_ticks", &r_encoder_msg);

//Sharp distance publisher

std_msgs::Float32 sharp_msg;
ros::Publisher sharp_distance_pub("obstacle_distance", &sharp_msg);

//Subscribers for left and right speed

ros::Subscriber<std_msgs::Int32> left_speed_sub("set_left_speed",&left_speed_cb);  // creation of subscriber object sub for recieving the cmd_vel
ros::Subscriber<std_msgs::Int32> right_speed_sub("set_right_speed",&right_speed_cb);  // creation of subscriber object sub for recieving the cmd_vel
ros::Subscriber<std_msgs::Bool> reset_sub("reset",&reset_cb);  // creation of subscriber object sub for recieving the cmd_vel



////////////////////////////////////////////////////////////////////////////////////////////////


// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        

// Loop frequency: 100 ms
const long interval = 50;           


//////////////////////////////////////////////////////////////////////////////////////////////////////
//ISR for two encoders
/**
 * @brief callback funcion to count the pulses from the left wheel encoder
 * 
 * @return void
 * 
 */
void counter1(){
 
        pulses1 = pulses1 + left_direction;    
     
}
/**
 * @brief callback funcion to count the pulses from the right wheel encoder
 * 
 * @return void
 * 
 */
void counter2(){
 
        pulses2 = pulses2 + right_direction;    
     
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setting encoder pins as interrupts
/**
 * @brief Set the up wheelencoders using interrupts
 * 
 * @return void
 */
void setup_wheelencoder()
{
 
   pinMode(encoder_pinA, INPUT);
   attachInterrupt(digitalPinToInterrupt (encoder_pinA), counter1, RISING);
   pulses1 = 0;
   pinMode(encoder_pinB, INPUT);
   attachInterrupt(digitalPinToInterrupt (encoder_pinB), counter2, RISING);
   pulses2 = 0;
   
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * @brief Read IR values and publish
 * 
 * @return void
 * 
 */

//NB: In this particular project, an Ultrasonic sensor is used hence this may be commented out

void update_IR()
{
 
  float volts = analogRead(IR_PIN)*0.0048828125;  // value from sensor * (5/1024)
  float distance = 13*pow(volts, -1); // worked out from datasheet graph

  sharp_msg.data = distance;
  sharp_distance_pub.publish(&sharp_msg);
 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief - updates the motor speed and direction
 * 
 * @return void
 * 
*/
void update_Motor()
{

  //If left speed is greater than zero
  if(l_speed >= 0)
  {
    digitalWrite (MotorA1, LOW);
    digitalWrite (MotorA2, HIGH);

    analogWrite(enableA,abs(l_speed));  

    left_direction = 1;
    
  }
  else
  {

    digitalWrite (MotorA1, HIGH);
    digitalWrite (MotorA2, LOW);

    analogWrite(enableA,abs(l_speed));    

    left_direction = -1;

    
  }

  if(r_speed >= 0)
  {

    digitalWrite (MotorB1, HIGH);
    digitalWrite (MotorB2, LOW);
    analogWrite(enableB,abs(r_speed));    

    right_direction = 1;

     
  }
  else
  {

    digitalWrite (MotorB1, LOW);
    digitalWrite (MotorB2, HIGH);
    analogWrite(enableB,abs(r_speed));    

    right_direction = -1;

   
  }
  

  
}

/**
 * @brief - sets up necessary properties for the program
 * @details - sets up serial communication, motor pins, encoder pins, ROS node, ROS publisher and ROS subscriber
 * 
 *
 * @return - void
 * 
 */
void setup()
{
  //Setting Serial1 and bluetooth as default serial port for communication via Bluetooth
  
  nh.getHardware()->setPort(&Serial);
  nh.getHardware()->setBaud(9600);

  // Setup motor pins
  pinMode (enableA, OUTPUT);
  pinMode (MotorA1, OUTPUT);
  pinMode (MotorA2, OUTPUT);  
   
  pinMode (enableB, OUTPUT);
  pinMode (MotorB1, OUTPUT);
  pinMode (MotorB2, OUTPUT); 
  
  // setup inbuilt led
  pinMode(LED_BUILTIN, OUTPUT);  

  //Setup wheel encoders
  setup_wheelencoder();

  //Initialize ROS node
  nh.initNode();

  //Setup publisher
  nh.advertise(l_enc_pub);
  nh.advertise(r_enc_pub);
  nh.advertise(sharp_distance_pub);

  //Setup subscriber
  nh.subscribe(left_speed_sub);
  nh.subscribe(right_speed_sub);
  nh.subscribe(reset_sub);
  
}

/**
 * @brief - main loop of the program. sends commands to control the motors
 * 
 * @return - void
 * 
 */
void loop()
{

  unsigned long currentMillis = millis();

  // Publish encoder data at 100 Hz
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;

    l_encoder_msg.data = pulses1;
    r_encoder_msg.data = pulses2;

    l_enc_pub.publish(&l_encoder_msg);
    r_enc_pub.publish(&r_encoder_msg);

    update_IR();
  
  }

  update_Motor();
  nh.spinOnce();

  delay(20);
}
