/*  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 * 
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * 
 *  You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OWN_TURTLEBOT_CORE_CONFIG_H_
#define OWN_TURTLEBOT_CORE_CONFIG_H_

// enable this line for Arduino Due
#define ARDUINO_DUE                     // ** COMMENT THIS LINE IF YOUR NOT USING A DUE **

#ifdef ARDUINO_DUE
#define USE_USBCON
#endif

/* Include librairies */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <Timer.h>
#include <MedianFilter.h>

/* Global parameters */
#define FREQUENCY_RATE 						30 			// [ms]
#define FREQUENCY_ROSPINONCE 				150 		// [ms]
#define FREQUENCY_CONTROLLER 				30 			// [ms]

/* Rate computing parameters */
#define RATE_DIRECTION_MEDIAN_FILTER_SIZE 	3
#define RATE_CONV 							0.0073882 	// [inc] -> [rad] 
#define RATE_AVERAGE_FILTER_SIZE 			4

/* Rate controller parameters */
#ifdef ARDUINO_DUE
#define PWM_MAX				 				4095
#define RATE_CONTROLLER_MIN_PWM 			-500
#define RATE_CONTROLLER_MAX_PWM 			500
#define RATE_INTEGRAL_FREEZE                250
#else
#define PWM_MAX				 				255
#define RATE_CONTROLLER_MIN_PWM 			-40
#define RATE_CONTROLLER_MAX_PWM 			40
#define RATE_INTEGRAL_FREEZE                15
#endif
#define RATE_CONTROLLER_KP 					130.0               // ** TUNE THIS VALUE **
#define RATE_CONTROLLER_KD 					5000000000000.0     // ** TUNE THIS VALUE **
#define RATE_CONTROLLER_KI 					0.00005             // ** TUNE THIS VALUE **

/* Mechanical parameters */
#define WHEEL_RADIUS 						0.0325 		// [m]
#define BASE_LENGTH							0.3 		// [m]

/* Define frequency loops */
Timer _frequency_rate(FREQUENCY_RATE);
Timer _frequency_rospinonce(FREQUENCY_ROSPINONCE);
Timer _frequency_controller(FREQUENCY_CONTROLLER);

/* Define median filter for direction */
MedianFilter motor_right_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);
MedianFilter motor_left_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);

/* Define pins */
// motor A (right)
const byte motorRightEncoderPinA = 38;      // ** MODIFY WITH YOUR PIN NB **
const byte motorRightEncoderPinB = 34;      // ** MODIFY WITH YOUR PIN NB **
const byte enMotorRight = 2;                // ** MODIFY WITH YOUR PIN NB **
const byte in1MotorRight = 4;               // ** MODIFY WITH YOUR PIN NB **
const byte in2MotorRight = 3;               // ** MODIFY WITH YOUR PIN NB **
// motor B (left)
const byte motorLeftEncoderPinA = 26;       // ** MODIFY WITH YOUR PIN NB **
const byte motorLeftEncoderPinB = 30;       // ** MODIFY WITH YOUR PIN NB **
const byte enMotorLeft = 7;                 // ** MODIFY WITH YOUR PIN NB **
const byte in1MotorLeft = 6;                // ** MODIFY WITH YOUR PIN NB **
const byte in2MotorLeft = 5;                // ** MODIFY WITH YOUR PIN NB **

/* Define motors variables */
// right
volatile int motor_right_inc;
int motor_right_direction;
int motor_right_filtered_direction;
float motor_right_filtered_inc_per_second;
float motor_right_rate_est;
float motor_right_rate_ref;
int motor_right_check_dir;
int motor_right_pwm_rate;
unsigned long motor_right_prev_time;
int pwmMotorRight = 0;
// left
volatile int motor_left_inc;
int motor_left_direction;
int motor_left_filtered_direction;
float motor_left_filtered_inc_per_second;
float motor_left_rate_est;
float motor_left_rate_ref;
int motor_left_check_dir;
int motor_left_pwm_rate;
unsigned long motor_left_prev_time;
int pwmMotorLeft = 0;

/* Define controlers variables */
// right
unsigned long controler_motor_right_prev_time;
float controler_motor_right_prev_epsilon = 0.0;
float controler_motor_right_int = 0.0;
// left
unsigned long controler_motor_left_prev_time;
float controler_motor_left_prev_epsilon = 0.0;
float controler_motor_left_int = 0.0;

/* Mixer variable */
float linear_velocity_ref;
float angular_velocity_ref;

/* ROS Nodehanlde */
ros::NodeHandle nh;

/* Prototype function */
void rateControler(const float rate_ref, const float rate_est, int & pwm_rate, unsigned long & prev_time, float & previous_epsilon, float & integral_epsilon);
float runningAverage(float prev_avg, const float val, const int n);
void setMotorRateAndDirection(int pwm_ref, const float rate_ref, const byte enMotor, const byte in1Motor, const byte in2Motor);

/* Cmd_vel subscriber */
// callback function prototype
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
// message
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", commandVelocityCallback);

#endif
