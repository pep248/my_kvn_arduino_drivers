/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 2
  #define LEFT_MOTOR_BACKWARD  3
  #define RIGHT_MOTOR_FORWARD  4
  #define LEFT_MOTOR_FORWARD   5
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#elif defined L9110_MOTOR_DRIVER
  #define RIGHT_MOTOR_FORWARD 4
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_FORWARD 2
  #define LEFT_MOTOR_BACKWARD 3

  #define LEFT 1
  #define RIGHT 0
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

