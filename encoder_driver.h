/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#elif defined KY040_ENC
  #define LEFT_SW 8
  #define LEFT_DT 9
  #define LEFT_CLK 10
  #define RIGHT_SW 11
  #define RIGHT_DT 12
  #define RIGHT_CLK 13
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

