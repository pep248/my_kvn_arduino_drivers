/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetStepsPerTimeFrame;    // target OutputSpeed in ticks per frame
  long Position;                  // Position count
  long PrevEnc;                  // last Position count

  /*
  * Using previous previousOutputSpeed (PrevInputSpeed) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInputSpeed;                // last previousOutputSpeed
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long OutputSpeed;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Position and PrevEnc the current Position value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetStepsPerTimeFrame = 0.0;
   leftPID.Position = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Position;
   leftPID.OutputSpeed = 0;
   leftPID.PrevInputSpeed = 0;
   leftPID.ITerm = 0;

   rightPID.TargetStepsPerTimeFrame = 0.0;
   rightPID.Position = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Position;
   rightPID.OutputSpeed = 0;
   rightPID.PrevInputSpeed = 0;
   rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long OutputSpeed;
  int previousOutputSpeed;

  //Perror = p->TargetStepsPerTimeFrame - (p->Position - p->PrevEnc);
  previousOutputSpeed = p->Position - p->PrevEnc;
  Perror = p->TargetStepsPerTimeFrame - previousOutputSpeed;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //OutputSpeed = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  OutputSpeed = (Kp * Perror - Kd * (previousOutputSpeed - p->PrevInputSpeed) + p->ITerm) / Ko;
  p->PrevEnc = p->Position;

  OutputSpeed += p->OutputSpeed;
  // Accumulate Integral error *or* Limit OutputSpeed.
  // Stop accumulating when OutputSpeed saturates
  if (OutputSpeed >= MAX_PWM)
    OutputSpeed = MAX_PWM;
  else if (OutputSpeed <= -MAX_PWM)
    OutputSpeed = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  p->ITerm += Ki * Perror;

  p->OutputSpeed = OutputSpeed;
  p->PrevInputSpeed = previousOutputSpeed;
}

/* Read the Position values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Position = readEncoder(LEFT);
  rightPID.Position = readEncoder(RIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInputSpeed is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevInputSpeed != 0 || rightPID.PrevInputSpeed != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.OutputSpeed, rightPID.OutputSpeed);
}

