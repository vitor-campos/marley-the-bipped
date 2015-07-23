// ***********************************************************
// ** TED (Totally Easy Design) the BiPed - BASE Code
// ** v1.00 - 12/2012 - Stephen W Nolen
// LetsMakeRobots.com TED Page - http://letsmakerobots.com/node/35632
// ***********************************************************

// ** This is the base walking code for TED the BiPed
// ** !!!!!!!!!!!! NOTE NOTE  NOTE NOTE  NOTE NOTE  NOTE NOTE  NOTE NOTE  NOTE NOTE  NOTE NOTE  !!!!!!!!!!!!!!!!

// ** You will need to tweak the values in the NormalWalk() sub to fit your servo center points and end limits

// ** !!!!!!!!!!!! NOTE NOTE  NOTE NOTE  NOTE NOTE  NOTE NOTE  NOTE NOTE  NOTE NOTE  NOTE NOTE  !!!!!!!!!!!!!!!!

// ** The basic concept is a "delay free" walking / avoiding solution using timers and incremental changes for walking options
// ** The main loop determines what walking mode we should be in and then calls the sub-routine(s) over and over to complete the walking action
// ** Walking "modes" can change on the fly and the sub routines will start executing that walking request


// ** Includes area - Add what else you need here
// ** This assums you are using an UltraSonic sensor and the NewPing library
// ** the Serial library isn't actually used here but can be to feed data to the console on a true Arduino or an LCD display for debugging
#include <Servo.h>
#include <NewPing.h>

// ** Setup Leg servos - only 2 DOF so Hips and Ankles only
// ** Once finalized some of these could be defines I guess but the "Limits" are handy to change on the fly for short, long, gimp, etc
Servo LeftHip;
Servo LeftAnkle;

Servo RightHip;
Servo RightAnkle;

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ** YOU WILL NEED TO CHANGE THE Center/In/Left/Right values in the NormalWalk() sub at the bottom of the code to match YOUR Configuration and servos
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
byte LeftHipPin          = 9;           // Define the servo pins and the In / Out / Center limits
  byte LeftHipCenterLimit;              // This way you can actually change the distance / settings on the fly via code for "Shortwalk", "LongWalk", etc
  byte LeftHipInLimit;                  // See the NormalWalk() and ShortWalk() subs at the bottom of the code
  byte LeftHipOutLimit;                 // This just makes it much easier to change things although it makes it a bit more complicated
  byte LeftHipCurrent;                  // This tracks the CURRENT servo position so we know what we need to do to move through sub-steps - could use the .read() function but this was easier at the time
  float LeftHipInc;                     // This variable is used by the sub-step routine that moves the servos from current to requested position as it is called over and over
                                        // These are floats as we are dividing how far we need to go by how many possible "substeps" we have to get there
                                        // Otherwise we loose resolution and do not complete the move from my personal experience... :-/

byte RightHipPin         = A4;          // This is on an analog to keep wiring shorter based on the Baby Orangutan pin layouts
  byte RightHipCenterLimit;
  byte RightHipInLimit;
  byte RightHipOutLimit;
  byte RightHipCurrent;
  float RightHipInc;

byte LeftAnklePin        = 11;
  byte LeftAnkleCenterLimit;
  byte LeftAnkleLeftLimit;
  byte LeftAnkleRightLimit;
  byte LeftAnkleCurrent;
  float LeftAnkleInc;

byte RightAnklePin        = A3;
  byte RightAnkleCenterLimit;
  byte RightAnkleLeftLimit;
  byte RightAnkleRightLimit;
  byte RightAnkleCurrent;
  float RightAnkleInc;

byte WalkMode         = 0;		// Set by the walking subs for the CURRENT walking status - this is set by the sub routines
byte WalkRequest      = 0;              // used in the main loop, this is set by sensor, etc and then processed at the end of the main loop

byte WalkModeStop     = 0;		// These are the walking status values so we do not have to remember numbers
  byte WalkModeForward  = 1;
  byte WalkModeBackwards= 2;
  byte WalkModeLeft     = 3;
  byte WalkModeRight    = 4;
  byte OldWalkMode;                     // where we save the last walk mode to try to recover it if we want to use it

byte RobotMode;				// Used to determine what the robot should be doing - i.e. roaming, seeking/avoiding light, waiting for someone, etc
  byte StopMode         = 0;		// just sit there for now watching the sensors
  byte RoamMode         = 1;		// roam mode - just walk around avoiding things
  //byte LightSeekMode    = 2;		// find the light using (future) LDRs
  //byte LightAvoidMode   = 3;		// avoid the light using (future) LDRs
  byte WaitMode         = 4;		// just wait for (future) PIR trigger


long TurnDirection;                     // from random, determines which was to turn when we see something
long WalkTimer;				// walk timer is used by walking routines to test which step we should be taking
int  WalkDelay = 200;			// walk delay is used by walktimer to wait to take the next "step" in the stepping process
                                        // You can adjust this as desired for a quicker pace but mini-bipeds are kind of tricky with smaller numbers

long SubStepTimer;                      // used to track what substep we are in within the main step/frame
int  SubSteps;                          // this is how many increments we are going to move a servo for each walk step or "frame"
                                        // it is calculated based on the LoopDelay below (how many times we call the walk routing) and
                                        // the "walkdelay" (how much time betwen each step or frame).
                                        // This is the number of times we get to move the servo so longer moves move faster, shorter ones slow
                                        // Not the best solution but it should work. We calulcate this in the SETUP sub

long AvoidTimer        = 0;      	// timer needed to determine how long we should be in avoid mode, start it up at zero
long AvoidTimerWalk    = 0;             // timer to use for waking stats during avoid

int LoopDelay          = 20;  		// this determines how much we delay the main loop - 50ms is about right

// ** Sonar setup and information
// ** NewPing variables settings#define TRIGGER_PIN  0  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define TRIGGER_PIN  3  //
#define ECHO_PIN     4  // Arduino pin tied to echo pin on the ultrasonic sensor - change as needed - these are just what I have setup on a Baby Orangutan
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int uS;                                 // sonar raw read
int uScm;                               // sonar CM value - may be difference for Orangutan?
int SonarHitRange      = 10;            // ** Make this smaller if you want to get closer to something before avoiding

// *******************************************************************************************************************************************
// ** Setup stuff
void setup()
{
  randomSeed(analogRead(0));            // we do not have a hard sensor here (open button) so randomize off it before making input

  // ** Ideally you would want to calculate your substeps based on the main walk delay you are using divided by the overall loop delay
  SubSteps = (WalkDelay / LoopDelay);  // this is how many chances we get to move a servo between each step or frame

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  SubSteps = 5;                         // You can OVERRIDE the substeps here and just pick a number, the higher the number the SLOWER the walk
  
  NormalWalk();                         // HAVE to set the leg limits before initializing them

  // *** Attach and center our legs 
  LeftHip.attach(LeftHipPin);		// config the left/right/center leg servos
  LeftAnkle.attach(LeftAnklePin);

  RightHip.attach(RightHipPin);         // config the right leg
  RightAnkle.attach(RightAnklePin);

  NormalWalk();                         // HAVE to set the leg limits before initializing them
  LegsInit();                           // initialize legs so we know where we are at
 
  //  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results
  //  You will need change your uS connections to use this I believe?

  // Start in standard position
  WalkStop();
  RobotMode = RoamMode;
  delay(2000);                          // Give us some time to put it on the ground
  HipsCenterPosition();  


  // ** read in the sonar off the bat to avoid false start hits
  uS = sonar.ping(); 	                        // Send ping, get ping time in microseconds (uS).
  uScm = uS / US_ROUNDTRIP_CM;	                // convert to CM for using - I *THINK* this is off due to 20mhz clock on Orangutan?

  AvoidTimer = 0;                               // zero out/ initialize these timers
  AvoidTimerWalk = 0;  
  WalkTimer = millis();
}

void loop()					// Main Loop -runs over and over
{

  // ** First read the sensors and set and timers or status vars as needed so our subs know what we have to deal with

  // ** Read the sonar sensor(s) up top so we have a value to work with
  uS = sonar.ping(); 	                        // Send ping, get ping time in microseconds (uS).
  uScm = uS / US_ROUNDTRIP_CM;	                // convert to CM for using - I *THINK* this is off due to 20mhz clock on Orangutan?
  if (uScm == 0)
  {
    uScm = 9999;
  }          // if no distance we assume out of range and nothing there
  
//  uScm = 9999;        // Override / Set high for testing without Sonar
  

  //**************************************************************
  //** Process the robot "modes" - these can be changed on the fly if desired based on timers etc

  // Our basic loop looks at what "mode" we are in and then performs any actions and sets the "WalkRequest" which is used at the bottom switch command
  // The "Modes" can be subsumed by sensors just before the WalkRequest processing
  // This is just how I code all my robots anymore - do the high level stuff and then allow subsumption at the end if needed
  
// BEGIN STOP MODE
  // ** if we should stop then set the stop request
  if (RobotMode == StopMode)
  {                    // pure stop mode - just doing nothing here - need a way out of it
    WalkRequest = WalkModeStop;
  }
// END STOP MODE


// BEGIN WAITMODE  
  if (RobotMode == WaitMode)                    // wait mode just waits for a close up sonar hit and then jumps into roam mode
  {
    WalkRequest = WalkModeStop;
   
    // if you put something in front of him he jumps to roam mode
    if (uScm < 5)                               // we can get here by a long walk timer or other robot mode change
    {
     RobotMode = RoamMode;                      // so if something bothers us trigger avoid and then back to roam mode
    }
  }
// END WAITMODE

// BEGIN ROAM MODE
  // ** roam mode checks for sensors and then handles the requests as needed
  if (RobotMode == RoamMode)
  {                    // normal roam mode - we generally are forcing a forward walk unless something is in the way
        WalkRequest = WalkModeForward;

        if (uScm < 5)                               // we can get here by a long walk timer or other robot mode change
        {
         RobotMode = WaitMode;                      // so if something bothers us trigger avoid and then back to roam mode
        }
  }
// END ROAM MODE

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!! *** You can add your own "mode" processing here
// ** Such as light follow, avoid, whatever you want
// Just DO NOT USE delays or the walking loops will not work properly
// You CAN call a subroutine and have it do it's thing with delays if you are in stop mode though
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  //**** This area can override or subsume any actions setup above to avoid objects, etc
  // ** Handle any sonar hit movements
  // ** We have these AFTER the main mode seeking so it will subsume any requests made above
  // ** Even if we are sitting still and we have something in front of us we should act on it

  if (uScm < SonarHitRange) 
  {				           // Something too close for comfort so we want to backup and then turn
    if (AvoidTimer == 0)
    {                                      // if avoid timer not already running then start it
      AvoidTimer = millis();               // avoid timer runs until we reset below for object avoidance
      AvoidTimerWalk = millis();           // this is used to track what phase we are in during an avoidance and is reset at end of phase
      TurnDirection = random(1,10);        // never could get random(1) to give me a 1 or 0 so doing 10 and comparing for < 5 or greater
   }
  }


  if ((getET(AvoidTimer) > 5000))          // if we have been avoiding for three seconds then reset and move forward
  {
   AvoidTimer = 0;                         // otherwise we get stuck in the "I see it" "I don't see it" situation
   AvoidTimerWalk = 0; 
  }
  

  // if avoidtimer is running we are in "avoid mode" so we need to set the proper action for the current phase of avoid
  // this calls the AvoidItem Sub that sets the current "walkrequest" as needed in the avoid mode
  if (AvoidTimer > 0)                      // if we have this timer running we ARE in avoid mode so call the sub
  {
      AvoidItem();      
  }

// ** Depending on what we have set the walkrequest var to above we execute it here
// ** This could/should be in a sub to clean up the main loop but easier to debug right here

 switch (WalkRequest)
  {
    case 0: 
      WalkStop();   	  
      break;
    case 1:
      WalkForward();    
      break;
    case 2: 
      WalkBackwards();  
      break;
    case 3: 
      WalkLeft();  	  
      break;
    case 4: 
      WalkRight();      
      break;
  }  // End Current Step

  delay(LoopDelay);			// loop delay to keep things civilized
}

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Function returns which MAIN "step" or frame the walker sub should be on based onthe current timer, current walking delay, and max steps for thiscycle
// Walking subs then need a select case (switch in Arduino talk) for each step or frame of the process
// Walking timers are reset at the start of each mode of walking or reset upon completing all the steps
// I am passing variables here as I originally was going to use a common "step" sub but that didn't work with the need to reset the timer
int WalkStep(long CurWalkTimer, long CurWalkDelay, long Maxstep){
  int result;
  result = ((millis() - CurWalkTimer) / CurWalkDelay);
  if (result > Maxstep){
    result = 0;
    WalkTimer = millis();
  }
  return result;
}


// returns the current "step" or frame we are in for the avoid option
// In this case it is if we are supposed to be moving backwards, turning or going again
int AvoidStep(long CurAvoidTimer, long CurAvoidDelay, long Maxstep){
  int result;
  result = ((millis() - CurAvoidTimer) / CurAvoidDelay);
  if (result > Maxstep){
    result = 0;
    AvoidTimer = millis();
  }
  return result;  
}

// ** Basic walk forward routine - this is called again and again from the main loop so we have to use a timer
// ** to break down what actual step or frame of the walk we are in and execute those functions
// ** This can be be one or more servo position request calls
// ** Note that the LeftAnkleLeft, LeftHipIn, etc routines have their own "sub-step timers" as well to move the servos
// ** for the biped solution - otherwise it's way to jerky to keep balanced
void WalkForward(){
  // if we are not in forward mode then set it and reset the walktimer
  // we use the walktimer to select the step we should be making for each call to the sub
  if (WalkMode != WalkModeForward) {
    WalkMode = WalkModeForward;
    WalkTimer = millis();
  }
  // get current step - this is the MAIN STEP - NOT the substeps in between
  int CurStep = (int) WalkStep(WalkTimer, WalkDelay, 8);

// ** Here you can see the basic steps of a walk forward
//  0) Lift to Left (Right foot off ground)
//  1) Center up Right Ankle and both Hips
//  2) Swivel hips moving right foot forward (Left Hip In, Right Hip Out)
//  3) Center left ankle putting you back on the ground
//  4) Lift to Right (Left foot off ground)
//  5) Center Left Ankle and Hips
//  6) Swivel hips moving left foot forward
//  7) Center right ankle to back on the ground
//  Note this leaves us in a left foot forward stance so the WalkStop, etc corrects that to normal position

  switch (CurStep){
  case 0:               // Lift to the left for step on
    SubStepTimer = millis();    // we reset the substeptimer each new frame so we know we restart the subframes
    LeftAnkleLeft();        
    RightAnkleLeft();
    break;
  case 1:               // now center our ankle that is in the air and center our hip on the ground
    SubStepTimer = millis();
    RightAnkleCenter();
    LeftHipCenter();          // we also center up both hips at the same time to smooth out the walk some
    RightHipCenter();
    break;
  case 2: 
    SubStepTimer = millis();
    LeftHipIn();        // no do the swing motion to put our right foot out front
    RightHipOut();
    break;
  case 3: 
    SubStepTimer = millis();
    LeftAnkleCenter();  // drop our left ankle and we have taken a step - now to the other side
    break;
  case 4: 
    SubStepTimer = millis();
    RightAnkleRight();  // Now lift the right side and move it
    LeftAnkleRight();
    break;
  case 5: 
    SubStepTimer = millis();
    LeftAnkleCenter();
    LeftHipCenter();
    RightHipCenter();   // Center up the hips and the ankle in center mode - we shouldn't stay here long or it looks goofy I would bet
    break;
  case 6:
    SubStepTimer = millis();
    RightHipIn();        // do the swing forward
    LeftHipOut();
    break;
  case 7:
    SubStepTimer = millis();
    RightAnkleCenter();  // put our right foot down
    break;
  }  // End Current Step
}

// ** WalkBackwards is pretty much just the reverse of walking forward, Hips swivel other way
void WalkBackwards(){
  if (WalkMode != WalkModeBackwards) 
  {
    WalkMode = WalkModeBackwards;
    WalkTimer = millis();
  }

  // get current step - this has 5 max steps
  int CurStep = (int) WalkStep(WalkTimer, WalkDelay, 5);
  switch (CurStep){
  case 0: 
    SubStepTimer = millis();
    LeftAnkleLeft();
    RightAnkleLeft();
    break;
  case 1: 
    SubStepTimer = millis();
    LeftHipOut();
    RightAnkleCenter();
    RightHipIn();
    break;
  case 2: 
    SubStepTimer = millis();
    LeftAnkleCenter();
    break;
  case 3: 
    SubStepTimer = millis();
    RightAnkleRight();
    LeftAnkleRight();    
    break;
  case 4: 
    SubStepTimer = millis();
    RightHipOut();
    LeftHipIn();
    break;
  case 5: 
    SubStepTimer = millis();
    RightAnkleCenter();
    break;
  }
}

// ** WalkLeft/Right is a little different
// ** I am SURE something has a better frame sequence than this but it works for me
// 0) Make sure we are centered up first or we can fall over
// 1) Lift to the Right, Left Foot up
// 2) Center up that left foot and open up the left hip
// 3) Center up the right ankle and we're back on the ground
// 4) Left Left (Right Leg off the ground)
// 5) Center ankle in the air and Bring left hip back to center
// 6) Center left ankle and we are back on the ground
// the last step was in there to lift again but is not being used I don't think
void WalkLeft(){
  if (WalkMode != WalkModeLeft) {
    WalkMode = WalkModeLeft;
    WalkTimer = millis();
  }
  // get current step - this has 5 max steps
  int CurStep = (int) WalkStep(WalkTimer, WalkDelay, 6);
  switch (CurStep){
  case 0: 
    SubStepTimer = millis();
    LeftHipCenter();
    RightHipCenter();
    LeftAnkleCenter();
    RightAnkleCenter();
    break;
  case 1: 
    SubStepTimer = millis();
    LeftAnkleRight();
    RightAnkleRight();
    break;
  case 2: 
    SubStepTimer = millis();
    LeftAnkleCenter();
    LeftHipOut();
    break;
  case 3: 
    SubStepTimer = millis();
    RightAnkleCenter();
    break;
  case 4: 
    SubStepTimer = millis();
    RightAnkleLeft();
    LeftAnkleLeft();
    break;
  case 5: 
    SubStepTimer = millis();
    RightAnkleCenter();
    LeftHipCenter();
    break;
  case 6: 
    SubStepTimer = millis();
    LeftAnkleCenter();
    break;
  case 7: 
    SubStepTimer = millis();
    RightAnkleRight();
    LeftAnkleRight();
    LeftHipCenter();
    break;

  } 
}

// ** Walk right should be pretty much the opposite of walk left
void WalkRight(){
  if (WalkMode != WalkModeRight) {
    WalkMode = WalkModeRight;
    WalkTimer = millis();
  }
  int CurStep = (int) WalkStep(WalkTimer, WalkDelay, 6);
  switch (CurStep){
  case 0: 
    SubStepTimer = millis();
    LeftHipCenter();
    RightHipCenter();
    LeftAnkleCenter();
    RightAnkleCenter();
    break;
  case 1: 
    SubStepTimer = millis();
    LeftAnkleLeft();
    RightAnkleLeft();
    break;
  case 2: 
    SubStepTimer = millis();
    RightAnkleCenter();
    RightHipOut();
    break;
  case 3: 
    SubStepTimer = millis();
    LeftAnkleCenter();
    break;
  case 4: 
    SubStepTimer = millis();
    RightAnkleRight();
    LeftAnkleRight();
    break;
  case 5: 
    SubStepTimer = millis();
    LeftAnkleCenter();
    RightHipCenter();
    break;
  case 6: 
    SubStepTimer = millis();
    RightAnkleCenter();
    break;
  case 7: 
    SubStepTimer = millis();
    RightAnkleLeft();
    LeftAnkleLeft();
    RightHipCenter();
    break;
  } 
}

// ** Initialize leg positions and set CURRENT values so we know where we are starting at
void LegsInit(){
  LeftHipCurrent     = LeftHipCenterLimit;      // Save the positions at start to have our reference points
  RightHipCurrent    = RightHipCenterLimit;
  LeftAnkleCurrent   = LeftAnkleCenterLimit;
  RightAnkleCurrent  = RightAnkleCenterLimit;

  LeftAnklePosition(LeftAnkleCurrent);
  RightAnklePosition(RightAnkleCurrent);

  LeftHipPosition(LeftHipCurrent);
  RightHipPosition(RightHipCurrent);
}


//** WalkSto This just centers everything up and sets the mode to "stop"
void WalkStop(){
  if (WalkMode != WalkModeForward) {
    WalkMode = WalkModeStop;
    WalkTimer = millis();
  }

  LeftHipCenter();
  RightHipCenter();

  LeftAnkleCenter();
  RightAnkleCenter();
}


// **** These are the individual leg frame positions needed for the movement
// **** These are called from other subs to figure out how to move
void MoveLeftHip(byte MoveToPosition)
{
  if (getET(SubStepTimer) < LoopDelay)              // If we are just starting this step, i.e. our timer is less than loop timer
  {                                                 // we calculate how many increments we need to make to get there
    LeftHipInc = (MoveToPosition - LeftHipCurrent) / (SubSteps);
  }
  // *** NOTE: If you remove the if statement here and just let it divide it will make fast starts and slow ending points
  // *** BUT it may not complete in the amount of step to step time you have - experiment with it and see


  LeftHipCurrent = LeftHipCurrent + LeftHipInc;     // we then add that increment to our current position and call the full write command
  LeftHip.write(LeftHipCurrent);                    // This makes the motion incremental based on your WalkTimer and SubStep variables which can be tuned
}

void MoveRightHip(byte MoveToPosition)
{
  if (getET(SubStepTimer) < LoopDelay)
  {
    RightHipInc = (MoveToPosition - RightHipCurrent) / (SubSteps);
  }

  RightHipCurrent = RightHipCurrent + RightHipInc;

  RightHip.write(RightHipCurrent);
}

void MoveLeftAnkle(byte MoveToPosition)
{
  if (getET(SubStepTimer) < LoopDelay)
  {
     LeftAnkleInc = (MoveToPosition - LeftAnkleCurrent) / (SubSteps);
  }

  LeftAnkleCurrent = LeftAnkleCurrent + LeftAnkleInc;

  LeftAnkle.write(LeftAnkleCurrent);
}

void MoveRightAnkle(byte MoveToPosition)
{
  if (getET(SubStepTimer) < LoopDelay)
  {
     RightAnkleInc = (MoveToPosition - RightAnkleCurrent) / SubSteps;
  }

  RightAnkleCurrent = RightAnkleCurrent + RightAnkleInc;

  RightAnkle.write(RightAnkleCurrent);
}

// ** These are the original commands to position joints
// ** They now call the MOVE subs that actually calculate and move the servos
// ** These do NOT put the servo in the direct position immediately and must be called over and over
// ** This is just easier to remember than "MoveRightHip(RightHipCenterLimit)" etc to me

// ** Right Hip Positions
void RightHipIn()
{
    MoveRightHip(RightHipInLimit);
}

void RightHipCenter()
{
  MoveRightHip(RightHipCenterLimit);
}

void RightHipOut()
{
  MoveRightHip(RightHipOutLimit);
}

// ** Left Hip Positions
void LeftHipIn()
{
  MoveLeftHip(LeftHipInLimit);
}

void LeftHipCenter()
{
  MoveLeftHip(LeftHipCenterLimit);
}

void LeftHipOut()
{
  MoveLeftHip(LeftHipOutLimit);
}

// ** Left Ankle Positions
void LeftAnkleLeft()
{
    MoveLeftAnkle(LeftAnkleLeftLimit);
}

void LeftAnkleCenter()
{
    MoveLeftAnkle(LeftAnkleCenterLimit);
}

void LeftAnkleRight()
{
    MoveLeftAnkle(LeftAnkleRightLimit);
}

// ** Right Ankle Positions
void RightAnkleLeft()
{  
    MoveRightAnkle(RightAnkleLeftLimit);
}

void RightAnkleCenter()
{
    MoveRightAnkle(RightAnkleCenterLimit);
}
void RightAnkleRight()
{
    MoveRightAnkle(RightAnkleRightLimit);
}

// *** THESE subs, on the other hand DO DIRECT POSITIONING

// ** These subs put servos in DIRECT position without timed moves
// ** Mainly for funky positions, etc
// ** There is no sweep to these subs 
// ** The XXCurrent variables are updated as well
void LeftAnklePosition(byte ServoPosition){
    LeftAnkleCurrent = ServoPosition;
    LeftAnkle.write(ServoPosition);
}

void RightAnklePosition(byte ServoPosition){
    RightAnkleCurrent = ServoPosition;
    RightAnkle.write(ServoPosition);
}

void LeftHipPosition(byte ServoPosition){
    LeftHipCurrent = ServoPosition;
    LeftHip.write(ServoPosition);
}

void RightHipPosition(byte ServoPosition){
    RightHipCurrent = ServoPosition;
    RightHip.write(ServoPosition);
}


// ** returns the ET in millis for timer variable passed
long getET(long mytimer){
  long result;
  result = (millis() - mytimer);
  return result;
}


// ** This is the basic "AVOID" routine
// ** If the avoidtimer is running, this should get called from the main loop
// ** depending on what phase we are in the timer we backup and turn
// ** if we have timed out then zero the timer so the normal command take over
// ****** NOTE: ALL THIS DOES IS SET THE WalkRequest varialbe for the main switch case to handle
void AvoidItem(){
  // ** three seconds of walking fowards
  if (getET(AvoidTimerWalk) < 2000) {
    WalkRequest = WalkModeBackwards;
  }

  // ** up the walking speed by double
  if ((getET(AvoidTimerWalk) > 2000 && getET(AvoidTimerWalk) < 6000)){
    //** Set a direction based on the random TurnDirection previously created
    if (TurnDirection < 6){             // if random direction was right then turn right
      WalkRequest = WalkModeRight;
    }
    else{
      WalkRequest = WalkModeLeft;
    }      
  }

  if ((getET(AvoidTimerWalk) > 6000)){      // At this point we are done avoiding so go forward, turn off avoid timer and LEDs
    WalkRequest = WalkModeForward;
    AvoidTimer = 0;    
  }
}



// ** Center Up Both Ankles and Hips at once
void CenterPosition()
{
  HipsCenterPosition();
  AnklesCenterPosition();
}

//** Moves both hip to the left - could use for shuffle side to side?
void HipsLeftPosition()
{
  LeftHipPosition(LeftHipOutLimit);
  RightHipPosition(RightHipInLimit);
}

void HipsRightPosition()
{
  LeftHipPosition(LeftHipInLimit);
  RightHipPosition(RightHipOutLimit);
}

void HipsCenterPosition()
{
  LeftHipPosition(LeftHipCenterLimit);
  RightHipPosition(RightHipCenterLimit);
}

void AnklesCenterPosition()
{
  LeftAnklePosition(LeftAnkleCenterLimit);
  RightAnklePosition(RightAnkleCenterLimit);
}




// *****************************************************************
// ** This is an example of how you can change the walk pattern on the fly
// ** Call the NormalWalk() routine for your normal settings
// ** Call the ShortWalk() routine and you now have a short shuffle type of walk
// ** 


void NormalWalk()
{
    LeftHipCenterLimit= 80;
    LeftHipInLimit = 40;
    LeftHipOutLimit  = 120;

    RightHipCenterLimit= 85;
    RightHipInLimit = 140;              // Limit / Positions for Front/Center/Rear for Right Leg
    RightHipOutLimit  = 45;

    LeftAnkleCenterLimit=90;
    LeftAnkleLeftLimit = 120;
    LeftAnkleRightLimit= 50;
    
    RightAnkleCenterLimit=90;
    RightAnkleLeftLimit = 135;
    RightAnkleRightLimit= 55;
}


// ** This shortens up the walk like a shuffle or similar
// ** You CAN leave out the CenterLimit variable if you do not want them changed
void ShortWalk()
{
    LeftHipCenterLimit= 90;
    LeftHipInLimit = 80;
    LeftHipOutLimit  = 100;

    RightHipCenterLimit= 90;
    RightHipInLimit = 100;
    RightHipOutLimit  = 80;

    LeftAnkleCenterLimit=90;
    LeftAnkleLeftLimit = 100;
    LeftAnkleRightLimit= 80;
    
    RightAnkleCenterLimit=90;
    RightAnkleLeftLimit = 100;
    RightAnkleRightLimit= 80;
}


