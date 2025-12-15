#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the IQ2 brain screen
vex::brain Brain;

// define your global instances of motors and other devices here
inertial BrainInertial = inertial();
controller Controller = controller();
motor MotorPinMotorA = motor(PORT3, false);
motor MotorPinMotorB = motor(PORT9, true);
motor_group MotorPin = motor_group(MotorPinMotorA, MotorPinMotorB);

motor MotorBeamMotorA = motor(PORT5, false);
motor MotorBeamMotorB = motor(PORT11, true);
motor_group MotorBeam = motor_group(MotorBeamMotorA, MotorBeamMotorB);

touchled TouchLED12 = touchled(PORT12);
pneumatic Pneumatic_Pin_Beam = pneumatic(PORT4);
motor MotorLeft = motor(PORT6, false);
motor MotorRight = motor(PORT10, true);
pneumatic guide = pneumatic(PORT8);

distance D1 = distance(PORT1);
distance D2 = distance(PORT2);

// AI Classification Competition Element IDs - Mix & Match
enum gameElementsMixAndMatch
{
  beam,
  bluePin,
  redPin,
  orangePin,
};

// AI Vision Code Descriptions
vex::aivision AIVision7(PORT7, aivision::ALL_AIOBJS);

// generating and setting random seed
void initializeRandomSeed()
{
  wait(100, msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  int seed = int(xAxis + yAxis + zAxis);
  srand(seed);
}

void vexcodeInit()
{
  initializeRandomSeed();
}

bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

// User defined function declarations
void Place_beam();
void Drop_down();
void Drop_Down_Grab_Up();
void Grab_Beam_up();
void R3F();
void Grab_then_up();
void Movemen_v_fb_d(double velocity, double FB, double distance);
void turnRightLong(double input, double ver);
void turnLeftLong(double input, double ver);
void Turn_h_v_m_shot_Aj(double heading, double velocity, double momentum);
void turnLeftLong(double input, double ver);
void Movemen_v_fb_d(double velocity, int FB, double distance);
void Movemen_v_fb_d_Aj(double velocity, int FB, double distance);

int Screen_precision = 0, Console_precision = 0, AIVision7_objectIndex = 0, LiftBeam = 0;
bool startgame, beamon, AjON, touched = true, beamoff = false, armfont = false, armback = false, Arm_Grab_pin = false;
double width, cenX;

// Starting angle
int _StartAngle = 26;

// ========== FUNTION BEAM ==========

void Drop_down_beam()
{
  guide.extend(cylinder1);
  wait(100, msec);
  MotorBeam.setStopping(hold);
  beamon ? MotorBeam.setVelocity(100, percent) : MotorBeam.setVelocity(45, percent);
  MotorBeam.setMaxTorque(100, percent);
  MotorBeam.spin(forward);
  wait(0.5, seconds);
  while (MotorBeam.velocity(vex::velocityUnits::pct) > 1)
  {
    MotorBeam.spin(forward);
  }
  MotorBeam.setStopping(brake);
  MotorBeam.setMaxTorque(100, percent);
  MotorBeam.stop();
  MotorBeam.stop(coast);
}

void Place_beam()
{
  MotorLeft.stop();
  MotorRight.stop();
  MotorBeam.setVelocity(65.0, percent);
  guide.extend(cylinder1);
  wait(100, msec);
  MotorBeam.spinFor(forward, 100.0, degrees, true);
  Pneumatic_Pin_Beam.retract(cylinder2);
  MotorBeam.spinFor(reverse, 100.0, degrees, false);
  MotorLeft.setVelocity(85.0, percent);
  MotorRight.setVelocity(85.0, percent);
  MotorLeft.spinFor(forward, 130.0, degrees, false);
  MotorRight.spinFor(forward, 130.0, degrees, false);
}

void Grab_Beam_up()
{
  MotorBeam.setStopping(hold);
  MotorBeam.spinFor(reverse, 150.0, degrees, false);
  wait(0.8, seconds);
  guide.retract(cylinder1);
  wait(0.7, seconds);
  MotorBeam.stop();
}

void BeamMotorUp()
{
  while (true)
  {
    if (LiftBeam == 1)
    {
      MotorBeam.setMaxTorque(100, percent);
      MotorBeam.setVelocity(100, percent);
      MotorBeam.spinFor(reverse, 520, degrees, true);
      break;
    }
    if (LiftBeam == 2)
    {
      MotorBeam.setMaxTorque(100, percent);
      MotorBeam.setVelocity(100, percent);
      MotorBeam.spinFor(reverse, 100, degrees, true);
      break;
    }
    if (LiftBeam == 3)
    {
      MotorBeam.setMaxTorque(100, percent);
      MotorBeam.setVelocity(100, percent);
      MotorBeam.spinFor(reverse, 420, degrees, true);
      break;
    }
  }
}

void Place_Standoff()
{
  MotorBeam.setMaxTorque(100, percent);
  // Changed Velocity Before 20
  if (!beamoff)
  {
    MotorBeam.setVelocity(100, percent);
    MotorBeam.spinFor(forward, 70, degrees, true);
    wait(0.3, seconds);
    Pneumatic_Pin_Beam.retract(cylinder2);
  }
  else
  {
    MotorBeam.setVelocity(45, percent);
    MotorBeam.spinFor(forward, 70, degrees, true);
    wait(0.4, seconds);
    Pneumatic_Pin_Beam.retract(cylinder2);
  }
  MotorBeam.spinFor(reverse, 60, degrees, false);
  Movemen_v_fb_d(100, 0, 50);
  armback = true;
}

// ========== FUNTION MOVE AND TURN ==========
void Movemen_v_fb_d(double velocity, int FB, double distance)
{
  MotorLeft.setVelocity(velocity, percent);
  MotorRight.setVelocity(velocity, percent);
  if (FB == 0)
  {
    MotorLeft.spinFor(forward, (distance / 0.833), degrees, false);
    MotorRight.spinFor(forward, (distance / 0.833), degrees, true);
    MotorLeft.setStopping(brake);
    MotorRight.setStopping(brake);
    MotorLeft.stop();
    MotorRight.stop();
    while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
    {
      wait(10, msec);
    }
  }
  if (FB == 1)
  {
    MotorLeft.spinFor(reverse, (distance / 0.833), degrees, false);
    MotorRight.spinFor(reverse, (distance / 0.833), degrees, true);
    MotorLeft.setStopping(brake);
    MotorRight.setStopping(brake);
    MotorLeft.stop();
    MotorRight.stop();
    while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
    {
      wait(10, msec);
    }
  }
}

void Movemen_v_fb_d_Aj(double velocity, int FB, double distance)
{
  double targetHeading = BrainInertial.heading();

  double kP = 0.9;
  double kD = 0.7;
  double deadband = 2.0;

  double error = 0;
  double lastError = 0;

  MotorLeft.resetPosition();
  MotorRight.resetPosition();

  while (fabs(MotorLeft.position(degrees)) < (distance / 0.833))
  {
    double currentHeading = BrainInertial.heading();
    error = currentHeading - targetHeading;

    // normalize error (-180 ถึง 180)
    if (error > 180)
      error -= 360;
    if (error < -180)
      error += 360;

    // deadband
    if (fabs(error) < deadband)
      error = 0;

    double derivative = error - lastError;
    lastError = error;

    double correction = (kP * error) + (kD * derivative);

    // จำกัดแรงเลี้ยว ป้องกัน over
    if (correction > 25)
      correction = 25;
    if (correction < -25)
      correction = -25;

    double leftSpeed = velocity - correction;
    double rightSpeed = velocity + correction;

    // clamp speed
    if (leftSpeed > 100)
      leftSpeed = 100;
    if (leftSpeed < -100)
      leftSpeed = -100;
    if (rightSpeed > 100)
      rightSpeed = 100;
    if (rightSpeed < -100)
      rightSpeed = -100;

    if (FB == 0)
    {
      MotorLeft.spin(forward, leftSpeed, percent);
      MotorRight.spin(forward, rightSpeed, percent);
    }
    else
    {
      MotorLeft.spin(reverse, leftSpeed, percent);
      MotorRight.spin(reverse, rightSpeed, percent);
    }

    wait(10, msec);
  }

  MotorLeft.stop(brake);
  MotorRight.stop(brake);
}

double ComputeTurnSpeed(double absErr, double maxSpeed)
{
  double minSpeed = maxSpeed * 0.2;
  double maxErrForScale = 90.0;

  double t = absErr / maxErrForScale;
  if (t > 1.0)
    t = 1.0;

  // curve: make small errors give much smaller speeds
  t = t * t; // square makes the curve “steeper” near 0

  double speed = minSpeed + (maxSpeed - minSpeed) * t;
  return speed;
}

void TurnRight_h_v_m(double heading, double velocity, double momentum)
{
  while (true)
  {
    double current = BrainInertial.heading();
    double error = heading - current; // positive → need left turn
    double absErr = fabs(error);

    // ---- 1. Done / inside momentum zone ----
    if (absErr <= momentum)
    {
      MotorLeft.setStopping(brake);
      MotorRight.setStopping(brake);
      MotorLeft.stop();
      MotorRight.stop();

      while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
        wait(10, msec);

      double finalHeading = BrainInertial.heading();
      double finalErr = fabs(heading - finalHeading);

      if (finalErr <= momentum)
      {
        printf("Final heading (OK): %.2f\n", finalHeading);
        break;
      }
      else
      {
        printf("Correcting overshoot... err=%.2f\n", finalErr);
        while (fabs(BrainInertial.heading() - heading) > momentum)
        {
          MotorLeft.spin(reverse, 15, percent);
          MotorRight.spin(forward, 15, percent);
          wait(10, msec);
        }
        MotorLeft.stop(brake);
        MotorRight.stop(brake);

        wait(100, msec);
        continue;
      }
    }

    // ---- 2. Slowdown system to prevent sliding ----
    double turnSpeed = ComputeTurnSpeed(absErr, velocity);
    // turnSpeed *= 0.9;
    if (turnSpeed < 10)
      turnSpeed = 10;

    MotorLeft.setVelocity(turnSpeed, percent);
    MotorRight.setVelocity(turnSpeed, percent);

    // ---- 3. Actual turn left ----
    MotorLeft.spin(forward);
    MotorRight.spin(reverse);

    wait(20, msec);
    if (fabs(MotorLeft.velocity(pct) < 1) && fabs(MotorRight.velocity(pct) < 1))
    {
      turnSpeed += 1;
    }
  }

  wait(0.1, seconds);
}

void TurnLeft_h_v_m(double heading, double velocity, double momentum)
{
  while (true)
  {
    double current = BrainInertial.heading();
    double error = heading - current; // negative → need right turn
    double absErr = fabs(error);

    if (absErr <= momentum)
    {
      MotorLeft.setStopping(brake);
      MotorRight.setStopping(brake);
      MotorLeft.stop();
      MotorRight.stop();

      while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
        wait(10, msec);

      double finalHeading = BrainInertial.heading();
      double finalErr = fabs(heading - finalHeading);

      if (finalErr <= momentum)
      {
        printf("Final heading (OK): %.2f\n", finalHeading);
        break;
      }
      else
      {
        printf("Correcting overshoot... err=%.2f\n", finalErr);
        while (fabs(BrainInertial.heading() - heading) > momentum)
        {
          MotorLeft.spin(forward, 15, percent);
          MotorRight.spin(reverse, 15, percent);
          wait(10, msec);
        }
        MotorLeft.stop(brake);
        MotorRight.stop(brake);

        wait(100, msec);
        continue;
      }
    }

    double turnSpeed = ComputeTurnSpeed(absErr, velocity);
    if (turnSpeed < 10)
      turnSpeed = 10;

    MotorLeft.setVelocity(turnSpeed, percent);
    MotorRight.setVelocity(turnSpeed, percent);

    MotorLeft.spin(reverse);
    MotorRight.spin(forward);

    wait(20, msec);
    if (fabs(MotorLeft.velocity(pct) < 1) && fabs(MotorRight.velocity(pct) < 1))
    {
      turnSpeed += 1;
    }
  }

  wait(0.1, seconds);
}

void turnRightLong(double Target, double ver)
{
  MotorLeft.setVelocity(ver, pct);
  MotorRight.setVelocity(ver, pct);
  MotorRight.setPosition(0.0, degrees);
  MotorLeft.setPosition(0.0, degrees);
  MotorLeft.stop(brake);
  MotorRight.stop(brake);
  double dis = 0;

  if (BrainInertial.heading() > Target)
  {
    dis = fabs((360 - BrainInertial.heading()) + Target);
  }
  else
  {
    dis = fabs(Target - BrainInertial.heading());
  }

  dis *= 2;
  MotorLeft.spinToPosition(dis, degrees, false);
  MotorRight.spinToPosition(-dis, degrees, true);
  MotorLeft.stop(brake);
  MotorRight.stop(brake);
  printf("fffffROK\n");
}

void turnLeftLong(double input, double ver)
{
  MotorLeft.setVelocity(ver, pct);
  MotorRight.setVelocity(ver, pct);
  MotorRight.setPosition(0.0, degrees);
  MotorLeft.setPosition(0.0, degrees);
  MotorLeft.stop(brake);
  MotorRight.stop(brake);
  double dis = 0;

  if (BrainInertial.heading() < input)
  {
    dis = fabs((360 - input) + BrainInertial.heading());
  }
  else
  {
    dis = fabs(input - BrainInertial.heading());
  }
  dis *= 2;
  MotorLeft.spinToPosition(-dis, degrees, false);
  MotorRight.spinToPosition(dis, degrees, true);
  MotorLeft.stop(brake);
  MotorRight.stop(brake);
  printf("fffffLOK\n");
}

void Turn_h_v_m_shot_Aj(double heading, double velocity, double momentum)
{
  while (!(heading == BrainInertial.heading()))
  {
    double headingnow = BrainInertial.heading();
    if ((heading - momentum) > headingnow)
    {
      MotorLeft.setVelocity(velocity, percent);
      MotorRight.setVelocity(velocity, percent);
      MotorLeft.spin(forward);
      MotorRight.spin(reverse);
    }
    else if ((heading + momentum) < headingnow)
    {
      MotorLeft.setVelocity(velocity, percent);
      MotorRight.setVelocity(velocity, percent);
      MotorLeft.spin(reverse);
      MotorRight.spin(forward);
    }
    else
    {
      MotorLeft.stop(brake);
      MotorRight.stop(brake);
      break;
    }
  }
}

// ========== FUNTION PIN ==========

void Grab_then_up()
{
  MotorPin.setStopping(hold);
  MotorLeft.stop();
  MotorRight.stop();
  Pneumatic_Pin_Beam.extend(cylinder1);
  MotorPin.spinFor(reverse, 160.0, degrees, true);
  guide.extend(cylinder2);
  MotorPin.stop();
  MotorLeft.stop();
  MotorRight.stop();
}

void Drop_down()
{
  MotorLeft.stop();
  MotorRight.stop();
  guide.retract(cylinder2);
  wait(0.35, seconds);
  MotorPin.setStopping(coast);
  MotorPin.spinFor(forward, 155.0, degrees, false);
  wait(0.3, seconds);
  MotorLeft.stop();
  MotorRight.stop();
  Pneumatic_Pin_Beam.retract(cylinder1);
  MotorPin.stop();
}

void Drop_Down_Grab_Up()
{
  guide.retract(cylinder2);
  wait(0.23, seconds);
  MotorPin.spinFor(forward, 80.0, degrees, true);
  Pneumatic_Pin_Beam.retract(cylinder1);
  MotorPin.spinFor(forward, 80.0, degrees, true);
  MotorLeft.stop(brake);
  MotorRight.stop(brake);
  MotorLeft.setVelocity(25.0, percent);
  MotorRight.setVelocity(25.0, percent);
  MotorLeft.spinFor(forward, 300.0, degrees, false);
  MotorRight.spinFor(forward, 300.0, degrees, false);
  wait(0.3, seconds);
  Pneumatic_Pin_Beam.extend(cylinder1);
  MotorPin.spinFor(reverse, 160.0, degrees, false);
  wait(0.25, seconds);
  guide.extend(cylinder2);
}

void R3F()
{
  Pneumatic_Pin_Beam.extend(cylinder2);
  MotorPin.spinFor(reverse, 700.0, degrees, false);
  wait(1500, msec);
  Pneumatic_Pin_Beam.retract(cylinder1);
  wait(0.1, seconds);
  armfont = true;
}

// ========== FUNTION AI CAM ==========

double ComputeApproachSpeed(double width, double maxSpeed)
{
  // Tune these three numbers for your robot & camera
  const double targetWidth =
      85.0; // how big the pin should look when you want to stop
  const double minDetectWidth =
      20.0; // typical width when it's "far but visible"

  double minSpeed = maxSpeed * 0.2; // don't go slower than 20% of max
  double maxErr = targetWidth - minDetectWidth;

  // Error = how much smaller than target the current width is
  double err = targetWidth - width;

  // Clamp error between 0 and maxErr
  if (err < 0)
    err = 0;
  if (err > maxErr)
    err = maxErr;

  // Normalize → 0..1
  double t = err / maxErr;

  // Curve: square to make it gentler near target
  t = t * t;

  // When far (err large) → t ~1 → speed ~maxSpeed
  // When close (err small) → t ~0 → speed ~minSpeed
  double speed = minSpeed + (maxSpeed - minSpeed) * t;
  return speed;
}

void Go_To_Pin(int Gopin)
{
  const double targetWidth = 100.0; // when object is this big, we stop
  const double maxSpeed = 100.0;    // max forward speed (tune if needed)

  while (true)
  {
    AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

    int bestIndex = -1;
    int maxWidth = 0;

    // 1) Find the closest (widest) object with the requested ID
    for (int i = 0; i < AIVision7.objectCount; i++)
    {
      if (AIVision7.objects[i].id == Gopin &&
          AIVision7.objects[i].width > maxWidth)
      {
        maxWidth = AIVision7.objects[i].width;
        bestIndex = i;
      }
    }

    if (bestIndex != -1)
    {
      double width = AIVision7.objects[bestIndex].width;

      // 2) Compute speed from width, like ComputeTurnSpeed
      double speed = ComputeApproachSpeed(width, maxSpeed);

      MotorLeft.setVelocity(speed, percent);
      MotorRight.setVelocity(speed, percent);
      MotorLeft.spin(forward);
      MotorRight.spin(forward);

      // 3) Close enough → brake and exit
      if (width >= targetWidth)
      {
        MotorLeft.stop(brake);
        MotorRight.stop(brake);

        // Optional: wait until really stopped
        while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
        {
          wait(10, msec);
        }
        MotorLeft.setStopping(coast);
        MotorRight.setStopping(coast);
        break;
      }
    }
    else
    {
      // No object found → stop and keep searching
      MotorLeft.stop();
      MotorRight.stop();
    }

    wait(10, msec); // small delay to avoid hammering the CPU
  }
}

//========================= START ROBOT ===========================
int take_off()
{
  startgame = true;
  beamon = true;
  AjON = false;
  MotorPin.setPosition(0.0, degrees);
  MotorBeam.setPosition(0.0, degrees);
  MotorPin.setVelocity(80.0, percent);
  MotorPin.setMaxTorque(100.0, percent);
  MotorBeam.setVelocity(100.0, percent);
  MotorBeam.setMaxTorque(100.0, percent);
  MotorLeft.spin(reverse, 2, pct);
  MotorRight.spin(reverse, 2, pct);
  MotorPin.setStopping(hold);
  MotorBeam.setStopping(coast);
  Pneumatic_Pin_Beam.retract(cylinder1);
  Pneumatic_Pin_Beam.retract(cylinder2);
  guide.extend(cylinder1);
  Pneumatic_Pin_Beam.pumpOn();
  TouchLED12.setColor(red);
  Brain.Screen.setCursor(1, 1);
  guide.retract(cylinder2);
  Drop_down_beam();
  MotorBeam.spinFor(reverse, 300.0, degrees);
  wait(1, seconds);
  Pneumatic_Pin_Beam.extend(cylinder2);
  beamon = false;
  return 0;
}

//===================== Dont wait FUNTION ======================
void controArm()
{
  while (true)
  {
    if (armfont)
    {
      R3F();
      guide.retract(cylinder2);
      MotorPin.setStopping(coast);
      MotorPin.spinFor(forward, 700.0, degrees, false);
      wait(1000, msec);
      MotorPin.stop(coast);
      armfont = false;
    }
    if (armback)
    {
      Drop_down_beam();
      armback = false;
    }
    if (Arm_Grab_pin)
    {
      MotorPin.setStopping(hold);
      Pneumatic_Pin_Beam.extend(cylinder1);
      MotorPin.spinFor(reverse, 160.0, degrees, true);
      guide.extend(cylinder2);
      MotorPin.stop();
      Arm_Grab_pin = false;
    }

    wait(10, msec);
  }
}

//================================ Print ===============================
int PrintCon()
{
  while (true)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("D2: %.2f", D2.objectDistance(mm));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("D1: %.2f", D1.objectDistance(mm));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("%.2f", BrainInertial.heading(degrees));

    int h = BrainInertial.heading();

    printf("Heading: %.d deg\n", h);
    wait(50, msec);
  }
  return 0;
}

//=========================== AI FUNTION ===================================
double ComputeAlignSpeed(double absErr, double maxSpeed)
{
  double minSpeed = maxSpeed * 0.25; // don't go slower than 25%
  double maxErrForScale = 80.0;      // pixels; tune for your camera view

  double t = absErr / maxErrForScale;
  if (t > 1.0)
    t = 1.0;

  // Square curve: big error → near maxSpeed, small error → gentle
  t = t * t;

  double speed = minSpeed + (maxSpeed - minSpeed) * t;
  return speed;
}

// void AjpinLeft(int targetIDLeft)
// {
//   // int count = 0;
//   while (true)
//   {
//     TouchLED12.setColor(orange);

//     AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

//     double bestDist = 999;
//     double offset = 0;

//     for (int i = 0; i < AIVision7.objectCount; i++)
//     {
//       if (AIVision7.objects[i].id == targetIDLeft)
//       {
//         double cx = AIVision7.objects[i].centerX;
//         double d = abs(cx - 80);

//         if (d < bestDist)
//         {
//           bestDist = d;
//           offset = cx - 80;
//         }
//       }
//     }

//     if (bestDist < 10)
//     {
//       MotorLeft.stop(brake);
//       MotorRight.stop(brake);
//       break;
//     }
//     else
//     {
//       MotorLeft.setVelocity(15, percent);
//       MotorRight.setVelocity(15, percent);
//       if (offset > 0)
//       {
//         MotorLeft.spin(forward);
//         MotorRight.stop();
//       }
//       else if (offset < 0)
//       {
//         MotorLeft.stop();
//         MotorRight.spin(forward);
//       }
//     }

//     wait(10, msec);
//     TouchLED12.setColor(blue_green);
//   }
// }

void Ajpinright(int targetIDRight)
{
  int count = 0;
  while (true)
  {
    TouchLED12.setColor(blue);

    AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

    double bestDist = 999;
    double offset = 0;

    for (int i = 0; i < AIVision7.objectCount; i++)
    {
      if (AIVision7.objects[i].id == targetIDRight)
      {
        double cx = AIVision7.objects[i].centerX;
        double d = abs(cx - 210);

        if (d < bestDist)
        {
          bestDist = d;
          offset = cx - 210;
        }
      }
    }

    if (bestDist < 10)
    {
      count++;
      if (count == 2)
      {
        MotorLeft.stop(brake);
        MotorRight.stop(brake);
        break;
      }
    }
    else
    {
      MotorLeft.setVelocity(15, percent);
      MotorRight.setVelocity(15, percent);
      if (offset > 0)
      {
        MotorLeft.spin(forward);
        MotorRight.stop();
      }
      else if (offset < 0)
      {
        MotorLeft.stop();
        MotorRight.spin(forward);
      }
    }

    wait(10, msec);
    TouchLED12.setColor(blue_green);
  }
}
// void AjpinFind(int targetID)
// {
//   int count = 0;
//   while (true)
//   {
//     TouchLED12.setColor(red);

//     AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

//     double bestDist = 999;
//     double offset = 0;

//     for (int i = 0; i < AIVision7.objectCount; i++)
//     {
//       if (AIVision7.objects[i].id == targetID)
//       {
//         double cx = AIVision7.objects[i].centerX;
//         double d = abs(cx - 160);

//         if (d < bestDist)
//         {
//           bestDist = d;
//           offset = cx - 160;
//         }
//       }
//     }

//     if (bestDist < 30)
//     {
//       count++;
//       if (count == 2)
//       {
//         MotorLeft.stop(brake);
//         MotorRight.stop(brake);
//         break;
//       }
//     }
//     else
//     {
//       MotorLeft.setVelocity(30, percent);
//       MotorRight.setVelocity(30, percent);

//       if (offset > 0)
//       {
//         MotorLeft.spin(forward);
//         MotorRight.stop();
//       }
//       else if (offset < 0)
//       {
//         MotorLeft.stop();
//         MotorRight.spin(forward);
//       }
//     }

//     wait(10, msec);
//     TouchLED12.setColor(blue_green);
//   }
// }

// void FindBeam(int targetID)
// {
//   int count = 0;
//   while (true)
//   {
//     TouchLED12.setColor(purple);

//     AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

//     double bestY = -1;
//     double offset = 0;

//     for (int i = 0; i < AIVision7.objectCount; i++)
//     {
//       if (AIVision7.objects[i].id == targetID)
//       {
//         double cy = AIVision7.objects[i].centerY;
//         double d = abs(cy - 160);

//         if (d < bestY)
//         {
//           bestY = d;
//           offset = cy - 160;
//         }
//       }
//     }

//     if (bestY < 30)
//     {
//       count++;
//       if (count == 2)
//       {
//         MotorLeft.stop(brake);
//         MotorRight.stop(brake);
//         break;
//       }
//     }
//     else
//     {
//       MotorLeft.setVelocity(30, percent);
//       MotorRight.setVelocity(30, percent);

//       if (offset > 0)
//       {
//         MotorLeft.spin(forward);
//         MotorRight.stop();
//       }
//       else if (offset < 0)
//       {
//         MotorLeft.stop();
//         MotorRight.spin(forward);
//       }
//     }

//     wait(10, msec);
//     TouchLED12.setColor(blue_green);
//   }
// }

// =================== TOUCH LED EVENT ===================

void onevent_TouchLED12_pressed_0()
{
  MotorLeft.stop(coast);
  MotorRight.stop(coast);
  MotorBeam.setVelocity(100.0, percent);
  MotorBeam.setMaxTorque(100.0, percent);
  MotorLeft.setMaxTorque(100.0, percent);
  MotorRight.setMaxTorque(100.0, percent);
  MotorLeft.setVelocity(100.0, percent);
  MotorRight.setVelocity(100.0, percent);
  startgame = false;
  Brain.Timer.reset();

  TouchLED12.setColor(purple);

  if (touched)
  {
    BrainInertial.setHeading(0.0, degrees);
    BrainInertial.setRotation(0.0, degrees);
    TouchLED12.setColor(orange);
    wait(1, seconds);
    touched = false;
  }
  else
  {
    //========================= Initial ===============================
    TouchLED12.setColor(purple);
    MotorBeam.setVelocity(100.0, percent);
    MotorBeam.setMaxTorque(100.0, percent);
    startgame = false;
    Brain.Timer.reset();
    BrainInertial.setRotation(0.0, degrees);
    MotorBeam.spinFor(reverse, 220, degrees, false);
    int count = 0;

    // ====================== Fist 2 Pins ==============================
    Movemen_v_fb_d(100, 0, 850.0);
    Movemen_v_fb_d(20, 0, 150.0);
    Pneumatic_Pin_Beam.extend(cylinder1);

    //====================== Init Second 2 Pins ========================
    Movemen_v_fb_d(100, 1, 390.0);
    // wait(200, msec);
    // while ((D2.objectDistance(mm) > 800) || (D2.objectDistance(mm) < 700))
    // {
    //   if ((D2.objectDistance(mm) > 800))
    //   {
    //     TouchLED12.setColor(green);
    //     MotorLeft.setVelocity(20, pct);
    //     MotorRight.setVelocity(20, pct);
    //     MotorLeft.spin(reverse);
    //     MotorRight.spin(reverse);
    //   }
    //   else if ((D2.objectDistance(mm) < 700))
    //   {
    //     TouchLED12.setColor(green);
    //     MotorLeft.setVelocity(20, pct);
    //     MotorRight.setVelocity(20, pct);
    //     MotorLeft.spin(forward);
    //     MotorRight.spin(forward);
    //   }
    //   wait(10, msec);
    // }
    // TouchLED12.setColor(purple);
    Arm_Grab_pin = true;
    TurnRight_h_v_m(335, 70, 3);
    wait(200, msec);
    while ((D2.objectDistance(mm) < 430) || (D2.objectDistance(mm) > 440))
    {
      if ((D2.objectDistance(mm) < 430))
      {
        TouchLED12.setColor(green);
        MotorLeft.setVelocity(20, pct);
        MotorRight.setVelocity(20, pct);
        MotorLeft.spin(forward);
        MotorRight.spin(forward);
      }
      else if ((D2.objectDistance(mm) > 440))
      {
        TouchLED12.setColor(green);
        MotorLeft.setVelocity(20, pct);
        MotorRight.setVelocity(20, pct);
        MotorLeft.spin(reverse);
        MotorRight.spin(reverse);
      }
      wait(10, msec);
    }
    TouchLED12.setColor(purple);
    TouchLED12.setColor(purple);
    Movemen_v_fb_d(100, 0, 350.0);
    TurnLeft_h_v_m(310, 50.0, 3);
    Movemen_v_fb_d(100, 0, 300.0);
    Movemen_v_fb_d(20, 0, 125.0);
    Drop_Down_Grab_Up();
    TurnLeft_h_v_m(270, 50.0, 3);
    TouchLED12.setColor(purple);

    // =============== Place Pin on Standoff =======================
    MotorLeft.setVelocity(100, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(reverse);
    MotorRight.spin(reverse);
    count = 0;
    while (true)
    {
      if (D1.objectDistance(inches) > 2.1 && D2.objectDistance(inches) > 2.1)
      {
        MotorLeft.spin(reverse);
        MotorRight.spin(reverse);
      }
      else
      {
        TouchLED12.setColor(green);
        count += 1;
        if (count > 5)
        {
          break;
        }
      }
      wait(20, msec);
    }
    TouchLED12.setColor(purple);
    Place_Standoff();
    Movemen_v_fb_d(100, 0, 150);

    // ========================= U Beam =====================================
    armback = true;
    TurnRight_h_v_m(90, 70, 3);
    MotorLeft.setVelocity(100, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(reverse);
    MotorRight.spin(reverse);
    wait(0.5, seconds);
    while (MotorLeft.velocity(percent) != 0)
    {
      MotorLeft.spin(reverse);
      MotorRight.spin(reverse);
    }
    armfont = true;
    MotorLeft.setVelocity(100, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(reverse);
    MotorRight.spin(reverse);
    while (MotorLeft.velocity(percent) != 0)
    {
      MotorLeft.spin(reverse);
      MotorRight.spin(reverse);
    }

    //===================== Y steak  On Standoff ===============
    Movemen_v_fb_d(100, 0, 150);
    // MotorBeam.spinFor(reverse, 200, degrees, false);
    turnRightLong(280, 100);
    wait(0.5, seconds);
    // TurnRight_h_v_m(273, 100, 3);
    MotorLeft.setVelocity(100, percent);
    wait(100, msec);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(forward);
    MotorRight.spin(forward);
    wait(0.3, seconds);
    while (MotorLeft.velocity(percent) != 0)
    {
      MotorLeft.spin(forward);
      MotorRight.spin(forward);
    }
    wait(0.5, seconds);
    MotorLeft.setVelocity(100, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(reverse);
    MotorRight.spin(reverse);
    MotorBeam.setVelocity(100, percent);
    MotorBeam.spinFor(reverse, 490, degrees, false);
    wait(0.5, sec);
    count = 0;
    while (true)
    {
      if (D1.objectDistance(inches) > 2.1 && D2.objectDistance(inches) > 2.1)
      {
        MotorLeft.spin(reverse);
        MotorRight.spin(reverse);
      }
      else
      {
        TouchLED12.setColor(green);
        count += 1;
        if (count > 5)
        {
          break;
        }
      }
      wait(20, msec);
    }
    wait(0.5, sec);
    TouchLED12.setColor(purple);
    beamoff = true;
    Place_Standoff();

    //----------------------------------------------------------
    //=================== part 2 ===============================
    //----------------------------------------------------------

    //================= grab 2 orangePin =======================
    MotorLeft.setVelocity(100, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(forward);
    MotorRight.spin(forward);
    wait(0.5, sec);
    while (MotorLeft.velocity(percent) > 1)
    {
      MotorLeft.spin(forward);
      MotorRight.spin(forward);
    }
    MotorLeft.stop(brake);
    MotorRight.stop(brake);
    Movemen_v_fb_d(100, 1, 100);
    Pneumatic_Pin_Beam.extend(cylinder1);
    TurnRight_h_v_m(18, 40, 3);
    Movemen_v_fb_d(100, 0, 450);
    TurnLeft_h_v_m(10, 70, 3);
    MotorLeft.setVelocity(80, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(forward);
    MotorRight.spin(forward);
    wait(0.5, seconds);
    while (MotorLeft.velocity(percent) > 1)
    {
      MotorLeft.spin(forward);
      MotorRight.spin(forward);
    }
    MotorLeft.stop();
    wait(0.3, sec);
    MotorRight.stop();
    Movemen_v_fb_d(100, 1, 100);
    Drop_down();
    MotorLeft.setVelocity(70, percent);
    MotorRight.setVelocity(70, percent);
    MotorLeft.spin(forward);
    MotorRight.spin(forward);
    wait(0.5, seconds);
    while (MotorLeft.velocity(percent) > 1)
    {
      MotorLeft.spin(forward);
      MotorRight.spin(forward);
    }
    MotorLeft.stop();
    MotorRight.stop();
    Pneumatic_Pin_Beam.extend(cylinder1);
    Movemen_v_fb_d(100, 1, 100);
    // turnRightLong(115, 100);
    Grab_then_up();
    TurnRight_h_v_m(113, 40, 3);

    //=================== 2 steak =======================
    Go_To_Pin(redPin);
    Movemen_v_fb_d(20, 0, 200);
    Drop_Down_Grab_Up();
    Movemen_v_fb_d(100, 1, 80);
    if (Brain.timer(sec) >= 50)
    {
      Drop_down();
      Movemen_v_fb_d(100, 1, 100);
      Brain.programStop();
    }
    //=================== U Beam ========================
    TurnRight_h_v_m(180, 128, 3);
    // turnRightLong(180, 100);
    MotorLeft.setVelocity(100, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(reverse);
    MotorRight.spin(reverse);
    wait(1, seconds);
    while (MotorLeft.velocity(percent) > 1)
    {
      MotorLeft.spin(reverse);
      MotorRight.spin(reverse);
    }
    Pneumatic_Pin_Beam.extend(cylinder2);
    MotorLeft.stop();
    MotorRight.stop();
    armfont = true;
    Movemen_v_fb_d(100, 0, 230);
    // MotorBeam.spinFor(reverse, 80, degrees, false);

    //====================== Y steak ========================
    MotorLeft.spinFor(forward, 361, degrees, false);
    MotorRight.spinFor(forward, 361, degrees, true);
    MotorBeam.spinFor(reverse, 80, degrees);
    Grab_Beam_up();
    Movemen_v_fb_d(35, 1, 300);
    guide.retract(cylinder1);
    wait(1, sec);
    Place_beam();
    Brain.programStop();
  }
}

int main()
{
  vexcodeInit();
  thread printco(PrintCon);
  thread controarm(controArm);
  TouchLED12.pressed(onevent_TouchLED12_pressed_0);
  take_off();
}