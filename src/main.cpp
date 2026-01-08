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

touchled TouchLED = touchled(PORT6);
pneumatic Pneumatic_Pin_Beam = pneumatic(PORT4);
motor MotorLeft = motor(PORT12, false);
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
void Drop_down_Grab_Up();
void Grab_Beam_up();
void R3F();
void Grab_then_up();
void turnRightLong(double input, double ver);
void turnLeftLong(double input, double ver);
// void Turn_h_v_m_shot_Aj(double heading, double velocity, double momentum);
void turnLeftLong(double input, double ver);
void Turn_h_v_m(double heading, double velocity, double momentum);
void Movemen_v_fb_d(double velocity, int FB, double distance);
void Movemen_v_fb_d_Aj(double velocity, int FB, double distance, double head);
void StopMove_v_fb(int velocity, int FB);
void TurnLeft_h_v_m_v1(double heading, double velocity, double momentum);
void TurnRight_h_v_m_v1(double heading, double velocity, double momentum);
void dis_heading_Align(double velocity);

int Screen_precision = 0, Console_precision = 0, AIVision7_objectIndex = 0, LiftBeam = 0;
bool startgame, beamon, AjON, touched = true, armfont = false, armback = false, Arm_Grab_pin = false, Onthuch = false;
double width, cenX, d_now_go;

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
  MotorBeam.setVelocity(45.0, percent);
  guide.extend(cylinder1);
  wait(100, msec);
  MotorBeam.spinFor(forward, 150.0, degrees, true);
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
  MotorBeam.spinFor(reverse, 50.0, degrees, false);
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

  MotorBeam.setVelocity(100, percent);
  MotorBeam.spinFor(forward, 90, degrees, true);
  wait(0.3, seconds);
  Pneumatic_Pin_Beam.retract(cylinder2);
  wait(0.5, seconds);
  MotorBeam.spinFor(reverse, 60, degrees, false);
  // MotorLeft.setStopping(brake);
  // MotorRight.setStopping(brake);
  // Movemen_v_fb_d(100, 0, 200);
}

// ========== FUNTION MOVE AND TURN ==========

void StopMove_v_fb(int velocity, int FB)
{
  MotorLeft.setVelocity(velocity, percent);
  MotorRight.setVelocity(velocity, percent);
  if (FB == 0)
  {
    MotorRight.spin(forward);
    MotorLeft.spin(forward);
    wait(0.4, seconds);
    while (MotorLeft.velocity(vex::velocityUnits::pct) != 0)
    {
      MotorRight.spin(forward);
      MotorLeft.spin(forward);
    }
    MotorRight.stop();
    MotorLeft.stop();
    return;
  }

  if (FB == 1)
  {
    MotorRight.spin(reverse);
    MotorLeft.spin(reverse);
    wait(0.4, seconds);
    while (MotorLeft.velocity(vex::velocityUnits::pct) != 0)
    {
      MotorRight.spin(reverse);
      MotorLeft.spin(reverse);
    }
    MotorRight.stop();
    MotorLeft.stop();
    return;
  }
}

void Movemen_v_fb_d_Aj(double maxVelocity, int FB, double distance, double head)
{
  double Kp_head = 0.6;
  double minTurn = 5;
  double maxTurn = 25;

  double target = fabs(distance / 0.833);
  MotorLeft.setPosition(0, degrees);
  MotorRight.setPosition(0, degrees);

  while (true)
  {
    double current =
        fabs(MotorLeft.position(degrees) + MotorRight.position(degrees)) / 2.0;
    if (current >= target)
      break;

    double baseSpeed = maxVelocity;

    // --- heading error ---
    double headError = head - BrainInertial.heading();
    if (headError > 180)
      headError -= 360;
    if (headError < -180)
      headError += 360;

    double turn = Kp_head * headError;

    if (fabs(headError) > 2 && fabs(turn) < minTurn)
      turn = (headError > 0) ? minTurn : -minTurn;

    if (turn > maxTurn)
      turn = maxTurn;
    if (turn < -maxTurn)
      turn = -maxTurn;

    if (FB != 0)
      turn = -turn;

    double leftSpeed = baseSpeed - turn;
    double rightSpeed = baseSpeed + turn;

    if (FB == 0)
    {
      MotorLeft.spin(forward, leftSpeed, pct);
      MotorRight.spin(forward, rightSpeed, pct);
    }
    else
    {
      MotorLeft.spin(reverse, leftSpeed, pct);
      MotorRight.spin(reverse, rightSpeed, pct);
    }

    wait(10, msec);
  }

  MotorLeft.stop(brake);
  MotorRight.stop(brake);
}

void Movemen_v_fb_d(double velocity, int FB, double distance)
{
  double K = 1.0;
  MotorLeft.setVelocity((velocity * K), percent);
  MotorRight.setVelocity((velocity * K), percent);
  if (FB == 0)
  {
    MotorLeft.spinFor(forward, (distance / 0.833), degrees, false);
    MotorRight.spinFor(forward, (distance / 0.833), degrees, true);
    while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
    {
      wait(10, msec);
    }
  }
  if (FB == 1)
  {
    MotorLeft.spinFor(reverse, (distance / 0.833), degrees, false);
    MotorRight.spinFor(reverse, (distance / 0.833), degrees, true);
    while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
    {
      wait(10, msec);
    }
  }
}

// double hypotenuse(double a, double b)
// {
//   return sqrt(a * a + b * b);
// }

void dis_heading_Align(double velocity)
{

  double current = BrainInertial.heading();
  double d = 615.95;
  double distance = 0;
  // double Hypotenuse;

  MotorLeft.stop(brake);
  MotorRight.stop(brake);

  if (current <= 45 || current >= 315)
  {
    double theta;
    //---ด้านเอียง

    if (current <= 45)
      theta = current;
    else
      theta = 360 - current; // แปลง 315–360 → 0–45

    //--Degrees to Radians--//--π--////
    double rad = theta * M_PI / 180.0;
    // ความยาวด้านหนึ่งพร้อมมุม (\(\theta \)) ที่เส้นทแยงมุมทำกับด้านนั้น //
    d = 1253 / cos(rad);
    printf("d: %d\n", (int)d);
  }
  double d_now_goO = 0;
  // สูตรพีทาโกรัส
  // Hypotenuse = hypotenuse(d_now_go, 500);
  StopMove_v_fb(100, 0);

  if (current > 3 && current <= 45)
  {
    // Movemen_v_fb_d(100, 0, Hypotenuse);
    StopMove_v_fb(100, 0);
    Movemen_v_fb_d(100, 1, 70);
    TurnRight_h_v_m_v1(90, 80, 4);
    if (d_now_goO < 30)
      d_now_goO = d - 1106;
      printf("d_now_go: %d\n", (int)d_now_goO);
    Movemen_v_fb_d(velocity, 1, d_now_goO);
  }
  else if (current < 357 && current >= 315)
  {
    // Movemen_v_fb_d(100, 0, Hypotenuse);
    StopMove_v_fb(100, 0);
    Movemen_v_fb_d(100, 1, 70);
    TurnRight_h_v_m_v1(90, 80, 4);
    if (d_now_goO < 30)
      d_now_goO = d - 1106;
      printf("d_now_go: %d\n", (int)d_now_goO);
    Movemen_v_fb_d(velocity, 0, d_now_goO);
  }
  else
  {
    // Movemen_v_fb_d(velocity, 0, 500);
    StopMove_v_fb(100, 0);
    Movemen_v_fb_d(100, 1, 70);
    printf("d_now_go: 0-0\n");
    TurnRight_h_v_m_v1(90, 80, 4);
  }
  for (int i = 0; i < 100; i++)
  {
    distance += D2.objectDistance(mm);
  }
  distance = distance / 100;
  double dis_turn = 2470 - distance;
  printf("distanceDis: %d\n", (int)dis_turn);
  printf("distanceTT: %d\n", (int)distance);
  double angle_deg = atan2(200.0, dis_turn) * 180.0 / M_PI;
  // double angle_deg = 95.0 + atan2(200.0, dis_turn);
  printf("dis_turn: %d\n", (int)dis_turn);
  printf("angle_deg: %d\n", (int)angle_deg);
  TurnRight_h_v_m_v1(98 + angle_deg, 40, 2);
}

// void Go_with_hypotenuse(double a, double b)
// {
//   if (b == 0)
//     b = d_now_go;
//   if (a == 0)
//     a = d_now_go;

//   double dis = (491 - a);
//   double dis_go = hypotenuse(dis, b);
//   Movemen_v_fb_d(100, 0, dis_go - 200);
// }

// double ComputeTurnSpeed(double absErr, double maxSpeed)
// {
// const double TURN_MIN = 15;
// double maxErrForScale = 90.0;

// double t = absErr / maxErrForScale;
// if (t > 1.0)
// t = 1.0;

// t = sqrt(t);

// double speed = TURN_MIN + (maxSpeed - TURN_MIN) * t;
// return speed;
// }

double OldComputeTurnSpeed(double absErr, double maxSpeed)
{
  double minSpeed = maxSpeed * 0.2;
  double maxErrForScale = 90.0;

  double t = absErr / maxErrForScale;
  if (t > 1.0)
    t = 1.0;

  // curve: make small errors give much smaller speeds
  t = t * t; // square makes the curve “steeper” near 0

  double speed = minSpeed + (maxSpeed - minSpeed) * t;

  // Cap speed
  if (speed > 90)
    speed = 90;

  return speed;
}

double ComputeTurnSpeed(double absErr, double maxSpeed)
{
  double minSpeed = 15.0;
  double maxErrForScale = 90.0;

  double t = absErr / maxErrForScale;
  if (t > 1.0)
    t = 1.0;

  // curve: make small errors give much smaller speeds
  t = t * t; // square makes the curve “steeper” near 0

  double speed = minSpeed + (maxSpeed - minSpeed) * t;
  return speed;
}

// void TurnRight_h_v_m(double heading, double velocity, double momentum)
// {
//   AjON = true;
//   while (true)
//   {
//     double current = BrainInertial.heading();
//     double error = heading - current; // positive → need left turn
//     double absErr = fabs(error);

//     // ---- 1. Done / inside momentum zone ----
//     if (absErr <= momentum)
//     {
//       MotorLeft.setStopping(brake);
//       MotorRight.setStopping(brake);
//       MotorLeft.stop();
//       MotorRight.stop();

//       while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
//         wait(10, msec);

//       double finalHeading = BrainInertial.heading();
//       double finalErr = fabs(heading - finalHeading);

//       if (finalErr <= momentum)
//       {
//         printf("Final heading (OK): %.2f\n", finalHeading);
//         break;
//       }
//       else
//       {
//         printf("Correcting overshoot... err=%.2f\n", finalErr);
//         Turn_h_v_m(heading, 15, momentum);
//         MotorLeft.stop(brake);
//         MotorRight.stop(brake);

//         wait(100, msec);
//         continue;
//       }
//       AjON = false;
//     }

//     // ---- 2. Slowdown system to prevent sliding ----
//     double turnSpeed = ComputeTurnSpeed(absErr, velocity);
//     // turnSpeed *= 0.9;
//     if (turnSpeed < 10)
//       turnSpeed = 10;

//     MotorLeft.setVelocity(turnSpeed, percent);
//     MotorRight.setVelocity(turnSpeed, percent);

//     // ---- 3. Actual turn left ----
//     MotorLeft.spin(forward);
//     MotorRight.spin(reverse);

//     wait(20, msec);
//     if (fabs(MotorLeft.velocity(pct)) < 10 &&
//         fabs(MotorRight.velocity(pct)) < 10)
//     {
//       turnSpeed += 1;
//     }
//   }

//   wait(0.1, seconds);
// }
//---------------------------
// void TurnRight_h_v_m(double heading, double velocity, double momentum)
// {
// const double TURN_MIN = 15;
// const double TURN_CORRECT = 18;

// while (true)
// {
// double current = BrainInertial.heading();
// double error = heading - current;

// // wrap heading
// if (error > 180)
// error -= 360;
// if (error < -180)
// error += 360;

// double absErr = fabs(error);

// // ---- 1. Done / inside momentum zone ----
// if (absErr <= momentum)
// {
// // ใช้แรงต่ำ ค่อย ๆ creep เข้า
// MotorLeft.spin(forward, TURN_CORRECT, percent);
// MotorRight.spin(reverse, TURN_CORRECT, percent);

// wait(40, msec);

// MotorLeft.stop(brake);
// MotorRight.stop(brake);

// double finalHeading = BrainInertial.heading();
// if (fabs(heading - finalHeading) <= momentum)
// {
// printf("Final heading (OK): %d\n", BrainInertial.heading());
// break;
// }

// continue;
// }

// // ---- 2. Slowdown system ----
// double turnSpeed = ComputeTurnSpeed(absErr, velocity);

// if (turnSpeed < TURN_MIN)
// turnSpeed = TURN_MIN;

// if (turnSpeed > velocity)
// turnSpeed = velocity;

// // ---- 3. Actual turn ----
// if (error > 0)
// {
// MotorLeft.spin(forward, turnSpeed, percent);
// MotorRight.spin(reverse, turnSpeed, percent);
// }
// else
// {
// MotorLeft.spin(reverse, turnSpeed, percent);
// MotorRight.spin(forward, turnSpeed, percent);
// }

// wait(20, msec);
// }

// wait(100, msec);
// }
// //------------------
// void TurnLeft_h_v_m(double heading, double velocity, double momentum)
// {
// const double TURN_MIN = 15;
// const double TURN_CORRECT = 18;

// while (true)
// {
// double current = BrainInertial.heading();
// double error = heading - current;

// // wrap angle
// if (error > 180)
// error -= 360;
// if (error < -180)
// error += 360;

// double absErr = fabs(error);

// // ---- 1. Done / inside momentum zone ----
// if (absErr <= momentum)
// {
// // creep เข้าเป้า แทนการ stop ทันที
// MotorLeft.spin(reverse, TURN_CORRECT, percent);
// MotorRight.spin(forward, TURN_CORRECT, percent);

// wait(40, msec);

// MotorLeft.stop(brake);
// MotorRight.stop(brake);

// double finalHeading = BrainInertial.heading();
// if (fabs(heading - finalHeading) <= momentum)
// {
// printf("Final heading (OK): %d\n", BrainInertial.heading());
// break;
// }
// continue;
// }

// // ---- 2. Slowdown system ----
// double turnSpeed = ComputeTurnSpeed(absErr, velocity);

// if (turnSpeed < TURN_MIN)
// turnSpeed = TURN_MIN;

// if (turnSpeed > velocity)
// turnSpeed = velocity;

// // ---- 3. Actual turn ----
// MotorLeft.spin(reverse, turnSpeed, percent);
// MotorRight.spin(forward, turnSpeed, percent);

// wait(20, msec);

// // (ถ้าจะเช็ค velocity ให้ถูก)
// /*
// if (fabs(MotorLeft.velocity(pct)) < 1 &&
// fabs(MotorRight.velocity(pct)) < 1)
// {
// // optional
// }
// */
// }

// wait(100, msec);
// }
// void TurnLeft_h_v_m(double heading, double velocity, double momentum)
// {
//   AjON = true;
//   while (true)
//   {
//     double current = BrainInertial.heading();
//     double error = heading - current; // negative → need right turn
//     double absErr = fabs(error);

//     if (absErr <= momentum)
//     {
//       MotorLeft.setStopping(brake);
//       MotorRight.setStopping(brake);
//       MotorLeft.stop();
//       MotorRight.stop();

//       while (MotorLeft.velocity(vex::velocityUnits::pct) > 1)
//         wait(10, msec);

//       double finalHeading = BrainInertial.heading();
//       double finalErr = fabs(heading - finalHeading);

//       if (finalErr <= momentum)
//       {
//         printf("Final heading (OK): %.2f\n", finalHeading);
//         break;
//       }
//       else
//       {
//         printf("Correcting overshoot... err=%.2f\n", finalErr);
//         Turn_h_v_m(heading, 15, momentum);
//         MotorLeft.stop(brake);
//         MotorRight.stop(brake);

//         wait(100, msec);
//         continue;
//       }
//       AjON = false;
//     }

//     double turnSpeed = ComputeTurnSpeed(absErr, velocity);
//     if (turnSpeed < 10)
//       turnSpeed = 10;

//     MotorLeft.setVelocity(turnSpeed, percent);
//     MotorRight.setVelocity(turnSpeed, percent);

//     MotorLeft.spin(reverse);
//     MotorRight.spin(forward);

//     wait(20, msec);
//     if (fabs(MotorLeft.velocity(pct) < 1) && fabs(MotorRight.velocity(pct) < 1))
//     {
//       turnSpeed += 1;
//     }
//   }

//   wait(0.1, seconds);
// }

// void turnRightLong(double Target, double ver)
// {
//   MotorLeft.setVelocity(ver, pct);
//   MotorRight.setVelocity(ver, pct);
//   MotorRight.setPosition(0.0, degrees);
//   MotorLeft.setPosition(0.0, degrees);
//   MotorLeft.stop(brake);
//   MotorRight.stop(brake);
//   double dis = 0;

//   if (BrainInertial.heading() > Target)
//   {
//     dis = fabs((360 - BrainInertial.heading()) + Target);
//   }
//   else
//   {
//     dis = fabs(Target - BrainInertial.heading());
//   }

//   dis *= 2;
//   MotorLeft.spinToPosition(dis, degrees, false);
//   MotorRight.spinToPosition(-dis, degrees, true);
//   MotorLeft.stop(brake);
//   MotorRight.stop(brake);
//   AjON = true;
//   Turn_h_v_m(Target, 15, 3);
//   AjON = false;
//   printf("fffffOK\n");
// }

// void turnLeftLong(double Target, double ver)
// {
//   MotorLeft.setVelocity(ver, pct);
//   MotorRight.setVelocity(ver, pct);
//   MotorRight.setPosition(0.0, degrees);
//   MotorLeft.setPosition(0.0, degrees);
//   MotorLeft.stop(brake);
//   MotorRight.stop(brake);
//   double dis = 0;

//   if (BrainInertial.heading() < Target)
//   {
//     dis = fabs((360 - Target) + BrainInertial.heading());
//   }
//   else
//   {
//     dis = fabs(Target - BrainInertial.heading());
//   }
//   dis *= 2;
//   MotorLeft.spinToPosition(-dis, degrees, false);
//   MotorRight.spinToPosition(dis, degrees, true);
//   MotorLeft.stop(brake);
//   MotorRight.stop(brake);
//   wait(250, msec);
//   AjON = true;
//   Turn_h_v_m(Target, 15, 3);
//   AjON = false;
// }

// void Turn_h_v_m(double heading, double velocity, double momentum)
// {
//   int count = 0;
//   AjON = true;
//   while (!(heading == BrainInertial.heading()))
//   {
//     double headingnow = BrainInertial.heading();
//     if ((heading - momentum) > headingnow)
//     {
//       MotorLeft.setVelocity(velocity, percent);
//       MotorRight.setVelocity(velocity, percent);
//       MotorLeft.spin(forward);
//       MotorRight.spin(reverse);
//     }
//     else if ((heading + momentum) < headingnow)
//     {
//       MotorLeft.setVelocity(velocity, percent);
//       MotorRight.setVelocity(velocity, percent);
//       MotorLeft.spin(reverse);
//       MotorRight.spin(forward);
//     }
//     else
//     {
//       if (fabs(headingnow - heading) <= momentum)
//       {
//         count++;
//         MotorLeft.stop(brake);
//         MotorRight.stop(brake);
//         if (count > 5)
//         {
//           MotorLeft.stop(brake);
//           MotorRight.stop(brake);
//           break;
//         }
//       }
//       else
//       {
//         count = 0;
//       }
//     }
//     wait(20, msec);
//   }
//   AjON = false;
// }

void TurnToAngle(double targetHeading, double maxVelocity, double momentum)
{
  double Kp = 1.2;  // แรงหมุน
  double Kd = 0.85; // ลดส่าย
  double minSpeed = 15;

  double lastError = 0;
  int count = 0;
  AjON = true;

  while (true)
  {
    double heading = BrainInertial.heading();

    // ===== คำนวณ error แบบสั้นที่สุด =====
    double error = targetHeading - heading;
    if (error > 180)
      error -= 360;
    if (error < -180)
      error += 360;

    // ===== ถึงเป้า =====
    if (fabs(error) <= momentum)
    {
      count++;
      MotorLeft.stop(brake);
      MotorRight.stop(brake);
      if (count > 10)
        break;
    }
    else
    {
      count = 0;
    }

    // ===== PD =====
    double derivative = error - lastError;
    double output = Kp * error + Kd * derivative;

    // จำกัดความเร็ว
    if (output > maxVelocity)
      output = maxVelocity;
    if (output < -maxVelocity)
      output = -maxVelocity;

    // ความเร็วต่ำสุด
    if (fabs(output) < minSpeed)
      output = (output > 0) ? minSpeed : -minSpeed;

    // ===== หมุน =====
    MotorLeft.spin(forward, output, pct);
    MotorRight.spin(reverse, output, pct);

    lastError = error;
    wait(10, msec);
  }
  AjON = false;
  MotorLeft.stop(brake);
  MotorRight.stop(brake);
}

void TurnLeft_h_v_m_v1(double heading, double velocity, double momentum)
{
  AjON = true;
  while (true)
  {
    double current = BrainInertial.heading();
    double error = heading - current; // positive → need left turn
    double absErr = fabs(error);

    if (absErr <= momentum)
    {
      MotorLeft.setStopping(brake);
      MotorRight.setStopping(brake);
      MotorLeft.stop();
      MotorRight.stop();

      while (fabs(MotorLeft.velocity(vex::velocityUnits::pct)) > 1 ||
             fabs(MotorRight.velocity(vex::velocityUnits::pct)) > 1)
        wait(10, msec);

      MotorLeft.setStopping(coast);
      MotorRight.setStopping(coast);
      printf("Final heading: %.2f\n", BrainInertial.heading());
      break;
    }

    double turnSpeed = OldComputeTurnSpeed(absErr, velocity);
    MotorLeft.setVelocity(turnSpeed, percent);
    MotorRight.setVelocity(turnSpeed, percent);

    // ✅ actual turn left
    MotorLeft.spin(reverse);
    MotorRight.spin(forward);

    wait(20, msec);
  }
  AjON = false;
  wait(0.1, seconds);
}

void TurnRight_h_v_m_v1(double heading, double velocity, double momentum)
{
  AjON = true;
  while (true)
  {
    double current = BrainInertial.heading();
    double error = heading - current; // positive → need left turn
    double absErr = fabs(error);

    if (absErr <= momentum)
    {
      MotorLeft.setStopping(brake);
      MotorRight.setStopping(brake);
      MotorLeft.stop();
      MotorRight.stop();

      while (fabs(MotorLeft.velocity(vex::velocityUnits::pct)) > 1 ||
             fabs(MotorRight.velocity(vex::velocityUnits::pct)) > 1)
        wait(10, msec);

      MotorLeft.setStopping(coast);
      MotorRight.setStopping(coast);
      printf("Final heading: %.2f\n", BrainInertial.heading());
      break;
    }

    double turnSpeed = OldComputeTurnSpeed(absErr, velocity);
    MotorLeft.setVelocity(turnSpeed, percent);
    MotorRight.setVelocity(turnSpeed, percent);

    // ✅ actual turn left
    MotorLeft.spin(forward);
    MotorRight.spin(reverse);

    wait(20, msec);
  }
  AjON = false;
  wait(0.1, seconds);
}

// ========== FUNTION PIN ==========

void Grab_then_up()
{
  MotorPin.resetPosition();
  MotorPin.setPosition(0.0, degrees);
  MotorPin.setStopping(hold);
  // MotorPin.spinFor(forward, 10.0, degrees, true);
  Pneumatic_Pin_Beam.extend(cylinder1);
  wait(0.3, seconds);
  // MotorPin.spinFor(forward, 180.0, degrees, false);
  MotorPin.spinToPosition(180.0, degrees, false);
  wait(0.2, seconds);
  guide.extend(cylinder2);
}

void Drop_down()
{
  MotorPin.resetPosition();
  MotorLeft.setStopping(hold);
  MotorRight.setStopping(hold);
  wait(0.35, seconds);
  // MotorPin.setStopping(coast);
  MotorPin.setVelocity(100, percent);
  // MotorPin.spinFor(reverse, 220.0, degrees, false);
  MotorPin.spinFor(reverse, 50.0, degrees, false);
  wait(0.1, seconds);
  guide.retract(cylinder2);
  MotorPin.spinFor(reverse, 170.0, degrees, false);
  wait(250, msec);
  MotorPin.setVelocity(100, percent);
  MotorPin.spin(reverse);
  while (MotorPin.velocity(vex::velocityUnits::pct) > 1)
  {
    MotorPin.spin(reverse);
    wait(100, msec);
  }
  // MotorPin.spinFor(forward, 155.0, degrees, false);
  Pneumatic_Pin_Beam.retract(cylinder1);
  MotorPin.setStopping(coast);
  MotorPin.stop(coast);
  // wait(0.3, seconds);
  MotorLeft.setStopping(coast);
  MotorRight.setStopping(coast);
  MotorPin.setPosition(0.0, degrees);
}

void Drop_down_Grab_Up_mod()
{
  // stop = true;
  // wait(0.1, seconds);
  // stop = false;

  MotorPin.resetPosition();
  MotorPin.setPosition(0.0, degrees);
  MotorPin.setStopping(hold);
  // MotorPin.spinFor(reverse, 50.0, degrees, false);
  MotorPin.spinToPosition(-50.0, degrees, false);
  wait(0.1, seconds);
  guide.retract(cylinder2);
  MotorPin.spinToPosition(-170.0, degrees, false);
  // MotorPin.spinFor(reverse, 170.0, degrees, false);
  // stop = true;
  wait(0.25, seconds);
  Pneumatic_Pin_Beam.retract(cylinder1);
  while (MotorPin.velocity(vex::velocityUnits::pct) > 1)
  {
    MotorPin.spin(reverse);
  }
  wait(0.2, seconds);
  MotorLeft.setVelocity(50.0, percent);
  MotorRight.setVelocity(50.0, percent);
  MotorLeft.spinFor(forward, 120.0, degrees, false);
  MotorRight.spinFor(forward, 120.0, degrees, false);
  wait(0.3, seconds);
  Pneumatic_Pin_Beam.extend(cylinder1);
  wait(0.3, seconds);
  MotorPin.resetPosition();
  // MotorPin.spinFor(forward, 220.0, degrees, false);
  MotorPin.spinToPosition(215.0, degrees, false);
  // stop = false;
  wait(0.2, seconds);
  guide.extend(cylinder2);
  // isBusy = false;
}

void Drop_down_Grab_Up()
{

  wait(0.1, seconds);
  MotorPin.resetPosition();
  MotorPin.setPosition(0.0, degrees);
  MotorPin.setStopping(hold);
  // MotorPin.spinFor(reverse, 50.0, degrees, false);
  MotorPin.spinToPosition(-50.0, degrees, false);
  wait(0.1, seconds);
  guide.retract(cylinder2);
  MotorPin.spinToPosition(-170.0, degrees, false);
  // MotorPin.spinFor(reverse, 170.0, degrees, false);
  wait(0.25, seconds);
  Pneumatic_Pin_Beam.retract(cylinder1);
  while (MotorPin.velocity(vex::velocityUnits::pct) > 1)
  {
    MotorPin.spin(reverse);
    wait(1, msec);
  }
  MotorLeft.setVelocity(50.0, percent);
  MotorRight.setVelocity(50.0, percent);
  MotorLeft.spinFor(forward, 500.0, degrees, false);
  MotorRight.spinFor(forward, 500.0, degrees, false);
  wait(0.2, seconds);
  Pneumatic_Pin_Beam.extend(cylinder1);
  wait(0.3, seconds);
  MotorLeft.stop(brake);
  MotorRight.stop(brake);
  MotorPin.resetPosition();
  // MotorPin.spinFor(forward, 220.0, degrees, false);
  MotorPin.spinToPosition(215.0, degrees, false);
  wait(0.2, seconds);
  guide.extend(cylinder2);
}

void R3F()
{
  guide.retract(cylinder2);
  Pneumatic_Pin_Beam.extend(cylinder2);
  MotorPin.spinFor(forward, 700.0, degrees, false);
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
  const double targetWidth = 115.0; // when object is this big, we stop
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

void AjpinRight(int targetIDLeft)
{
  // int count = 0;
  while (true)
  {
    TouchLED.setColor(orange);

    AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

    double bestDist = 999;
    double offset = 0;

    for (int i = 0; i < AIVision7.objectCount; i++)
    {
      if (AIVision7.objects[i].id == targetIDLeft)
      {
        double cx = AIVision7.objects[i].centerX;
        double d = abs(cx - 255);

        if (d < bestDist)
        {
          bestDist = d;
          offset = cx - 255;
        }
      }
    }

    if (bestDist < 10)
    {
      MotorLeft.stop(brake);
      MotorRight.stop(brake);
      break;
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
    TouchLED.setColor(blue_green);
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
  TouchLED.setColor(red);
  Brain.Screen.setCursor(1, 1);
  guide.retract(cylinder2);
  Drop_down();
  Drop_down_beam();
  MotorBeam.spinFor(reverse, 520.0, degrees);
  // wait(1, seconds);
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
      guide.retract(cylinder2);
      R3F();
      MotorPin.setStopping(coast);
      MotorPin.spinFor(reverse, 700.0, degrees, false);
      wait(1000, msec);
      MotorBeam.spinFor(reverse, 200, degrees, false);
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
      MotorPin.spinFor(forward, 180.0, degrees, true);
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
    if (startgame)
    {
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("start");
    }
    else
    {
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("D2: %.2f", D2.objectDistance(mm));
    }
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("%.2f", BrainInertial.heading(degrees));

    int h = BrainInertial.heading();

    printf("Heading: %.d deg\n", h);
    wait(50, msec);
  }
  return 0;
}

int Status_light()
{
  while (Onthuch)
  {
    if (AjON)
      TouchLED.setColor(green);
    wait(100, msec);
    if (touched)
      TouchLED.setColor(orange);
    else
      TouchLED.setColor(purple);
    wait(25, msec);
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
// // int count = 0;
// while (true)
// {
// TouchLED.setColor(orange);

// AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

// double bestDist = 999;
// double offset = 0;

// for (int i = 0; i < AIVision7.objectCount; i++)
// {
// if (AIVision7.objects[i].id == targetIDLeft)
// {
// double cx = AIVision7.objects[i].centerX;
// double d = abs(cx - 80);

// if (d < bestDist)
// {
// bestDist = d;
// offset = cx - 80;
// }
// }
// }

// if (bestDist < 10)
// {
// MotorLeft.stop(brake);
// MotorRight.stop(brake);
// break;
// }
// else
// {
// MotorLeft.setVelocity(15, percent);
// MotorRight.setVelocity(15, percent);
// if (offset > 0)
// {
// MotorLeft.spin(forward);
// MotorRight.stop();
// }
// else if (offset < 0)
// {
// MotorLeft.stop();
// MotorRight.spin(forward);
// }
// }

// wait(10, msec);
// TouchLED.setColor(blue_green);
// }
// }

// void Ajpinright(int targetIDRight)
// {
// int count = 0;
// while (true)
// {
// TouchLED.setColor(blue);

// AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

// double bestDist = 999;
// double offset = 0;

// for (int i = 0; i < AIVision7.objectCount; i++)
// {
// if (AIVision7.objects[i].id == targetIDRight)
// {
// double cx = AIVision7.objects[i].centerX;
// double d = abs(cx - 210);

// if (d < bestDist)
// {
// bestDist = d;
// offset = cx - 210;
// }
// }
// }

// if (bestDist < 10)
// {
// count++;
// if (count == 2)
// {
// MotorLeft.stop(brake);
// MotorRight.stop(brake);
// break;
// }
// }
// else
// {
// MotorLeft.setVelocity(15, percent);
// MotorRight.setVelocity(15, percent);
// if (offset > 0)
// {
// MotorLeft.spin(forward);
// MotorRight.stop();
// }
// else if (offset < 0)
// {
// MotorLeft.stop();
// MotorRight.spin(forward);
// }
// }

// wait(10, msec);
// TouchLED.setColor(blue_green);
// }
// }
// void AjpinFind(int targetID)
// {
// int count = 0;
// while (true)
// {
// TouchLED.setColor(red);

// AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

// double bestDist = 999;
// double offset = 0;

// for (int i = 0; i < AIVision7.objectCount; i++)
// {
// if (AIVision7.objects[i].id == targetID)
// {
// double cx = AIVision7.objects[i].centerX;
// double d = abs(cx - 160);

// if (d < bestDist)
// {
// bestDist = d;
// offset = cx - 160;
// }
// }
// }

// if (bestDist < 30)
// {
// count++;
// if (count == 2)
// {
// MotorLeft.stop(brake);
// MotorRight.stop(brake);
// break;
// }
// }
// else
// {
// MotorLeft.setVelocity(30, percent);
// MotorRight.setVelocity(30, percent);

// if (offset > 0)
// {
// MotorLeft.spin(forward);
// MotorRight.stop();
// }
// else if (offset < 0)
// {
// MotorLeft.stop();
// MotorRight.spin(forward);
// }
// }

// wait(10, msec);
// TouchLED.setColor(blue_green);
// }
// }

// void FindBeam(int targetID)
// {
// int count = 0;
// while (true)
// {
// TouchLED.setColor(purple);

// AIVision7.takeSnapshot(aivision::ALL_AIOBJS);

// double bestY = -1;
// double offset = 0;

// for (int i = 0; i < AIVision7.objectCount; i++)
// {
// if (AIVision7.objects[i].id == targetID)
// {
// double cy = AIVision7.objects[i].centerY;
// double d = abs(cy - 160);

// if (d < bestY)
// {
// bestY = d;
// offset = cy - 160;
// }
// }
// }

// if (bestY < 30)
// {
// count++;
// if (count == 2)
// {
// MotorLeft.stop(brake);
// MotorRight.stop(brake);
// break;
// }
// }
// else
// {
// MotorLeft.setVelocity(30, percent);
// MotorRight.setVelocity(30, percent);

// if (offset > 0)
// {
// MotorLeft.spin(forward);
// MotorRight.stop();
// }
// else if (offset < 0)
// {
// MotorLeft.stop();
// MotorRight.spin(forward);
// }
// }

// wait(10, msec);
// TouchLED.setColor(blue_green);
// }
// }

// =================== TOUCH LED EVENT ===================

void Autonomous()
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

  TouchLED.setColor(purple);

  if (touched)
  {
    BrainInertial.setHeading(0.0, degrees);
    BrainInertial.setRotation(0.0, degrees);
    Onthuch = true;
    wait(1, seconds);
    touched = false;
  }
  else
  {
    //========================= Initial ===============================
    TouchLED.setColor(purple);
    MotorBeam.setVelocity(100.0, percent);
    MotorBeam.setMaxTorque(100.0, percent);
    startgame = true;
    Brain.Timer.reset();
    BrainInertial.setRotation(0.0, degrees);
    // MotorBeam.spinFor(reverse, 220, degrees, false);
    // double dis = 0;

    // ====================== Fist 2 Pins ==============================
    Movemen_v_fb_d(100, 0, 900.0);
    Movemen_v_fb_d(20, 0, 150.0);
    Pneumatic_Pin_Beam.extend(cylinder1);

    //====================== Init Second 2 Pins ========================
    Movemen_v_fb_d(100, 1, 460);

    Arm_Grab_pin = true;
    TurnRight_h_v_m_v1(59, 70, 3);
    Movemen_v_fb_d(100, 0, 765);
    MotorLeft.spin(forward, 40, pct);
    MotorRight.spin(forward, 30, pct);
    wait(500, msec);
    while (MotorLeft.velocity(percent) > 9)
    {
      double speedf = MotorLeft.velocity(percent);
      MotorLeft.setVelocity(speedf - 10, percent);
      double speedr = MotorRight.velocity(percent);
      MotorRight.setVelocity(speedr - 10, percent);
      MotorLeft.spin(forward);
      MotorRight.spin(forward);
      wait(50, msec);
    }
    MotorLeft.stop(coast);
    MotorRight.stop(coast);
    Drop_down_Grab_Up();
    Movemen_v_fb_d(100, 1, 263);
    TurnLeft_h_v_m_v1(0, 80.0, 4);

    // =============== Place Pin on Standoff =======================
    StopMove_v_fb(100, 1);

    MotorLeft.stop(hold);
    MotorRight.stop(hold);
    Place_Standoff();
    Movemen_v_fb_d(100, 0, 100);

    // // ========================= U Beam =====================================
    armback = true;
    TurnLeft_h_v_m_v1(183, 80, 3);
    MotorRight.setVelocity(100, percent);
    MotorLeft.setVelocity(100, percent);
    MotorRight.spin(reverse);
    MotorLeft.spin(reverse);
    wait(0.4, seconds);
    while (MotorLeft.velocity(vex::velocityUnits::pct) != 0)
    {
      MotorRight.spin(reverse);
      MotorLeft.spin(reverse);
    }
    wait(0.2, seconds);
    Pneumatic_Pin_Beam.extend(cylinder1);
    wait(0.15, seconds);
    MotorLeft.stop();
    MotorRight.stop();
    armfont = true;
    //===================== Y stack On Standoff ===============
    MotorLeft.setVelocity(70, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(forward);
    MotorRight.spin(forward);
    wait(800, msec);
    MotorLeft.stop(brake);
    MotorRight.stop(brake);
    TurnLeft_h_v_m_v1(0, 70, 3);

    MotorBeam.setVelocity(100, percent);
    MotorBeam.spinFor(reverse, 290, degrees, false);
    MotorLeft.setVelocity(80, percent);
    MotorRight.setVelocity(100, percent);
    MotorLeft.spin(reverse);
    MotorRight.spin(reverse);
    wait(0.35, seconds);
    while (MotorLeft.velocity(percent) != 0)
    {
      MotorLeft.spin(reverse);
      MotorRight.spin(reverse);
      wait(50, msec);
    }
    wait(0.3, sec);
    Place_Standoff();
    MotorLeft.stop(coast);
    MotorRight.stop(coast);
    // ====================== PART TWO ==================================
    // ====================== First 2 Pins ==============================
    printf("===================================================================== \n");
    printf("======================== PART TWO START ============================= \n");
    printf("===================================================================== \n");
    // Movemen_v_fb_d(100, 0, 300);
    // MotorLeft.setStopping(brake);
    // MotorRight.setStopping(brake);
    // TurnRight_h_v_m_v1(90, 80, 4);
    dis_heading_Align(100);

    // Pneumatic_Pin_Beam.extend(cylinder1);
    Arm_Grab_pin = true;
    wait(0.1, seconds);
    armback = true;
    // MotorBeam.setStopping(coast);
    // MotorLeft.setVelocity(100, percent);
    // MotorRight.setVelocity(100, percent);
    // Pneumatic_Pin_Beam.extend(cylinder1);
    // MotorLeft.spin(forward);
    // MotorRight.spin(forward);
    // wait(0.5, seconds);
    // Pneumatic_Pin_Beam.retract(cylinder1);
    // MotorLeft.spin(forward);
    // MotorRight.spin(forward);
    // wait(0.7, seconds);
    // MotorLeft.setVelocity(100, percent);
    // MotorRight.setVelocity(54, percent);
    // MotorLeft.spin(forward);
    // MotorRight.spin(forward);
    // wait(0.9, seconds);
    // MotorLeft.setVelocity(10, percent);
    // MotorRight.setVelocity(100, percent);
    // MotorLeft.spin(forward);
    // MotorRight.spin(forward);
    // wait(0.3, seconds);
    // MotorLeft.setStopping(hold);
    // MotorRight.setStopping(hold);
    // MotorLeft.setVelocity(80, percent);
    // MotorRight.setVelocity(80, percent);
    // MotorRight.spin(forward);
    // MotorLeft.spin(forward);
    // wait(0.5, seconds);
    // while (MotorLeft.velocity(vex::velocityUnits::pct) != 0)
    // {
    //   MotorRight.spin(forward);
    //   MotorLeft.spin(forward);
    // }
    // wait(0.5, seconds);
    // MotorRight.stop();
    // MotorLeft.stop();
    // Pneumatic_Pin_Beam.extend(cylinder1);
    // Pneumatic_Pin_Beam.retract(cylinder2);
    // wait(0.2, seconds);
    // MotorLeft.setStopping(brake);
    // MotorRight.setStopping(brake);
    // // // ====================== Second 2 Pins ==============================
    // MotorPin.spinFor(forward, 180.0, degrees, false);
    // Movemen_v_fb_d(100, 1, 100);
    // guide.extend(cylinder2);
    // TurnRight_h_v_m_v1(202, 80, 4);
    // Movemen_v_fb_d(100, 0, 350);
    // Movemen_v_fb_d(40, 0, 150);
    // Drop_down_Grab_Up_mod();
    // wait(0.2, seconds);
    // // // ====================== U Beam ==============================
    // Movemen_v_fb_d(100, 1, 180);
    // TurnRight_h_v_m_v1(290, 80, 4);
    // StopMove_v_fb(100, 1);
    // Pneumatic_Pin_Beam.extend(cylinder2);
    // MotorBeam.spinFor(reverse, 50, degrees, false);
    // Movemen_v_fb_d(100, 0, 100);
    // MotorLeft.setVelocity(100, percent);
    // MotorRight.setVelocity(70, percent);
    // MotorLeft.spin(reverse);
    // MotorRight.spin(reverse);
    // wait(0.5, seconds);
    // StopMove_v_fb(50, 1);
    // MotorBeam.setStopping(hold);
    // armfont = true;
    // MotorBeam.setStopping(hold);
    // MotorBeam.setMaxTorque(100, percent);
    // MotorBeam.setVelocity(100, percent);
    // Movemen_v_fb_d(100, 0, 185);
    // wait(1.5, seconds);
    // // // // ====================== Y Beam ==============================
    // MotorBeam.spinFor(reverse, 150, degrees, false);
    // TurnLeft_h_v_m_v1(181, 80, 4);
    // Movemen_v_fb_d(40, 0, 270);
    // Grab_then_up();
    // Movemen_v_fb_d(40, 0, 220);
    // Drop_down();
    // Movemen_v_fb_d(100, 1, 150);
    // MotorBeam.spinFor(reverse, 150, degrees, false);
    // TurnLeft_h_v_m_v1(10, 80, 6);
    // StopMove_v_fb(60, 1);
    // Movemen_v_fb_d(50, 0, 100);
    // Place_beam();

    // ---------------------------------------------- part 2 New
    // TurnRight_h_v_m_v1(140, 80, 3);
    // Go_with_hypotenuse(0, 762);
    // Movemen_v_fb_d(30, 0, 200.0);
    // Pneumatic_Pin_Beam.extend(cylinder1);
    // Movemen_v_fb_d(100, 1, 500);
    StopMove_v_fb(100, 0);
    MotorRight.stop(coast);
    MotorLeft.stop(coast);
    MotorRight.spin(forward, 100, pct);
    wait(0.25, seconds);
    while (MotorRight.velocity(vex::velocityUnits::pct) != 0)
    {
      MotorRight.spin(forward);
      MotorLeft.spin(forward);
    }
    MotorRight.stop(coast);
    MotorLeft.stop(coast);
    Movemen_v_fb_d(100, 1, 100);
    Drop_down();
    wait(0.25, seconds);
    // MotorLeft.setVelocity(100, percent);
    // MotorRight.setVelocity(100, percent);
    // Pneumatic_Pin_Beam.extend(cylinder1);
    // MotorLeft.spin(forward);
    // MotorRight.spin(forward);
    // wait(0.2, seconds);
    // while (MotorLeft.velocity(vex::velocityUnits::pct) != 0)
    // {
    //   MotorRight.spin(forward);
    //   MotorLeft.spin(forward);
    // }
    StopMove_v_fb(100, 0);
    Pneumatic_Pin_Beam.extend(cylinder1);
    Movemen_v_fb_d(100, 1, 100);
    Arm_Grab_pin = true;
    TurnRight_h_v_m_v1(210, 80, 4);
    Movemen_v_fb_d(100, 0, 350);
    Movemen_v_fb_d(40, 0, 180);
    Drop_down_Grab_Up_mod();
    wait(0.2, seconds);
    // // ====================== U Beam ==============================
    Movemen_v_fb_d(100, 1, 230);
    TurnRight_h_v_m_v1(285, 80, 4);
    StopMove_v_fb(100, 1);
    Pneumatic_Pin_Beam.extend(cylinder2);
    MotorBeam.spinFor(reverse, 50, degrees, false);
    Movemen_v_fb_d(100, 0, 100);
    armfont = true;
    MotorBeam.setMaxTorque(100, percent);
    MotorBeam.setVelocity(100, percent);
    Movemen_v_fb_d(100, 0, 100);
    wait(1.5, seconds);
    // // // ====================== Y Beam ==============================
    MotorBeam.spinFor(reverse, 50, degrees, false);
    TurnLeft_h_v_m_v1(181, 80, 4);
    Movemen_v_fb_d(40, 0, 150);
    Movemen_v_fb_d(20, 0, 100);
    Grab_then_up();
    wait(0.1, seconds);
    StopMove_v_fb(60, 0);
    Movemen_v_fb_d(100, 1, 50);
    Drop_down();
    MotorLeft.spin(reverse, 100, pct);
    MotorRight.spin(reverse, 40, pct);
    wait(0.5, seconds);
    MotorBeam.spinFor(reverse, 50, degrees, false);
    TurnLeft_h_v_m_v1(15, 80, 4);
    StopMove_v_fb(60, 1);
    MotorRight.spin(reverse, 100, pct);
    wait(0.3, seconds);
    Movemen_v_fb_d(50, 0, 100);
    Place_beam();
    MotorLeft.stop(coast);
    MotorRight.stop(coast);
  }
}

int main()
{
  vexcodeInit();
  thread printco(PrintCon);
  thread controarm(controArm);
  thread statuslight(Status_light);
  TouchLED.pressed(Autonomous);
  take_off();
}