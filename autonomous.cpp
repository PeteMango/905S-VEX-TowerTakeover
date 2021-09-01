#include "main.h"
#include "Declarations.hpp"
#include "Functions.hpp"
#include "okapi/api.hpp"

using namespace okapi;
auto chassis = ChassisControllerBuilder()
    .withMotors({17,13}, {5,16})
    .withDimensions(AbstractMotor::gearset::blue, {{2.75_in, 8.5_in}, imev5BlueTPR})
    .withSensors(
        {'A', 'B', true},
        {'C', 'D', true}
    )
    .build();

double getDistanceTraveled (double ticks) {
  return (ticks / 21.9329);
}
double convertCMToTicks (double cm) {
  return 21.9329 * cm;
}
double limitSpd (double curSpd) {
  if(curSpd > 0) {
    curSpd = std::min(50.0, curSpd);
  }
  else {
    curSpd = std::max(-50.0, curSpd);
  }
  return curSpd;
}
void selfCheck () {
  L.move(-127); R.move(-127);
  delay(300);
  L.move(127); R.move(127);
  delay(700);
  L.move(0); R.move(0);
}
void deploy() {
  A.move(127);
  delay(1300);
  A.move(-127);
  delay(1300);
}
void stack () {
  T.move(85);
  delay(2600);
}
void moveTrayBack () {
  T.move(-127);
  delay(2000);
  T.move(0);
}

void skillStack () {
  while(PT.get_value() < 1350) {
    T.move_velocity(127);
  }
  while(PT.get_value() > 1349 && PT.get_value() < 2200){
    T.move_velocity(40);
  }
  pros::delay(300);
  T.move_velocity(0);
  T.set_brake_mode(MOTOR_BRAKE_HOLD);
  pros::delay(20);
}


void driveTurn (double Dst) {
  LT.reset(); RT.reset();
  double kP = 0.5, kD = 0.009, kI = 1.2, E, prevE, I, D, P, Pwr, LS, RS, T;
    do {
      LS = LT.get_value(); RS = RT.get_value(); T = (getDistanceTraveled(LS)-getDistanceTraveled(RS))*4.0;

      E = Dst - T;
      I += E;

      if(E == 0 || std::abs(E) > std::abs(Dst)) {
        I = 0;
      }

      D = E - prevE;
      prevE = E;

      pros::lcd::print(0, "%.2f %.2f %.2f", LS, RS, T);

      Pwr = E*kP + I*kI + D*kD;
      Pwr = limitSpd(Pwr);

      LF.move(Pwr); RF.move(-Pwr);
      LB.move(Pwr); RB.move(-Pwr);
      pros::delay(20);
    } while(abs(E) >= 5);
    LT.reset(); RT.reset();
    LF.move_velocity(0); RF.move_velocity(0);
    LB.move_velocity(0); RB.move_velocity(0);
    LF.set_brake_mode(MOTOR_BRAKE_HOLD); RF.set_brake_mode(MOTOR_BRAKE_HOLD);
    LB.set_brake_mode(MOTOR_BRAKE_HOLD); RB.set_brake_mode(MOTOR_BRAKE_HOLD);
}
void matchDeploy() {
  A.move(127);
  delay(400);
  A.move(-127);
  delay(400);
  A.move(0);
}
void driveDistance (double Dst) {
  LT.reset(); RT.reset();
  Dst = convertCMToTicks(Dst);
  double kP = 0.5, kD = 0.09, kI = 1.2, E, prevE, I, D, P, Pwr;
    do {

      E = Dst - (LT.get_value()+RT.get_value())/2;

      I += E;

      if(E == 0 || std::abs(E) > std::abs(Dst)) {
        I = 0;
      }

      D = E - prevE;
      prevE = E;

      Pwr = E*kP + I*kI + D*kD;
      Pwr = limitSpd(Pwr);

      LF.move(Pwr); RF.move(Pwr);
      LB.move(Pwr); RB.move(Pwr);

      pros::delay(20);
    } while(abs(E) >= 30);
    LT.reset(); RT.reset();
    LF.move_velocity(0); RF.move_velocity(0);
    LB.move_velocity(0); RB.move_velocity(0);
    LF.set_brake_mode(MOTOR_BRAKE_HOLD); RF.set_brake_mode(MOTOR_BRAKE_HOLD);
    LB.set_brake_mode(MOTOR_BRAKE_HOLD); RB.set_brake_mode(MOTOR_BRAKE_HOLD);
}

void delay(int time) {
  int target = pros::millis() + time;

  while(true)
  {
    if(pros::millis() >= target)
    {
      break;
    }
  }
}
void spinIntake () {
  L.move(127);
  R.move(127);
}
void spinOutake () {
  L.move(-127);
  R.move(-127);
}
void spinOutakeSlow () {
  L.move(-100);
  R.move(-100);
}

void stopMovement() {
  LF.move_velocity(0); RF.move_velocity(0);
  LB.move_velocity(0); RB.move_velocity(0);

  A.move_velocity(0); T.move_velocity(0);

  L.move_velocity(0); R.move_velocity(0);
}

void BlueS () {
  spinIntake();
  chassis->setMaxVelocity(180);
  chassis->moveDistance(140_cm);
  stopMovement();
  chassis->waitUntilSettled();
  chassis->turnAngle(25_deg);
  chassis->moveDistance(40_cm);
  chassis->turnAngle(-38_deg);
  chassis->moveDistance(30_cm);
  chassis->moveDistance(-20_cm);
  chassis->turnAngle(-250_deg);
  chassis->setMaxVelocity(300);
  chassis->moveDistance(115_cm);
  L.move(-70); R.move(-70);
  delay(400);
  L.move(0); R.move(0);
  LF.move(50); RF.move(50);
  LB.move(50); RB.move(50);
  stack();
  LF.move(0); RF.move(0);
  LB.move(0); RB.move(0);
  L.move(-127); R.move(-127);
  LF.move(-50); RF.move(-50);
  LB.move(-50); RB.move(-50);
  delay(500);
  LF.move(0); RF.move(0);
  LB.move(0); RB.move(0);
}
void RedS () {
  spinIntake();
  chassis->setMaxVelocity(200);
  chassis->moveDistance(140_cm);
  chassis->turnAngle(-25_deg);
  chassis->moveDistance(40_cm);
  chassis->turnAngle(38_deg);
  chassis->moveDistance(30_cm);
  chassis->moveDistance(-20_cm);
  chassis->turnAngle(240_deg);
  chassis->setMaxVelocity(300);
  chassis->moveDistance(115_cm);
  L.move(-70); R.move(-70);
  delay(400);
  L.move(0); R.move(0);
  LF.move(50); RF.move(50);
  LB.move(50); RB.move(50);
  stack();
  LF.move(0); RF.move(0);
  LB.move(0); RB.move(0);
  L.move(-127); R.move(-127);
  LF.move(-50); RF.move(-50);
  LB.move(-50); RB.move(-50);
  delay(500);
  LF.move(0); RF.move(0);
  LB.move(0); RB.move(0);
}
void BlueL () {
  spinIntake();
  chassis->setMaxVelocity(220);
  chassis->moveDistance(170_cm);
  chassis->turnAngle(180_deg);
  chassis->moveDistance(110_cm);
  L.move(-70); R.move(-70);
  delay(400);
  L.move(0); R.move(0);
  LF.move(50); RF.move(50);
  LB.move(50); RB.move(50);
  stack();
  LF.move(0); RF.move(0);
  LB.move(0); RB.move(0);
  L.move(-127); R.move(-127);
  LF.move(-50); RF.move(-50);
  LB.move(-50); RB.move(-50);
  delay(500);
  LF.move(0); RF.move(0);
  LB.move(0); RB.move(0);
}

void PIDArms (double Dst) {
  double kP = 0.6, kD = 0.0075, kI = 1.2, E, prevE, I, D, P, Pwr;
    do {
      if(M.get_digital(DIGITAL_Y)) break;
      E = Dst - PA.get_value();
      I += E;

      if(E == 0 || std::abs(E) > std::abs(Dst)) {
        I = 0;
      }

      D = E - prevE;
      prevE = E;

      Pwr = E*kP + I*kI + D*kD;
      A.move(Pwr);
      pros::delay(20);
    } while(abs(E) >= 30);
    A.move_velocity(0);
    A.set_brake_mode(MOTOR_BRAKE_HOLD);
    delay(300);
}
void tower () {
  PIDArms(2000);
  spinIntake();
  delay(400);
  spinOutakeSlow();
  delay(70);
  stopMovement();
  delay(300);
  PIDArms(2350);
  chassis->moveDistance(10_cm);
  spinOutake();
  delay(1000);
  stopMovement();
  spinIntake();
  chassis->moveDistance(-15_cm);
  PIDArms(1350);
  A.move(-127);
  delay(500);
  A.move(0);
}
void moveForwardTower () {
  PIDArms(2000);
  spinIntake();
  delay(400);
  spinOutakeSlow();
  delay(70);
  stopMovement();
  delay(300);
  PIDArms(2350);
  chassis->moveDistance(10_cm);
  chassis->turnAngle(-23_deg);
  spinOutake();
  delay(1000);
  stopMovement();
  spinIntake();
  chassis->moveDistance(-30_cm);
  PIDArms(1350);
  A.move(-127);
  delay(500);
  A.move(0);
}
void towerHIGH () {
  PIDArms(2750);
  chassis->moveDistance(35_cm);
  spinOutake();
  delay(1000);
  stopMovement();
  PIDArms(1350);
  A.move(-127);
  delay(500);
  A.move(0);
}

void skillsDeploy() {
  LF.move(-127); RF.move(-127);
  LB.move(-127); RB.move(-127);
  A.move(127);
  delay(1300);
  A.move(-127);
  delay(1300);
  A.move(0);
  stopMovement();
}

void skills () {
  skillsDeploy();
  spinIntake();
  chassis->setMaxVelocity(150);
  chassis->moveDistance(160_cm);
  chassis->moveDistance(-65_cm);
  chassis->turnAngle(-120_deg);
  chassis->moveDistance(23_cm);
  selfCheck();
  stopMovement();
  delay(100);
  tower();
  spinIntake();
  chassis->turnAngle(110_deg);
  chassis->moveDistance(70_cm);
  chassis->turnAngle(40_deg);
  chassis->moveDistance(30_cm);
  selfCheck();
  delay(500);
  stopMovement();
  delay(100);
  moveForwardTower();
  spinIntake();
  chassis->setMaxVelocity(130);
  chassis->turnAngle(-37_deg);
  chassis->moveDistance(270_cm);
  LF.move(80); RF.move(80);
  LB.move(80); RB.move(80);
  delay(1000);
  spinIntake();
  chassis->moveDistance(-70_cm);
  chassis->turnAngle(-65_deg);
  LF.move(40); RF.move(40);
  LB.move(40); RB.move(40);
  delay(1500);
  L.move(-90); R.move(-90);
  delay(400);
  L.move(0); R.move(0);
  skillStack();
  LF.move(0); RF.move(0);
  LB.move(0); RB.move(0);
  L.move(-127); R.move(-127);
  LF.move(-50); RF.move(-50);
  LB.move(-50); RB.move(-50);
  delay(500);
  LF.move(0); RF.move(0);
  LB.move(0); RB.move(0);
  moveTrayBack();
  chassis->turnAngle(230_deg);
  LF.move(-60); RF.move(-60);
  LB.move(-60); RB.move(-60);
  delay(1500);
  spinIntake();
  chassis->setMaxVelocity(250);
  chassis->moveDistance(160_cm);
  L.move(-90); R.move(-90);
  delay(300);
  L.move(0); R.move(0);
  chassis->moveDistance(-10_cm);
  towerHIGH();
  spinIntake();
}

void FourStack () {
  chassis->setMaxVelocity(150);
  chassis->moveDistance(80_cm);
  double kP = 0.6, kD = 0.0075, kI = 1.2, E, prevE, I, D, P, Pwr, Dst = 2150;
    do {
      if(M.get_digital(DIGITAL_Y)) break;
      E = Dst - PA.get_value();
      I += E;

      if(E == 0 || std::abs(E) > std::abs(Dst)) {
        I = 0;
      }

      D = E - prevE;
      prevE = E;

      Pwr = E*kP + I*kI + D*kD;

      A.move(Pwr);
      pros::delay(20);
    } while(abs(E) >= 30);
    A.move_velocity(0);
    A.set_brake_mode(MOTOR_BRAKE_HOLD);

    delay(200);
    spinIntake();
    chassis->moveDistance(20_cm);

    Dst = 1100;
      do {
        if(M.get_digital(DIGITAL_Y)) break;
        E = Dst - PA.get_value();
        I += E;

        if(E == 0 || std::abs(E) > std::abs(Dst)) {
          I = 0;
        }

        D = E - prevE;
        prevE = E;

        Pwr = E*kP + I*kI + D*kD;

        A.move(std::min(Pwr, 80.0));
        pros::delay(20);
      } while(abs(E) >= 30);
      A.move_velocity(0);
      A.set_brake_mode(MOTOR_BRAKE_HOLD);
}

void trackingWheelsTest () {
  driveTurn(90);
}
//
// void absTurnController(double target) {
//   double kP = 1, kI = 0.0002;
//   double totalError = 0;
//   int heading = 0;
//   while(abs(heading - target) > 3) {
//     heading = (int) I.get_heading() > 180 ? I.get_heading() - 360 : I.get_heading();
//     totalError += heading - target;
//     double pwr = (double) signum(heading - target) * (abs((heading - target) * kP) + 20) + signum(heading - target) * totalError * kI;
//     RF.move(pwr);
//     RB.move(pwr);
//     LF.move(-pwr);
//     LB.move(-pwr);
//     pros::lcd::print(0, "heading: %d\n", (int) heading);
//     pros::lcd::print(1, "pwr: %f\n", (float) pwr);
//     pros::delay(20);
//     if(abs(totalError) > 2000) {
//       totalError = 0;
//     }
//   }
// }
//
// void move(double target, double speed) {
//   // TEST FUNCTION
//   // DO NOT USE
//   // CURRENT FUNCTION: HYPERBOLIC TANGENT CONTROLER WITH PID INERTIAL SENSOR ERROR CORRECTION [TANH_PIDIEC_CTRL]
//   // SPECIAL NOTES:
//   // DO NOT USE TARGET VALUE OF <500
//   double kP = 1, kI = 0.0002;
//   double kDe = (double) 1 / 80;
//   double kAc = (double) 1 / 10;
//   double pos = 0;
//   double pwr = 0;
//   double minV = 30;
//   double totalError = 0;
//   int heading = (int) I.get_heading();
//   int currentHeading = (int) I.get_heading();
//   if(target > 0) {
//     do {
//       pos = (LE.get_value() + RE.get_value()) / 2;
//       double pwr = ((speed / 2) * (tanh(kAc * pos - 2) - tanh(kDe * (pos - target + 150))) + minV) > 127 ? 127 : ((speed / 2) * (tanh(1 / kAc * pos - 2) - tanh(1 / kDe * (pos - target + 150))) + minV);
//       heading = (int) I.get_heading() - currentHeading > 180 ? I.get_heading() - 360 - currentHeading : I.get_heading() - currentHeading;
//       totalError += heading;
//       double ecpwr = (double) signum(heading) * (abs((heading) * kP) + 20) + signum(heading) * totalError * kI;
//       double lpwr = pwr - ecpwr;
//       double rpwr = pwr + ecpwr;
//       LF.move(lpwr);
//       LB.move(lpwr);
//       RF.move(rpwr);
//       RB.move(rpwr);
//       pros::delay(20);
//       double thing = tanh(kDe * pos - 2);
//       double other_thing = tanh(kDe * (pos - target + 100));
//       if(totalError > 2000) {
//         totalError = 0;
//       }
//     } while(abs(target - pos) > 25);
//     LF.move_velocity(0);
//     LB.move_velocity(0);
//     RF.move_velocity(0);
//     RB.move_velocity(0);
//   } else {
//     target = -target;
//     do {
//       pos = -(LE.get_value() + RE.get_value()) / 2;
//       double pwr = ((speed / 2) * (tanh(kAc * pos - 2) - tanh(kDe * (pos - target + 150))) + minV) > 127 ? 127 : ((speed / 2) * (tanh(1 / kAc * pos - 2) - tanh(1 / kDe * (pos - target + 150))) + minV);
//       heading = (int) I.get_heading() > 180 ? I.get_heading() - 360 : I.get_heading();
//       totalError += heading;
//       double ecpwr = (double) signum(heading - currentHeading) * (abs((heading - currentHeading) * kP) + 20) + signum(heading - currentHeading) * totalError * kI;
//       double lpwr = -pwr - ecpwr;
//       double rpwr = -pwr + ecpwr;
//       LF.move(lpwr);
//       LB.move(lpwr);
//       RF.move(rpwr);
//       RB.move(rpwr);
//       pros::delay(20);
//       if(totalError > 2000) {
//         totalError = 0;
//       }
//     } while(abs(target - pos) > 25);
//     LF.move_velocity(0);
//     LB.move_velocity(0);
//     RF.move_velocity(0);
//     RB.move_velocity(0);
//   }
//   while(abs(heading) >= 1) {
//     heading = (int) I.get_heading() > 180 ? I.get_heading() - 360 : I.get_heading();
//     totalError += heading;
//     double pwr = (double) signum(heading) * (abs(heading * kP) + 20) + signum(heading) * totalError * kI;
//     RF.move(pwr);
//     RB.move(pwr);
//     LF.move(-pwr);
//     LB.move(-pwr);
//     pros::delay(20);
//     if(totalError > 2000) {
//       totalError = 0;
//     }
//   }
//   RF.move(0);
//   RB.move(0);
//   LF.move(0);
//   LB.move(0);
//   LE.reset();
//   RE.reset();
// }

void autonomous() {
  matchDeploy();
  BlueS();
}
