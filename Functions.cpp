#include "main.h"
#include "Declarations.hpp"
#include "Functions.hpp"
using namespace std;

int sgn (double v) {
  return v / abs(v);
}
void Drv (void*) {
  int LS, RS, LP, RP;
  while(true) {
    LS = M.get_analog(ANALOG_LEFT_Y) + M.get_analog(ANALOG_LEFT_X)*0.7;
    RS = M.get_analog(ANALOG_LEFT_Y) - M.get_analog(ANALOG_LEFT_X)*0.7;

    LP = int(pow(LS, 2) / 127) * sgn(LS);
    RP = int(pow(RS, 2) / 127) * sgn(RS);

    LF.move(LP); LB.move(LP);
    RF.move(RP); RB.move(RP);
    pros::delay(20);
  }

}
void Itk (void*) {
  while(true) {
    if(M.get_digital(DIGITAL_R1)) {
      L.move(127); R.move(127);
    }
    else if(M.get_digital(DIGITAL_R2) && M.get_digital(DIGITAL_Y)) {
      L.move(-70); R.move(-70);
    }
    else if(M.get_digital(DIGITAL_R2)) {
      L.move(-127); R.move(-127);
    }
    else {
      L.move_velocity(0); R.move_velocity(0);
      L.set_brake_mode(MOTOR_BRAKE_HOLD); R.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
    pros::delay(20);
  }
}
void Lft (void*) {
  while(true) {
    if(M.get_digital(DIGITAL_L1)) {
      A.move(127);
    }
    else if(M.get_digital(DIGITAL_L2)) {
      A.move(-127);
    }
    else if(M.get_digital(DIGITAL_UP)) {
      double kP = 0.6, kD = 0.0075, kI = 1.2, E, prevE, I, D, P, Pwr, Dst = 2670;
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
    }
    else if(M.get_digital(DIGITAL_LEFT)) {
      double kP = 0.6, kD = 0.0075, kI = 1.2, E, prevE, I, D, P, Pwr, Dst = 2200;
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
    }
    else if(M.get_digital(DIGITAL_RIGHT)) {
      double kP = 0.6, kD = 0.0075, kI = 1.2, E, prevE, I, D, P, Pwr, Dst = 2130;
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
    }
    else if(M.get_digital(DIGITAL_DOWN)) {
      double kP = 0.6, kD = 0.0075, kI = 1.2, E, prevE, I, D, P, Pwr, Dst = 1200;
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
        A.move(-127);
        pros::delay(100);
        A.move_velocity(0);
        A.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
    else {
      A.move_velocity(0);
      A.set_brake_mode(MOTOR_BRAKE_COAST);
    }
    pros::delay(20);
  }
}
void Tlt (void*) {
  while(true) {
    if(abs(M.get_analog(ANALOG_RIGHT_Y)) > 10) {
      T.move(M.get_analog(ANALOG_RIGHT_Y));
    }
    else if(M.get_digital(DIGITAL_X)) {
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
    else if(M.get_digital(DIGITAL_B)) {
      double kP = 0.6, kD = 0.0075, kI = 1.2, E, prevE, I, D, P, Pwr, Dst = 450;
        do {
          if(M.get_digital(DIGITAL_Y)) break;
          E = Dst - PT.get_value();
          I += E;

          if(E == 0 || std::abs(E) > std::abs(Dst)) {
            I = 0;
          }

          D = E - prevE;
          prevE = E;

          Pwr = E*kP + I*kI + D*kD;

          T.move(Pwr);
          pros::delay(20);
        } while(abs(E) >= 30);
        T.move_velocity(0);
        T.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
    else {
      T.move_velocity(0);
    }
    pros::delay(20);
  }
}
