#include "main.h"
#include "Declarations.hpp"
#include "Functions.hpp"

using namespace pros;

void initialize() {
  lcd::initialize();
  PA.calibrate();
  PT.calibrate();
  LT.reset(); RT.reset();
}
void disabled() {}
void competition_initialize() {}

void opcontrol() {
	Task DrvTask (Drv, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
	Task ItkTask (Itk, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
	Task TltTask (Tlt, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
	Task LftTask (Lft, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
	while (true) {
    lcd::print(0, "%d %d", LT.get_value(), RT.get_value());
    delay(20);
  }
}
