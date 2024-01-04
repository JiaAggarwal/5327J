#include "main.h"
#include "lemlib/api.hpp" // lemlib - pid, odom and pure pursuit vrc library
#include "pros/misc.h"

pros::Motor front_left_mtr(FRONT_LEFT_PORT, pros::E_MOTOR_GEARSET_06, true,
                           pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor front_right_mtr(FRONT_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false,
                            pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor mid_left_mtr(MID_LEFT_PORT, pros::E_MOTOR_GEARSET_06, true,
                         pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor mid_right_mtr(MID_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false,
                          pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor back_left_mtr(BACK_LEFT_PORT, pros::E_MOTOR_GEARSET_06, true,
                          pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor back_right_mtr(BACK_RIGHT_PORT, pros::E_MOTOR_GEARSET_06, false,
                           pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor flywheel_mtr_1(FLYWHEEL_ONE_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Motor intake_mtr(INTAKE_ONE_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor puncher_mtr(PUNCHER_PORT, pros::E_MOTOR_GEARSET_18, true);

pros::ADIDigitalOut pushbar_piston_1(1, true);
pros::ADIDigitalOut blocker_piston(2, true);
pros::ADIDigitalOut elevation_piston_1(3,
                                       false); // elevation deactivated (left)
pros::ADIDigitalOut elevation_piston_2(4,
                                       false); // elevation deactivated (right)
// pros::ADIDigitalOut elevation_piston_3(6, false); // blocker down

// // lemlib motor groups
// pros::MotorGroup left_side_motors({front_left_mtr, mid_left_mtr,
//                                    back_left_mtr});
// pros::MotorGroup right_side_motors({front_right_mtr, mid_right_mtr,
//                                     back_right_mtr});

// lemlib::Drivetrain_t drivetrain{
//     &left_side_motors,  // left drivetrain motors
//     &right_side_motors, // right drivetrain motors
//     8,                  // track width - width between wheels
//     3.25,               // wheel diameter
//     360                 // wheel rpm
// };

// lemlib pids
// lemlib::OdomSensors_t sensors{nullptr, nullptr, nullptr, nullptr, nullptr};
// create the chassis for lemlib
// forward/backward PID
// lemlib::ChassisController_t lateralController{
//     7,   // kP
//     70,  // kD
//     1,   // smallErrorRange
//     100, // smallErrorTimeout
//     3,   // largeErrorRange
//     500, // largeErrorTimeout
//     5    // slew rate
// };

// turning PID
// lemlib::ChassisController_t angularController{
//     7.2, // kP
//     90,  // kD
//     1,   // smallErrorRange
//     100, // smallErrorTimeout
//     3,   // largeErrorRange
//     500, // largeErrorTimeout
//     0    // slew rate
// };

// no odom sensors - will use nullptr

// lemlib::Chassis chassis(drivetrain, lateralController, angularController,
// sensors);

pros::Controller master(pros::E_CONTROLLER_MASTER);

void brake_coast() {
  front_left_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  front_right_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  mid_left_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  mid_right_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  back_left_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  back_right_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

bool flywheel_flag = false;
void run_flywheel(int rpm) {
  if (!flywheel_flag) {
    flywheel_mtr_1.move(rpm);
    flywheel_mtr_1.move(rpm);
  } else {
    flywheel_mtr_1.move(0);
    flywheel_mtr_1.move(0);
  }
  flywheel_flag = !flywheel_flag;
}

bool brake_flag = false;
bool cata_flag = false;
bool slow_flag = false;

void run_cata() {
  if (!cata_flag) {
    puncher_mtr.move(127);
    cata_flag = true;
    pros::delay(100);
  } else {
    puncher_mtr.move(0);
    cata_flag = false;
    pros::delay(100);
  }
}

////

// constants
const double kP = 2.33;
const double kI = 0.0000005; // 0.2
const double kD = 0.0003;

int lateralPID(bool reset_drive_encoders, double desired_value, int velocity) {

  desired_value = 360 * (desired_value / (4.125 * PI));
  double frontleftmotor_position = front_left_mtr.get_position();
  double frontrightmotor_position = front_right_mtr.get_position();

  double average_position =
      ((frontleftmotor_position + frontrightmotor_position) / 2);

  double error = (desired_value - average_position);
  double previous_error = 0;
  double integral = 0;

  int lateral_motor_power = 127;

  if (reset_drive_encoders) {
    front_left_mtr.tare_position();
    front_right_mtr.tare_position();
    back_left_mtr.tare_position();
    back_right_mtr.tare_position();
    mid_left_mtr.tare_position();
    mid_right_mtr.tare_position();

    reset_drive_encoders = false;
  }

  while (fabs(error) >= 0.10) {

    frontleftmotor_position = front_left_mtr.get_position();
    frontrightmotor_position = front_right_mtr.get_position();

    average_position =
        ((frontleftmotor_position + frontrightmotor_position) / 2);

    error = (desired_value - average_position);

    integral += error;
    if (error <= 0) {
      integral = 0;
    }

    double derivative = (error - previous_error);

    lateral_motor_power = round(error * kP + integral * kI + derivative * kD);

    front_left_mtr.move_absolute(lateral_motor_power, velocity);
    front_right_mtr.move_absolute(lateral_motor_power, velocity);
    back_left_mtr.move_absolute(lateral_motor_power, velocity);
    back_right_mtr.move_absolute(lateral_motor_power, velocity);
    mid_left_mtr.move_absolute(lateral_motor_power, velocity);
    mid_right_mtr.move_absolute(lateral_motor_power, velocity);

    return lateral_motor_power;

    previous_error = error;

    pros::delay(20);
  }

  front_left_mtr.move(0);
  back_left_mtr.move(0);
  front_right_mtr.move(0);
  back_right_mtr.move(0);
  mid_left_mtr.move(0);
  mid_right_mtr.move(0);

  return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// turning pid
const double turn_kP = 0.84;
const double turn_kI = 0.125;
const double turn_kD = 0.06;

int turnPID(bool reset_drive_encoders, double desired_value_in_degrees,
            int velocity) {

  double desired_value_in_radians = desired_value_in_degrees * (PI / 180);
  double bot_radius = 10.25;

  double arc_length = bot_radius * desired_value_in_radians;

  arc_length = 360 * (arc_length / (4.125 * PI));
  double frontleftmotor_position = front_left_mtr.get_position();
  double frontrightmotor_position = front_right_mtr.get_position();

  double average_position =
      ((frontleftmotor_position + frontrightmotor_position) / 2);

  double error = (arc_length - average_position);
  double previous_error = 0;
  double integral = 0;

  int lateral_motor_power = 127;

  if (reset_drive_encoders) {
    front_left_mtr.tare_position();
    front_right_mtr.tare_position();
    back_left_mtr.tare_position();
    back_right_mtr.tare_position();
    mid_left_mtr.tare_position();
    mid_right_mtr.tare_position();

    reset_drive_encoders = false;
  }

  while (fabs(error) >= 0.25) {

    frontleftmotor_position = front_left_mtr.get_position();
    frontrightmotor_position = front_right_mtr.get_position();

    average_position =
        ((frontleftmotor_position + frontrightmotor_position) / 2);

    error = (arc_length - average_position);

    integral += error;
    if (error <= 0) {
      integral = 0;
    }

    double derivative = (error - previous_error);

    lateral_motor_power =
        round(error * turn_kP + integral * turn_kI + derivative * turn_kD);

    front_left_mtr.move_absolute(-lateral_motor_power, velocity);
    front_right_mtr.move_absolute(lateral_motor_power, velocity);
    back_left_mtr.move_absolute(-lateral_motor_power, velocity);
    mid_left_mtr.move_absolute(-lateral_motor_power, velocity);
    back_right_mtr.move_absolute(lateral_motor_power, velocity);
    mid_right_mtr.move_absolute(lateral_motor_power, velocity);

    return lateral_motor_power;

    previous_error = error;

    pros::delay(20);
  }

  front_left_mtr.move(0);
  back_left_mtr.move(0);
  front_right_mtr.move(0);
  back_right_mtr.move(0);
  mid_left_mtr.move(0);
  mid_right_mtr.move(0);

  return 0;
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of
 * the LCD text between "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the
 * program is started.
 *
 * All other competition modes are blocked by
 * initialize; it is recommended to keep execution time
 * for this mode under a few seconds.
 */

void screen() {
  // loop forever
  // while (true) {
  //   lemlib::Pose pose =
  //       chassis.getPose(); // get the current
  //       position of the robot
  //   pros::lcd::print(0, "x: %f", pose.x); // print
  //   the x position pros::lcd::print(1, "y: %f",
  //   pose.y);           // print the y position
  //   pros::lcd::print(2, "heading: %f", pose.theta);
  //   // print the heading pros::delay(10);
}

void initialize() {
  // chassis.calibrate();
  // pros::lcd::initialize();
  // pros::lcd::set_text(1, "5327J");
  // pros::lcd::register_btn1_cb(on_center_button);
  blocker_piston.set_value(false);
  pushbar_piston_1.set_value(false);
  // pros::Task screenTask(screen);
  // create a task to print the position to the screen
}

/**
 * Runs while the robot is in the disabled state of
 * Field Management System or the VEX Competition
 * Switch, following either autonomous or opcontrol.
 * When the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when
 * connected to the Field Management System or the VEX
 * Competition Switch. This is intended for
 * competition-specific initialization routines, such as
 * an autonomous selector on the LCD.
 *
 * This task will exit when the robot is enabled and
 * autonomous or opcontrol starts.
 */

///////////////////////////////////////////////////////////////////////////////////////////

void competition_initialize() {}
bool intake_flag = false;
bool outtake_flag = false;
void run_outtake() {
  if (!outtake_flag) {
    intake_mtr.move(-127);
    outtake_flag = true;
  } else {
    intake_mtr.move(0);
    outtake_flag = false;
  }
  pros::delay(500);
}
void run_intake() {
  if (!intake_flag) {
    intake_mtr.move(127);
    intake_flag = true;
  } else {
    intake_mtr.move(0);
    intake_flag = false;
  }
  pros::delay(500);
}

///////////////////////////////////////////////////////////////////////////////////////////

bool wings_flag = false;
void wings() {
  if (wings_flag) {
    pushbar_piston_1.set_value(true);
    wings_flag = false;
    pros::delay(100);
  } else {
    pushbar_piston_1.set_value(false);
    wings_flag = true;
    pros::delay(100);
  }
  pros::delay(100);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool blocker_flag = false;
void blocker() {
  if (blocker_flag) {
    blocker_piston.set_value(false);
    blocker_flag = false;
    pros::delay(100);
  } else {
    blocker_piston.set_value(true);
    blocker_flag = true;
    pros::delay(100);
  }
  pros::delay(200);
}

void move_chasis(int left, int right) {
  front_left_mtr.move(left);
  mid_left_mtr.move(left);
  back_left_mtr.move(left);
  front_right_mtr.move(right);
  mid_right_mtr.move(right);
  back_right_mtr.move(right);
}

/**
 * Runs the user autonomous code. This function will be
 * started in its own task with the default priority and
 * stack size whenever the robot is enabled via the
 * Field Management System or the VEX Competition Switch
 * in the autonomous mode. Alternatively, this function
 * may be called in initialize or opcontrol for
 * non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost,
 * the autonomous task will be stopped. Re-enabling the
 * robot will restart the task, not re-start it from
 * where it left off.
 */
void autonomous() {
  lateralPID(true, 3, 256);
  pros::delay(400);
  turnPID(true, 56, 256);
  pros::delay(300);
  lateralPID(true, 27, 256);
  pros::delay(1500);
  turnPID(true, -62, 256);
  pros::delay(500);
  lateralPID(true, 5, 256); // Ram
  run_outtake();
  pros::delay(500);
  run_outtake();
  lateralPID(true, -6, 256); // bXK UP
  pros::delay(500);
  turnPID(true, -205, 256); // turn around
  pros::delay(800);
  lateralPID(true, -7, 256); // go back
  pros::delay(500);
  lateralPID(true, 8.5, 256); // forward to match load area
  pros::delay(500);
  turnPID(true, 44, 256); // turn parallel to match load
  pros::delay(700);
  lateralPID(true, 13, 256); // forward
  pros::delay(1000);
  pushbar_piston_1.set_value(true);
  pros::delay(300);
  lateralPID(true, 2, 256); // forward
  pros::delay(300);
  turnPID(true, 40, 150); // turn to hook out
  pros::delay(500);
  turnPID(true, 90, 256); //
  pros::delay(500);
  turnPID(true, -100, 500);
  pros::delay(500);
  wings();
  pros::delay(300);
  lateralPID(true, 30, 256); // forward
  pros::delay(600);
  turnPID(true, 30, 256); //
  pros::delay(400);
  // lateralPID(true, 25, 256); // forward
  // pros::delay(400);
  // blocker();
  // pros::delay(300);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // prog skills
  // flywheel_mtr_1.move(-127);
  // pros::delay(60000);
  // flywheel_mtr_1.move(0);
  // pros::delay(300);
  // turnPID(true, 30, 256);
  // pros::delay(300);
  // lateralPID(true, 30, 256); // forward
  // pros::delay(2500);
  // turnPID(true, -25, 256);
  // pros::delay(2500);
  // lateralPID(true, 50, 256); // forward
  // pros::delay(5000);
  // turnPID(true, -90, 256);
  // pros::delay(5000);
  // lateralPID(true, 10, 256); // forward
  // pros::delay(2100);
  // turnPID(true, -90, 256);
  // pros::delay(5000);

  // pros::delay(2100);

  // for (int i = 0; i < 3; i++) {
  //   lateralPID(true, 20, 256); // forward
  //   pros::delay(1500);
  //   turnPID(true, -30, 256);
  //   pros::delay(400);
  //   lateralPID(true, -40, 256); // forward
  //   pros::delay(3000);
  //   turnPID(true, 30, 256);
  // pros::delay(300);
}

/**
 * Runs the operator control code. This function will be
 started in its own task
 * with the default priority and stack size whenever the
 robot is enabled via
 * the Field Management System or the VEX Competition
 Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function
 will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost,
 the
 * operator control task will be stopped. Re-enabling
 the robot will restart the
 * task, not resume it from where it left off.

 */

// void slow_cata() {
//   if (!slow_flag) {
//     puncher_mtr.move(20);
//     slow_flag = true;
//     pros::delay(100);
//   } else {
//     puncher_mtr.move(0);
//     slow_flag = false;
//     pros::delay(100);
//   }
// }

void elevation_up() {
  elevation_piston_1.set_value(true);
  elevation_piston_2.set_value(true);
}
void elevate() {
  elevation_piston_1.set_value(false);
  elevation_piston_2.set_value(false);
}

void opcontrol() {
  while (true) {

    // getting joystick input
    // int left = master.get_analog(ANALOG_LEFT_Y);
    // int right = master.get_analog(ANALOG_RIGHT_Y);
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X) * 0.7;
    int left = power + turn;
    int right = power - turn;
    // deadzones
    if (abs(left) < 10) {
      left = 0;

    } else {
      front_left_mtr.move(left);
      mid_left_mtr.move(left);
      back_left_mtr.move(left);
    }

    if (abs(right) < 10) {
      right = 0;
    }

    else {
      front_right_mtr.move(right);
      back_right_mtr.move(right);
      mid_right_mtr.move(right);
    }

    // brake logic
    if (left == 0) {
      front_left_mtr.brake();
      mid_left_mtr.brake();
      back_left_mtr.brake();
    }

    if (right == 0) {
      front_right_mtr.brake();
      mid_right_mtr.brake();
      back_right_mtr.brake();
    }
    brake_flag = false;

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      // intake
      run_intake();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      // outtake
      run_outtake();
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      // wings
      blocker();
      // elevation_piston_1.set_value(
      //     true); // blocker and elevator up
      //     (preparing for elevation)
      // elevation_piston_2.set_value(true);
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      wings();
      // elevation_piston_1.set_value(
      //     false); // blocker and elevator down
      //     (elevating)
      // elevation_piston_2.set_value(false);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      // stick fw
      flywheel_mtr_1.move(127);
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      // elevation up
      elevation_up();
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      // elevation down
      elevate();
    }

    pros::delay(20);
  }
}
