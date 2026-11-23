#include <type_traits>
#include "EZ-Template/PID.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"


// These are out of 127
const int DRIVE_SPEED = 110;  // estas son las constantes de conduccion para los autonomos puedes ponerlas de 0 a 127
const int TURN_SPEED = 90;    // drive es la velocidad de conduccion adelante/atras
const int SWING_SPEED = 110;  // turn es la velocidad de gira (puedes ajustarlas a como tu prefieras) y swins es un tipo de giro en s

///             / esta parte son los valores de p,i,d por lo regular la i no se toca asi que nomas moveras las constantes p y d
// Constants    / te recomiendo que uses solo pid_drive, pid_turn, y si tu lo prefieres el swing
///             / esto es un poco complejo asi que pon atencion y si es necesario leelo varias veces
void default_constants() {
  /*
  una ves hecha la traccion y configurado los motores y el inercial tendras que poner el robot en los tapetes de competicion 
  acomodalo bien y recuerda no mover el robot cuando este iniciando el programa, el tapete mide 24 pulgadas 
  ponlo justo en la union de tapetes y el robot tendra que avanzar a la proxima union con el otro tapete
  para ello te deje unos ejemplos para que puedas hacer esto mas facil 
  ejecutaras el programa y en la pantalla seleccionaras el autonomo de la pagina 4 
  para ejecutar el autonomo pulsa flecha abajo mas la B y el autonomo se ejecutara
  este tendra que avanzar 24 pulgadas y lo puedes comprobar con la cinta metrica
  no lo hara a la primera debes estar cambiando las constantes p,i,d y para que no estes cargando el programa una y otra vez
  puedes estarlo haciendo en live corre el programa 
  pulsa la x en el control y te cambiara la pantalla del cerebro 
  te apareceran las constantes de pid para cambiarlas utiliza la flechas para mover entre las constantes
  y con A incrementa y con Y decrementa
 
     use A and Y to increment / decrement the constants
     use the arrow keys to navigate the constants
     
     digital_down + digital_B correra el autonomo seleccionado 

  si avanzo mucho baja la p y d como te lo indique y si le hizo falta aumenta p o d
   realiza de nuevo el autonomo y repite el proceso hasta que lo haga lo mas exacto posible

   una vez lo haya hecho bien recuerda como quedo p y d , tendras que ponerlas aqui en el codigo debido a que no se guardan

  y ese es el proceso de calibracion para el pid 
  raliza el mismo proceso con las que te indique hay un ejemplo de cada una asi como el 4 es el drive el 5 es el turn y el 6 swing
  y asi consecutivamente solo utilizaras drive, turn y el swing si lo prefieres

  es el mismo proceso en todas en los giros si gira demas baja p y d si hace falta girar aumenta p y d
  
  */
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(19.90, 0.0, 95.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(10.0, 0.0, 17.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.10, 0.05, 20.0, 0.0);     // Turn in place constants
  chassis.pid_swing_constants_set(7.70, 0.0, 55.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions
  
  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
//auto blue allianz left
///
void blue_allianz1(){

 chassis.pid_drive_set(16_in, DRIVE_SPEED, true); //esta parte avanza
 chassis.pid_wait();
 
 intake.move(-127);//asi se activa los mecanismos para pararlo es igual solo pon en el voltage en 0


 chassis.pid_drive_set(10_in, 60, true); //avanza mas lento
 chassis.pid_wait();

 chassis.pid_turn_set(-125_deg, TURN_SPEED); //aqui gira a -125 grados
 chassis.pid_wait();

 chassis.pid_drive_set(-18,DRIVE_SPEED);
 chassis.pid_wait();
 
 intake1.move(-127);
 expulsion.move(127);
 expulsion1.move(127);

 pros::delay(1500); //para que espere un determinado tiempo

 expulsion.move(0);
 expulsion1.move(0);

 chassis.pid_wait();
 
 chassis.pid_drive_set(24,DRIVE_SPEED);
 chassis.pid_wait();

chassis.pid_turn_set(-120_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_drive_set(31,DRIVE_SPEED);
 chassis.pid_wait();

 doinker1.set(true); //activa sistema de neumatico
 
 doinker.set(true);

 chassis.pid_turn_set(-165_deg, TURN_SPEED);
 chassis.pid_wait();

 pros::delay(400);
 
 chassis.pid_drive_set(8,90);
 chassis.pid_wait();

 pros::delay(400);

 chassis.pid_turn_set(-165_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_drive_set(-30,DRIVE_SPEED);
 chassis.pid_wait();

 expulsion.move(-127);
 expulsion1.move(127);
 intake1.move(-127);
 intake.move(-127);


 
 pros::delay(3000);
 

}

///
//auto blue allianz right
///
void blue_allianz11(){
 chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);
 chassis.pid_wait();
 
 intake.move(-127);
 doinker.set(true);

 chassis.pid_turn_set(-90,TURN_SPEED);
 chassis.pid_wait();

 intake.move(-127);
 intake1.move(-127);

 chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
 chassis.pid_wait();

  pros::delay(400);

 chassis.pid_drive_set(-30_in, 120, true);
 chassis.pid_wait();

 expulsion.move(-127);
 expulsion1.move(127);

}

///
//auton solo AWP
///
void autonAWP(){
 
  chassis.pid_drive_set(24,DRIVE_SPEED,true);
  chassis.pid_wait();

  chassis.pid_turn_set(180,TURN_SPEED);
  chassis.pid_wait();

  doinker.set(true);

  intake.move(-127);  

  chassis.pid_drive_set(12,DRIVE_SPEED,true);
  chassis.pid_wait();
  pros::delay(3000);

  chassis.pid_drive_set(-24,DRIVE_SPEED,true);
  chassis.pid_wait();

  expulsion.move(-127);
  expulsion1.move(127);

  chassis.pid_drive_set(12,DRIVE_SPEED,true);
  chassis.pid_wait();

  
  chassis.pid_turn_set(-45,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(6,DRIVE_SPEED,true);
  chassis.pid_wait();

  chassis.pid_drive_set(6,30,true);
  chassis.pid_wait();


}

void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

}

void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait();

}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}


///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .