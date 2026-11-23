#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

extern Drive chassis;
////                                                                                                                             
/*en esta parte se declaran los demas dispositivos que utiliza el robot*/
////                                                                                                                             
//* aqui solo se declaran aun no estas utilizables en el opcontrol 
// recuerda que para reversiar un motor hay que ponerlo negativo

inline pros::Motor intake(-12); // esta es la forma que se declara un motor individual
inline pros::Motor intake1(-8); //*inline pros::Motor nombre_que_quieras(puerto_del_motor);

// tambien puedes declarar los motores en grupo o MotorGroup se hace de la siguiente forma 
// inline pros::MotorGroup nombre_que_quieras({port_motor1,port_motor2}); puedes poner los motores que quieras

/* ejemplo */
inline pros::MotorGroup grupo1({1, -3});

inline ez::Piston doinker('A');   // esta es la forma de declarar un piston
inline ez::Piston doinker1('B');  // inline ez::Piston nombre_que_quieras('puerto_del_piston');

inline pros::Motor expulsion(11);
inline pros::Motor expulsion1(-9);

// a continuacion dirijete a el archivo main.cpp hasta la parte de abajo
// te eseñare como hacer funcionar estos dispositivos en el opcontrol

/*si estas utilizando otros dispositivos como sensor de colores o sensor de distancia puedes buscar en internet ez-template ahi 
  te aparecera como declararlos esto debido a que no los utilizamos esta vez y tambien debes pensar un poco en como hacer eso */