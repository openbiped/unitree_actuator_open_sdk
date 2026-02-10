#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>
#define PI 3.1415926

int main() {

  SerialPort  serial("/dev/ttyUSB0");
  MotorCmd    cmd;
  MotorData   data;

  while(true) 
  {
    cmd.motorType = MotorType::GO_2;
    data.motorType = MotorType::GO_2;
    cmd.mode = queryMotorMode(MotorType::GO_2,MotorMode::FOC);
    cmd.id   = 0;
    cmd.kp   = 0.0;
    cmd.kd   = 0.01;
    cmd.q    = 0.0;
    cmd.dq   = -6.28*queryGearRatio(MotorType::GO_2);
    cmd.tau  = 0.0;
    serial.sendRecv(&cmd,&data);

    std::cout <<  std::endl;
    std::cout <<  "motor.q: "    << data.q    <<  std::endl;
    std::cout <<  "motor.temp: "   << data.temp   <<  std::endl;
    std::cout <<  "motor.W: "      << data.dq      <<  std::endl;
    std::cout <<  "motor.merror: " << data.merror <<  std::endl;
    std::cout <<  std::endl;

    usleep(200);
  }

}
