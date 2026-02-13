#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>

#if !defined(GOVERSION) || (GOVERSION!=1 && GOVERSION!=2)
  #error GOVERSION must be defined to be either 1 or 2
#endif

static void usage(const char* argv0) {
    printf("Go Motor ID change tool.\n");
    printf("Version: %d\n\n", GOVERSION);
    printf("Motor broadcast ID:15\n");
    printf("Notice: There cannot be motors with the same ID on a RS-485 bus!!!\n\n");
    printf("usage: %s [tty device] [id] [target_id]\n", argv0);
    printf("ex:    %s /dev/ttyUSB0 0 1  :Set motor 0 to id 1 [id] [target_id]\n", argv0);
    printf("ex:    %s /dev/ttyUSB0 15 1 :Set All motor to id 1 (Use with caution)\n", argv0);
}

int main(int argc, const char* argv[]) {
  if (argc < 4) {
    usage(argv[0]);
    return 0;
  }
  const char* serialPort = argv[1];
  int oldid = atoi(argv[2]);
  int newid = atoi(argv[3]);
  uint8_t buffer[] = {
    0xFB,
    1,
    uint8_t(((newid&0xF)<<4)|(oldid&0xF)),
    0xBB
  };

  SerialPort  serial(serialPort);
  MotorCmd    cmd;
  MotorData   data;

  // Enter boot loader
#if GOVERSION == 1
  cmd.motorType = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;
#elif GOVERSION == 2
  cmd.motorType = MotorType::GO_2;
  data.motorType = MotorType::GO_2;
#else
  #error GOVERSION must be defined to be either 1 or 2
#endif
  cmd.mode = 7;
  cmd.id   = 15;
  cmd.kp   = 0.0;
  cmd.kd   = 0.0;
  cmd.q    = 0.0;
  cmd.dq   = 0.0;
  cmd.tau  = 0.0;
  serial.sendRecv(&cmd,&data);
  sleep(2);

  if (serial.send(buffer, sizeof(buffer)) != sizeof(buffer)) {
    fprintf(stderr, "Failed to send changeid command\n");
    return 1;
  }
  usleep(200);
  (void) serial.recv(buffer, sizeof(buffer));
  sleep(2);

  // Exit boot loader
  for (char id = 0; id < 15; id++) {
    uint8_t exitBuffer[] = {
      0xFB, 0x04, 0x00, 0xBB,
      0xFB, 0x03, 0x00, 0xBB
    };
    exitBuffer[2] = id;
    exitBuffer[6] = id;
    if (serial.send(&exitBuffer[0], 4) != 4) {
        fprintf(stderr, "Failed to send swmotor command\n");
        return 1;
    }
    usleep(200);
    if (serial.send(&exitBuffer[4], 4) != 4) {
        fprintf(stderr, "Failed to send swmotor command\n");
        return 1;
    }
    usleep(500);
  }

  printf("SUCCESS\n");
  return 0;
}
