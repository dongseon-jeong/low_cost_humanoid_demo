#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial

const int DXL_DIR_PIN = 84;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup()
{
  DEBUG_SERIAL.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  uint8_t old_id = 1;
  uint8_t new_id = 7;

  if(dxl.ping(old_id))
  {
    DEBUG_SERIAL.println("Motor found");

    dxl.torqueOff(old_id);

    if(dxl.setID(old_id, new_id))
    {
      DEBUG_SERIAL.println("ID changed to");
    }
    else
    {
      DEBUG_SERIAL.println("ID change failed");
    }
  }
  else
  {
    DEBUG_SERIAL.println("Motor not found");
  }
}

void loop()
{
}