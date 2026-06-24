#include <Dynamixel2Arduino.h>
#include <IMU.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
#define RPI_SERIAL   Serial   // 라즈베리파이 USB 시리얼 연결 (OpenCR은 Serial이 USB 가상 시리얼입니다)

cIMU imu;
const int DXL_DIR_PIN = 84;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t BAUDRATE = 57600;

const uint8_t IDS[12] = {
  1, 2, 3, 4, 5, 6,    // 왼발 (1~6)
  7, 8, 9, 10, 11, 12  // 오른발 (7~12)
};

int32_t DYNAMIC_CENTER_POS[12] = {0, };


const float URDF_MIN_RAD[12] = {
  -1.57,   -1.57,   -1.57,    0.0,   -0.43, -1.04, // 왼발 (1번: -1.57)
  0.0,     -0.43,   -0.43,   -1.39,  -1.57, -1.57  // 오른발 (7번: 0.0)
};

const float URDF_MAX_RAD[12] = {
  0.0,    0.43,     0.43,     1.39,   1.57,  1.57,  // 왼발 (1번: 0.0)
  1.57,   1.57,     1.57,     0.0,    0.43,  1.04   // 오른발 (7번: 1.57)
};

const float RAD_TO_DXL_STEP = 651.74;
unsigned long lastFeedbackTime = 0;
const int FEEDBACK_PERIOD = 20; // 50Hz


void sendRobotState()
{
  Serial.print("POS:");
  for(int i = 0; i < 12; i++)
  {
    int32_t raw =
      dxl.getPresentPosition(
        IDS[i],
        UNIT_RAW
      );
    float joint_rad =
      (
        raw - DYNAMIC_CENTER_POS[i]
      )
      /
      RAD_TO_DXL_STEP;
    // 방향 반전
    if(
      IDS[i]==3 ||
      IDS[i]==4 ||
      IDS[i]==5 ||
      IDS[i]==6 ||
      IDS[i]==9 ||
      IDS[i]==10 ||
      IDS[i]==11 ||
      IDS[i]==12
    )
    {
      joint_rad = -joint_rad;
    }
    Serial.print(
      joint_rad,
      5
    );
    if(i < 11)
      Serial.print(",");
  }

  Serial.println();
  Serial.print("VEL:");
  for(int i = 0; i < 12; i++)
  {
    int32_t rpm =
      dxl.getPresentVelocity(
        IDS[i],
        UNIT_RPM
      );
    float rad_s =
      rpm *
      2.0 *
      PI /
      60.0;
    Serial.print(
      rad_s,
      5
    );
    if(i < 11)
      Serial.print(",");
  }
  Serial.println();
}

void setup()
{
  DEBUG_SERIAL.begin(115200); // 파이썬 통신 속도와 일치시킵니다.
  RPI_SERIAL.setTimeout(5);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for(int i = 0; i < 12; i++) {
    uint8_t id = IDS[i];
    if(dxl.ping(id)) {
      DYNAMIC_CENTER_POS[i] = dxl.getPresentPosition(id, UNIT_RAW);
    } else {
      DYNAMIC_CENTER_POS[i] = 2048; 
    }
  }

  for(int i = 0; i < 12; i++) {
    uint8_t id = IDS[i];
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_EXTENDED_POSITION); 
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, 5);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 3);
    dxl.writeControlTableItem(POSITION_P_GAIN, id, 850); 
    dxl.writeControlTableItem(POSITION_D_GAIN, id, 0); 
    dxl.torqueOn(id);
    dxl.setGoalPosition(id, (uint32_t)DYNAMIC_CENTER_POS[i], UNIT_RAW);
    delay(20);
  }
  imu.begin();
  delay(100);
}

void loop()
{
  // 라즈베리파이(RPI_SERIAL)로부터 데이터 수신
  if (RPI_SERIAL.available() > 0)
  {
    String inputString = RPI_SERIAL.readStringUntil('\n');
    inputString.trim();

    if (inputString.length() == 0) return;

    if (inputString == "0") {
      for(int i = 0; i < 12; i++) {
        dxl.setGoalPosition(IDS[i], (uint32_t)DYNAMIC_CENTER_POS[i], UNIT_RAW);
      }
      RPI_SERIAL.println("ACK:RESET");
      return;
    }

    int currentPos = 0;
    int nextSlash = 0;

    while (currentPos < inputString.length()) {
      nextSlash = inputString.indexOf('/', currentPos);
      
      String token;
      if (nextSlash == -1) {
        token = inputString.substring(currentPos);
        currentPos = inputString.length();
      } else {
        token = inputString.substring(currentPos, nextSlash);
        currentPos = nextSlash + 1;
      }

      token.trim();
      if (token.length() == 0) continue;

      int commaIndex = token.indexOf(',');
      if (commaIndex == -1) continue;

      String idPart = token.substring(0, commaIndex);
      String offsetPart = token.substring(commaIndex + 1);

      int targetID = idPart.toInt();
      int32_t offsetInput = offsetPart.toInt();

      if (targetID < 1 || targetID > 12) continue;

      int idx = targetID - 1;
      int32_t center = DYNAMIC_CENTER_POS[idx];

      int32_t min_offset_limit = (int32_t)(URDF_MIN_RAD[idx] * RAD_TO_DXL_STEP);
      int32_t max_offset_limit = (int32_t)(URDF_MAX_RAD[idx] * RAD_TO_DXL_STEP);

      // 물리적 방향/한계 분기문
      if (targetID == 1|| targetID == 4|| targetID == 6||targetID == 7||targetID ==10) {
        offsetInput = -offsetInput; 
      }


      // 가드 클램핑
      if (offsetInput < min_offset_limit) offsetInput = min_offset_limit;
      if (offsetInput > max_offset_limit) offsetInput = max_offset_limit;

      int32_t finalTarget = center + offsetInput;
      dxl.setGoalPosition(targetID, finalTarget, UNIT_RAW);
    }
    // 라즈베리파이에게 명령을 잘 처리했다고 응답 전송 (파이썬 블로킹 방지용)
    RPI_SERIAL.println("ACK:OK");
  }

  imu.update();


  float raw_roll  = imu.rpy[0];
  float raw_pitch = imu.rpy[1];
  float raw_yaw   = imu.rpy[2];

  // 시계방향 90도 회전 보정
  float corrected_roll  = -raw_roll;
  float corrected_pitch = -raw_pitch;
  float corrected_yaw   =  raw_yaw;

  // yaw 범위 정규화 (-180 ~ 180)
  if (corrected_yaw < -180.0f) corrected_yaw += 360.0f;
  if (corrected_yaw >  180.0f) corrected_yaw -= 360.0f;

  if(
    millis() - lastFeedbackTime 
    > FEEDBACK_PERIOD
  )
  {
    lastFeedbackTime = millis();
    Serial.print("IMU:");

    Serial.print(corrected_roll);
    Serial.print(",");

    Serial.print(corrected_pitch);
    Serial.print(",");

    Serial.print(corrected_yaw);
    Serial.print(",");

    Serial.print(imu.gyroData[0]);
    Serial.print(",");

    Serial.print(imu.gyroData[1]);
    Serial.print(",");

    Serial.println(imu.gyroData[2]);
  }
sendRobotState();

delay(1);
  
}