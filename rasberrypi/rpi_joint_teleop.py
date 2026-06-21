import serial
import time

# OpenCR USB 포트 설정 (라즈베리파이 환경에 맞춰 포트명 변경 필요)
PORT = '/dev/ttyACM1' 
BAUDRATE = 115200

try:
    py_serial = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=1)
    time.sleep(2) # OpenCR 리셋 대기 시간
    print("➔ OpenCR Connection Success!")
except Exception as e:
    print(f"❌ Connection Failed: {e}")
    exit()

print("==============================================")
print("  Raspberry Pi to OpenCR Joint Controller")
print("  Input Format Example: 9,-200  or  3,-100/1,200")
print("  Press '0' to Reset All Motors to Center")
print("  Press 'q' to Exit")
print("==============================================")

while True:
    cmd = input("Input Command ➔ ").strip()
    
    if cmd.lower() == 'q':
        print("Exit Program.")
        py_serial.close()
        break
        
    if not cmd:
        continue

    # OpenCR로 개행문자(\n)를 포함해 데이터 전송
    py_serial.write(f"{cmd}\n".encode())
    
    # OpenCR로부터 ACK 응답 대기
    response = py_serial.readline().decode().strip()
    if response:
        print(f"⟲ OpenCR Response: {response}")
