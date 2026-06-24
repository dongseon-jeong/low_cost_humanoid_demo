import serial
import time

PORT = '/dev/ttyACM0'
BAUDRATE = 115200

try:
    py_serial = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=0.1
    )
    time.sleep(2)
    print("➔ OpenCR Connection Success!")
except Exception as e:
    print(f"❌ Connection Failed: {e}")
    exit()

print("==============================================")
print(" Raspberry Pi to OpenCR Joint Controller")
print("")
print(" Example:")
print("   1,100")
print("   3,-100/1,200")
print("")
print(" 0 : Reset Motors")
print(" q : Exit")
print("==============================================")

def read_serial():
    while py_serial.in_waiting:
        try:
            line = (
                py_serial
                .readline()
                .decode()
                .strip()
            )
            if line:
                print(
                    "OpenCR >>",
                    line
                )
        except:
            pass

while True:
    # 먼저 들어온 센서 데이터 출력
    read_serial()
    cmd = input(
        "Input Command ➔ "
    ).strip()
    if cmd.lower() == 'q':
        print("Exit")
        py_serial.close()
        break
    if not cmd:
        continue

    # 명령 전송
    py_serial.write(
        (cmd+"\n").encode()
    )
    print(
        "Send >>",
        cmd
    )
    time.sleep(0.1)
    # ACK / 센서 출력 확인
    read_serial()