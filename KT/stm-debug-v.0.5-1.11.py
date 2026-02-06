import serial
import sys

PORT = '/dev/ttyACM0'
BAUD = 115204

def run(mode):
    print(f"--- 启动模式: {mode} (监听 {PORT}) ---")
    try:
        # timeout=0.1 保证循环快速响应
        with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
            ser.flush()
            while True:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue # 下位机没发送，就不打印
                
                try:
                    line = line_bytes.decode('utf-8', errors='ignore').strip()
                except:
                    continue

                if not line:
                    continue

                # 逻辑 1：处理 [K]
                if mode == 'K' and line.startswith('[K]'):
                    print(f"下位机原始数据: {line}")
                    # 模拟自动回复
                    reply = (line[3:-1] + 'b\n').encode()
                    ser.write(reply)

                # 逻辑 2：处理 [P]
                elif mode == 'P' and line.startswith('[P]'):
                    print(f"下位机回显确认: {line}")

    except Exception as e:
        print(f"错误: {e}")

if __name__ == '__main__':
    # 运行时输入 python3 serial_debug.py K 或 P
    if len(sys.argv) > 1:
        run(sys.argv[1].upper())
    else:
        print("请指定运行模式: K 或 P")
