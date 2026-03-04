import serial
import time
import threading
import queue

# 配置参数
PORT = '/dev/ttyACM0'
BAUD = 115204

# --- 信箱定义 (完全保留名称) ---
stm32_send_mailbox = queue.Queue()    
stm32_receive_mailbox = queue.Queue() 
stm32_echo_mailbox = queue.Queue()    

def push_to_send_mailbox(data: str):
    """供外部调用的填充信箱函数"""
    stm32_send_mailbox.put(data)

def display_thread():
    """打印线程：负责轮询并清空接收信箱"""
    while True:
        if not stm32_receive_mailbox.empty():
            print("\n--- [S] Lower Machine Status ---")
            while not stm32_receive_mailbox.empty():
                print(f"Data: {stm32_receive_mailbox.get()}")
        
        if not stm32_echo_mailbox.empty():
            print("\n--- [R] Lower Machine Echo ---")
            while not stm32_echo_mailbox.empty():
                print(f"Echo: {stm32_echo_mailbox.get()}")
        time.sleep(0.1)

def run_serial_node():
    """串口核心主循环"""
    print(f"--- 串口节点启动 ({PORT}) ---")
    try:
        with serial.Serial(PORT, BAUD, timeout=0.01) as ser:
            ser.flush()
            t = threading.Thread(target=display_thread, daemon=True)
            t.start()

            while True:
                # 1. 发送逻辑
                while not stm32_send_mailbox.empty():
                    msg = stm32_send_mailbox.get()
                    ser.write((msg + '\n').encode('utf-8'))

                # 2. 接收逻辑
                line_bytes = ser.readline()
                if line_bytes:
                    try:
                        line = line_bytes.decode('utf-8', errors='ignore').strip()
                        if line.startswith('[S]'):
                            stm32_receive_mailbox.put(line[3:])
                        elif line.startswith('[R]'):
                            stm32_echo_mailbox.put(line[3:])
                    except:
                        continue
                time.sleep(0.001)
    except Exception as e:
        print(f"串口错误: {e}")

# 只有直接运行此脚本时才会启动（防止被 import 时阻塞）
if __name__ == '__main__':
    run_serial_node()
