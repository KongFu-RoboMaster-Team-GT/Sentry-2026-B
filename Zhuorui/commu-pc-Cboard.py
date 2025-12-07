import serial
import time

# --- 配置参数 ---
PORT = 'COM9'  
BAUDRATE = 115200

try:
    # 初始化串口
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,  # 读取超时1秒
        xonxoff=False,
        rtscts=False,
        dsrdtr=False
    )
    
    if ser.isOpen():
        print(f"Serial port {PORT} opened successfully at {BAUDRATE} baud.")
        ser.reset_input_buffer()  # 清空输入缓冲区
        ser.reset_output_buffer() # 清空输出缓冲区
    else:
        print(f"Failed to open port {PORT}.")
        exit()

    print("--- Starting to receive data ---")
    print("Press Ctrl+C to stop...")
   
    while True:
        # 检查是否有数据可读
        if ser.in_waiting > 0:
            # 读取所有可用数据
            data = ser.read(ser.in_waiting)
            
            try:
                # 尝试解码为字符串
                decoded_data = data.decode('utf-8', errors='ignore')
                print(f"Received ({len(data)} bytes): {decoded_data}", end='')
            except Exception as e:
                print(f"Decode error: {e}")
                print(f"Raw bytes (hex): {data.hex()}")
       
        time.sleep(0.1)  # 适当延时

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    if 'ser' in locals() and ser.isOpen():
        ser.close()
        print("Serial port closed.")