import serial
import time

# --- 串口配置参数 ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115204           
TIMEOUT = 0.01               

def run_serial_modified_echo():
    """
    接收数据，将收到的最后一个字节改为 'b' (0x62)，然后发回
    """
    print(f"尝试连接串口: {SERIAL_PORT}...")
    
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT
        )
        
        if ser.isOpen():
            print(f"串口 {SERIAL_PORT} 连接成功！")
            # print("逻辑：收到数据后，将最后一个字符改为 'b' 并返回...")
            ser.flushInput()
            
            while True:
                # 检查缓冲区是否有数据
                waiting_bytes = ser.in_waiting
                if waiting_bytes > 0:
                    # 1. 读取数据
                    incoming_data = ser.read(waiting_bytes)
                    
                    if incoming_data:
                        # 2. 将 bytes 转换为可修改的 bytearray
                        data_to_send = bytearray(incoming_data)
                        
                        # 3. 修改最后一个字节
                        # 'b' 的 ASCII 码是 0x62
                        old_val = data_to_send[-1]
                        data_to_send[-1] = ord('b') 
                        
                        # 4. 发送修改后的数据
                        ser.write(data_to_send)
                        
                        # --- 调试打印 ---
                        # 打印原始数据和修改后的数据（方便对比）
                        # try:
                        # orig_str = incoming_data.decode('utf-8', errors='ignore').strip()
                        # mod_str = data_to_send.decode('utf-8', errors='ignore').strip()
                        # print(f"[MODIFY] 原: {orig_str} -> 改: {mod_str}")
                        print(f"origin:{old_val}")
                        # except:
                        #     print(f"[MODIFY] 原(hex): {incoming_data.hex()} -> 改(hex): {data_to_send.hex()}")
                
                time.sleep(0.01)

    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n用户停止程序。")
    finally:
        if 'ser' in locals() and ser.isOpen():
            ser.close()
            print("串口已关闭。")

if __name__ == '__main__':
    run_serial_modified_echo()
