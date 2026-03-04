import serial
import time
import sys

# --- 串口配置参数 ---
SERIAL_PORT = '/dev/ttyACM0'  # 你的串口设备名
BAUD_RATE = 115200           # 你的 STM32 配置的波特率
TIMEOUT = 0.5                # 读取超时时间 (秒)

def read_serial_data():
    """
    初始化串口并循环读取数据
    """
    print(f"尝试连接串口: {SERIAL_PORT} @ {BAUD_RATE}...")
  
    try:
        # 初始化串口对象
        # write_timeout=0 表示写入操作是非阻塞的，或者使用默认None
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT
        )
      
        # 确保串口已打开
        if ser.isOpen():
            print(f"串口 {SERIAL_PORT} 连接成功！等待数据...")
          
            # 清除输入缓冲区，避免读取到旧数据
            ser.flushInput()
          
            while True:
                # 尝试读取一行数据 (适用于STM32发送带换行符的数据，如 print 或 println)
                # 注意：如果STM32发送的是原始二进制数据，应使用 ser.read(size)
              
                # 假设STM32发送的是 ASCII 文本，并以 \n 结尾
                line = ser.readline()
              
                if line:
                    try:
                        # 尝试将字节数据解码为字符串
                        data_str = line.decode('utf-8').strip()
                        print(f"[RECV] {time.strftime('%H:%M:%S')} - {data_str}")
                      
                        # 在这里可以添加数据解析逻辑（例如：将字符串转换为数字）
                      
                    except UnicodeDecodeError:
                        # 如果是二进制数据或解码失败，打印原始字节
                        print(f"[RECV] {time.strftime('%H:%M:%S')} - Raw Bytes: {line}")
                      
                else:
                    # 如果设置了 timeout，并且在规定时间内没有收到数据
                    # print("等待数据...")
                    time.sleep(0.01) # 稍作等待，避免CPU占用过高
                  
    except serial.SerialException as e:
        print(f"错误: 无法打开或连接串口 {SERIAL_PORT}. 请检查设备是否连接或权限是否正确。")
        print(f"详细错误: {e}")
        # 常见错误：[Errno 13] Permission denied: '/dev/ttyACM0'
        # 解决方法见下文
    except KeyboardInterrupt:
        print("\n程序终止。关闭串口...")
        if 'ser' in locals() and ser.isOpen():
            ser.close()
          
if __name__ == '__main__':
    read_serial_data()
