import threading
import time
# 核心修改点：引用底层脚本的函数和启动函数
from stm_mailbox_v0_7_1_11 import push_to_send_mailbox, run_serial_node

def start_interactive_interface():
    print("=== STM32 串口交互终端已启动 ===")
    print("输入任何内容并回车即可发送至下位机 (输入 'quit' 退出)")
    
    # 1. 在后台启动串口驱动线程
    serial_thread = threading.Thread(target=run_serial_node, daemon=True)
    serial_thread.start()
    
    # 延迟一小会儿确保串口已打开
    time.sleep(1)

    # 2. 主线程：Input 阻塞并按回车传入信箱
    try:
        while True:
            user_msg = input("\n[发送] > ")
            
            if user_msg.lower() == 'quit':
                break
            
            if user_msg:
                # 调用底层脚本的函数
                push_to_send_mailbox(user_msg)
                
    except KeyboardInterrupt:
        print("\n交互已终止")

if __name__ == '__main__':
    start_interactive_interface()
