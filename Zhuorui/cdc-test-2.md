int main(void)
{
	
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
  
	MX_USB_DEVICE_Init();
	USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
	USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
	USBD_Start(&hUsbDeviceFS);

  /* USER CODE BEGIN 2 */
  uint32_t last_tx_time = HAL_GetTick();
  char *test_msg = "STM32 CDC Test: Hello from Downlink!\r\n";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   /* 1. 周期性主动发送数据给上位机 */
    if (HAL_GetTick() - last_tx_time > 1000) // 每隔 1000ms 发送一次
    {
        // 检查 USB 是否已连接并发送数据
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
            CDC_Transmit_FS((uint8_t *)test_msg, strlen(test_msg));
        }
        last_tx_time = HAL_GetTick();
    }

    /* 2. 处理接收到的数据 (Echo 回环测试) */
    // UserRxLen 在接收回调函数 (CDC_Receive_FS) 中被设置
    if (UserRxLen > 0)
    {
        // 接收到的数据在 UserRxBufferFS 中，长度为 UserRxLen
       
        // 构造回环消息
        char echo_prefix[] = "Echo: ";
        uint8_t tx_buffer[APP_RX_DATA_SIZE + sizeof(echo_prefix)];
       
        memcpy(tx_buffer, echo_prefix, sizeof(echo_prefix) - 1); // 复制前缀
        memcpy(tx_buffer + sizeof(echo_prefix) - 1, UserRxBufferFS, UserRxLen); // 复制接收到的数据
       
        // 将数据发回给上位机
        CDC_Transmit_FS(tx_buffer, UserRxLen + sizeof(echo_prefix) - 1);
       
        // 清零长度标志，等待下一次接收
        UserRxLen = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
