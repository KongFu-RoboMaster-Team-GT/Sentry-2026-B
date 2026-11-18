/* Actual CDC adapter; only USB stack and RTOS notification APIs are simulated. */
#define __USBD_CDC_IF_H__
#define APP_RX_DATA_SIZE 2048
#define APP_TX_DATA_SIZE 2048
#define USBD_OK 0u
#define USBD_BUSY 1u
#define USBD_FAIL 2u
#define USBD_STATE_CONFIGURED 3u
enum { CDC_SEND_ENCAPSULATED_COMMAND, CDC_GET_ENCAPSULATED_RESPONSE,
       CDC_SET_COMM_FEATURE, CDC_GET_COMM_FEATURE, CDC_CLEAR_COMM_FEATURE,
       CDC_SET_LINE_CODING, CDC_GET_LINE_CODING, CDC_SET_CONTROL_LINE_STATE, CDC_SEND_BREAK };
typedef void *osThreadId;
typedef int BaseType_t;
#define pdFALSE 0
#define UNUSED(value) ((void)(value))
typedef struct { void *pClassData; uint8_t dev_state; } USBD_HandleTypeDef;
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct {
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;
USBD_HandleTypeDef hUsbDeviceFS;
osThreadId usbreceive_taskHandle;
static USBD_CDC_HandleTypeDef test_cdc;
static uint8_t *usb_tx_pointer, *usb_rx_pointer;
static uint32_t usb_tx_length, usb_rearms, usb_notifications, usb_notify_rearms;
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device, uint8_t *p, uint32_t n)
{ usb_tx_pointer = p; usb_tx_length = n; return USBD_OK; }
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *device, uint8_t *p)
{ usb_rx_pointer = p; return USBD_OK; }
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *device)
{ test_cdc.TxState = 1; return USBD_OK; }
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *device)
{ ++usb_rearms; return USBD_OK; }
static void vTaskNotifyGiveFromISR(osThreadId handle, BaseType_t *woken)
{ CHECK(handle != NULL); ++usb_notifications; usb_notify_rearms = usb_rearms; *woken = 1; }
static void test_usb_yield(BaseType_t woken)
{ if (woken) CHECK(usb_rearms > usb_notify_rearms); }
#define portYIELD_FROM_ISR(woken) test_usb_yield(woken)
#include "../USB_DEVICE/App/usbd_cdc_if.c"
#undef portYIELD_FROM_ISR

static void test_usb_transport(void)
{
    uint8_t query[] = {0xAA,0x55,2,0,2};
    uint8_t aim_data[] = {0xAA,0x55,1,9,0,0,0,0,0,0,0,0,0,10};
    uint8_t *p; uint16_t n; uint32_t received = 5;
    tick = 1;
    hUsbDeviceFS.pClassData = NULL; hUsbDeviceFS.dev_state = 0;
    CHECK(CDC_Transmit_FS(query, 5) == USBD_FAIL);
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    CHECK(CDC_Transmit_FS(query, 5) == USBD_FAIL);
    hUsbDeviceFS.pClassData = &test_cdc;
    CHECK(CDC_Transmit_FS(NULL, 5) == USBD_FAIL);
    CHECK(CDC_Transmit_FS(query, APP_TX_DATA_SIZE + 1) == USBD_FAIL);
    CHECK(USBD_Interface_fops_FS.Init() == USBD_OK);
    pc_process_rx();
    /* LOWER/no task still rearms RX, without passing NULL to the scheduler. */
    usbreceive_taskHandle = NULL;
    CHECK(USBD_Interface_fops_FS.Receive(query, &received) == USBD_OK);
    CHECK(usb_rearms == 1 && usb_notifications == 0 && rx_head == rx_tail);
    usbreceive_taskHandle = (void *)1;
    CHECK(USBD_Interface_fops_FS.Receive(query, &received) == USBD_OK);
    CHECK(usb_notifications == 1 && usb_rx_pointer == query);
    memset(query, 0, sizeof(query)); pc_process_rx();
    CHECK(pc_tx_peek(&p, &n) && n == 19 && p[2] == 2);
    CHECK(CDC_Transmit_FS(p, n) == USBD_OK);
    CHECK(usb_tx_pointer != p && usb_tx_length == 19 && usb_tx_pointer[2] == 2);
    pc_tx_pop();
    /* Queue memory may advance/reuse while the USB IN frame remains immutable. */
    CHECK(usbreceive(aim_data, sizeof(aim_data)));
    CHECK(pc_tx_peek(&p, &n) && n == 5 && p[2] == 1);
    CHECK(CDC_Transmit_FS(p, n) == USBD_BUSY);
    CHECK(usb_tx_length == 19 && usb_tx_pointer[2] == 2);
    CHECK(pc_tx_peek(&p, &n));
    test_cdc.TxState = 0; /* Simulate stack completing the first transfer. */
    CHECK(CDC_Transmit_FS(p, n) == USBD_OK); pc_tx_pop();
    CHECK(usb_tx_length == 5 && usb_tx_pointer[2] == 1 && usb_tx_pointer[4] == 1);
    CHECK(!pc_tx_peek(&p, &n));
    CHECK(USBD_Interface_fops_FS.DeInit() == USBD_OK);
    pc_process_rx(); CHECK(!pc_fire_requested());
    hUsbDeviceFS.pClassData = NULL; hUsbDeviceFS.dev_state = 0;
    test_cdc.TxState = 0; usbreceive_taskHandle = NULL;
}
