#include "usbreceive_task.h"
#include "dvc_referee.h"

void USBRECEIVE_TASK(void const *argument)
{
    for (;;) {
        uint8_t *reply; uint16_t len;
        pc_referee_status_t status;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
        taskENTER_CRITICAL();
        status.health = referee.Robot_Status.HP;
        status.fire_number = referee.Robot_Remaining_Ammo.Booster_17mm;
        status.heat = (float)referee.Robot_Power_Heat.shooter_17mm_barrel_heat;
        status.zone_is_ours = 0; /* Region/field not specified by host protocol. */
        status.stop = referee.Game_Status.Stage_Enum != 4;
        taskEXIT_CRITICAL();
        pc_set_referee_status(&status);
        /* Bounded RX work and at most one TX attempt per wakeup. BUSY leaves
         * the reply queued; CDC owns a copy until transmit completes. */
        pc_process_rx();
        taskENTER_CRITICAL();
        if (pc_tx_peek(&reply, &len) && CDC_Transmit_FS(reply, len) == USBD_OK)
            pc_tx_pop();
        taskEXIT_CRITICAL();
    }
}
