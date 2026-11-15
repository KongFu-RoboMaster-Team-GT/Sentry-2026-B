#ifndef TEST_REFEREE_H
#define TEST_REFEREE_H
#include <stdint.h>
/* Only the external referee input is simulated; heat_control.c is real. */
enum { Referee_Status_DISABLE = 0, Referee_Status_ENABLE = 1 };
typedef struct {
    uint32_t Referee_Status;
    struct {
        uint16_t shooter_42mm_barrel_heat, shooter_17mm_barrel_heat;
    } Robot_Power_Heat;
} Referee_System_t;
extern Referee_System_t referee;
#endif
