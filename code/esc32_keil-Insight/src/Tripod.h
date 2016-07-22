#include "main.h"

#define DEFAULT_TRIPOD_X00MSEC 75 //7.5s
#define TRIPOD_ARM1 1
#define TRIPOD_ARM2 2
#define TRIPOD_ARMBOTH 3
extern void tripodInit(void);
extern void tripodFirstAction(void);
extern void tripodstop(void);
extern void tripodAdcAction(int arm);
extern void tripodTimerAction(void);
extern void pwmNewInput(uint16_t setpoint);
