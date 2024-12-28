#ifndef MAINSTATEHANDLER_H
#define MAINSTATEHANDLER_H

typedef enum
{
    INIT_STA,
    INIT_CONTROLLER,
    DEINIT_STA,
    INIT_AP,
    DEINIT_AP,
    INIT_HTTP,
    DEINIT_HTTP,
    IDLE
} mainState_t;

mainState_t mainStateHandler_getMainState(void);
void mainStateHandler_setMainState(mainState_t state);
#endif // MAINSTATEHANDLER_H