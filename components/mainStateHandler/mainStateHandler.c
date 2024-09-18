#include <stdio.h>
#include "mainStateHandler.h"

extern mainState_t mainState;

mainState_t mainStateHandler_getMainState(void)
{
    return mainState;
}

void mainStateHandler_setMainState(mainState_t state)
{
    mainState = state;
}
