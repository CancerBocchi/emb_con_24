#ifndef __AVOID_H__
#define __AVOID_H__

#include "headfile.h"


extern osSemaphoreId_t *avoid_sem;


void avoid_run();
int Should_Avoid();

#endif // !__AVOID_H__
