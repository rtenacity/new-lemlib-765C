#ifndef __ARM_H__
#define __ARM_H__

#include "main.h"
#include "config.h"
#include "chassis.h"
#include "routes.h"
#include "pid.h"

namespace Arm
{
    void init();
    void setTarget(int target);
} // namespace Arm

#endif