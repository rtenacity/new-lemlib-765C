#ifndef __EFFECTORS_H__
#define __EFFECTORS_H__

#include "main.h"
#include "config.h"
#include "chassis.h"
#include "controller.h"
#include "routes.h"

namespace Effectors
{
    void togglePiston();
    void toggleLowerStage();
    void toggleUpperStage();
    void toggleUpperStageSpeed();
    void toggleIntakeDirection();
    void toggleMatchLoader();
    void toggleMiddlePiston();

} // namespace Effectors

#endif