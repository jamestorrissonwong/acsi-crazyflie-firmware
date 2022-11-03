/*

Multiranger Test 

Meant to statically read in position values to determine sensor noise. 

James Wong
v1 -- 10/31/22

*/

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#include "multiranger_test.h"

// States
typedef enum
{
  idle,
  lowUnlock,
  unlocked,
  stopping
} StateOuterLoop;

static StateOuterLoop stateOuterLoop = idle;

void appMain()
{

    // M2T == milliseconds to ticks
    vTaskDelay(M2T(3000));

    logVarId_t idUp = logGetVarId("range", "up");
    logVarId_t idLeft = logGetVarId("range", "left");
    logVarId_t idRight = logGetVarId("range", "right");
    logVarId_t idFront = logGetVarId("range", "front");
    logVarId_t idBack = logGetVarId("range", "back");
    logVarId_t idDown = logGetVarId("range", "down");
    
    paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
    paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");

    while (1){
        uint8_t positioningInit = paramGetUint(idPositioningDeck);
        uint8_t multirangerInit = paramGetUint(idMultiranger);

        if (stateOuterLoop == unlocked)
        {

            float frontRange = (float)logGetUint(idFront) / 1000.0f;
            float leftRange = (float)logGetUint(idLeft) / 1000.0f;
            float rightRange = (float)logGetUint(idRight) / 1000.0f;
            float backRange = (float)logGetUint(idBack) / 1000.0f;
            float upRange = (float)logGetUint(idUp) / 1000.0f;
            float downRange = (float)logGetUint(idDown) / 1000.0f;

            if (cmdHeight < spHeight - 0.2f)
            {
                stateOuterLoop = stopping;
                DEBUG_PRINT("X\n");
            }
        }
        else {
            // Handling locking and unlocking
            if (stateOuterLoop == stopping && up > stoppedTh)
            {
                DEBUG_PRINT("%i", up);
                stateOuterLoop = idle;
                DEBUG_PRINT("S\n");
            }

            // If the up multiranger is activated for the first time, prepare to be unlocked
            if (up < unlockThLow && stateOuterLoop == idle && up > 0.001f)
            {
                DEBUG_PRINT("Waiting for hand to be removed!\n");
                stateOuterLoop = lowUnlock;
            }

            // Unlock CF if hand above is removed, and if the positioningdeckand multiranger deck is initalized.
            if (up > unlockThHigh && stateOuterLoop == lowUnlock && positioningInit && multirangerInit)
            {
                DEBUG_PRINT("Unlocked!\n");
                stateOuterLoop = unlocked;
            }

            // Stop the crazyflie with idle or stopping state
            if (stateOuterLoop == idle || stateOuterLoop == stopping)
            {
                memset(&setpoint, 0, sizeof(setpoint_t));
                commanderSetSetpoint(&setpoint, 3);
            }
        }

    }

}

LOG_GROUP_START(app)
LOG_ADD(LOG_FLOAT, frontRange, &frontRange)
LOG_ADD(LOG_FLOAT, leftRange, &leftRange)
LOG_ADD(LOG_FLOAT, rightRange, &rightRange)
LOG_ADD(LOG_FLOAT, backRange, &backRange)
LOG_ADD(LOG_FLOAT, upRange, &upRange)
LOG_ADD(LOG_FLOAT, downRange, &downRange)
LOG_ADD(LOG_UINT8, stateOuterLoop, &stateOuterLoop)
LOG_GROUP_STOP(app)