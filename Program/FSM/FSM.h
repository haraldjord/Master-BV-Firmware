

#ifndef _FSM_H_
#define _FSM_H_

#include "main.h"
#include "timers.h"

/** @file
 *
 * @defgroup fsm FSM program file
 * @{
 * @ingroup Program
 *
 * @brief Contain FSM module with related structures and functions.
 */

/**@brief Type definition of system state enumeration
*/
typedef enum
{
  INIT, IDLE, CONFIGURE, MISSION, PICKUP, SLEEP, LOWPOWER, FAILURE
} systemState_t;

/**@brief Type definition of system events.
*
*@attention Depricated, not in use
*/
typedef enum
{
  Initialize,
  Init_Finished,
  HallEffectInterrupt,
  BLEInterrupt,
  Failure,
  MissionStart,
  MissionFinished,
  MissionAbort,
  Sleep

} systemEvent_t;

/**@brief Structure of FSM specific variables.
*/
typedef struct
{
  systemState_t state, nextState; /**< state enumeration instances to hold current state and nextState, respectively.*/
  bool stateInitialized;/**< Flag to signal that current state is initialized.*/
  bool IDLEstateTimeout; /**< Flag to signal idle state transition to sleep state.*/
  bool BLEgotoIdle;/**< Flag to signal BLE command state transition to idle state.*/
  bool BLEgotoConfig;/**< Flag to signal BLE command state transition to configure state.*/
  bool BLEstartMission;/**< Flag to signal BLE command state transition to mission state.*/
  bool hallEffectButton;/**< Flag to signal hall effect button interrupt.*/
}FSM_t;

extern FSM_t fsm;

/**@brief Main FSM loop
*/
void FSM(void);

/**@brief Failure state, vehicle is floated to surface, SD card is unmounted
*
* @note System requre reset to recover from failure state
*
*/
void FAILUREstate(void);

/**@brief Low Power state, vehicle is floated to surface, SD card is unmounted
* 
* @note stay in Low Power state until system is reset to avoid mission to continue and pull the vehicle under water again.
*/
void LOWPOWERstate(void);



#endif // _FSM_H_

/** @} */