//
// Definitions for loco profiles
//

// build configs
//#define DAPOL_B4        // build for Dapol B4
#define DAPOL_58XX    // build for Dapol 58XX
//#define DAPOL_09      // build for Dapol 09

#ifdef DAPOL_58XX
#define LOCO_ID      1    // 1 = 14/58xx, 2 = 08/09, 3 = B4
#define FIREBOX_LEDS 1    // red and yellow LEDs in firebox
// motor defines
#define FWD_DEAD_BAND 130 // minimum power to move motor forward (used in power mapping)
#define REV_DEAD_BAND 140 // minimum power to move motor backwards (used in power mapping)
#define MAX_POWER 255     // desired max power (used in power mapping)
#define INITIAL_NUDGE 120.0 // first cycle 'nudge' after stop
#define NUDGE_DECAY   2.0   // 'nudge' decay rate per cycle
// PWM init
#define PWM_BITS  8
#define PWM_FREQ  10000
#endif

#ifdef DAPOL_09
#define LOCO_ID      2    // 1 = 14xx, 2 = 08/09, 3 = B4
#define FIREBOX_LEDS 0    // red and yellow LEDs in firebox
// motor defines
#define FWD_DEAD_BAND 140 // minimum power to move motor forward (used in power mapping)
#define REV_DEAD_BAND 130 // minimum power to move motor backwards (used in power mapping)
#define MAX_POWER 255     // desired max power (used in power mapping)
#define INITIAL_NUDGE 120.0 // first cycle 'nudge' after stop
#define NUDGE_DECAY   2.0   // 'nudge' decay rate per cycle
// PWM init
#define PWM_BITS  8
#define PWM_FREQ  10000
#endif

#ifdef DAPOL_B4
#define LOCO_ID      3    // 1 = 14xx, 2 = 08/09, 3 = B4
#define FIREBOX_LEDS 1    // red and yellow LEDs in firebox
// motor defines
#define FWD_DEAD_BAND 120// minimum power to move motor forward (used in power mapping)
#define REV_DEAD_BAND 135 // minimum power to move motor backwards (used in power mapping)
#define MAX_POWER 255     // desired max power (used in power mapping)
#define INITIAL_NUDGE 120.0 // first cycle 'nudge' after stop
#define NUDGE_DECAY   2.0   // 'nudge' decay rate per cycle
// PWM init
#define PWM_BITS  8
#define PWM_FREQ  10000
#endif
