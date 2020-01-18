#ifndef __CABLELOCK_H
#define __CABLELOCK_H
typedef enum _CableLockMode {
	CABLE_LOCK_MODE_LOCKED,
	CABLE_LOCK_MODE_UNLOCKED,
} CableLockMode;


typedef enum _ProcessCableEvent {
	PROCESS_CABLE_EVENT_UNLOCK,
}ProcessCableEvent;

CableLockMode getCableLockModeAlive(void);
#endif

