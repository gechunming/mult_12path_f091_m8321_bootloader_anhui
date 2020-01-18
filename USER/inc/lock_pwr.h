#ifndef __LOCK_PWR_H_
#define __LOCK_PWR_H_

enum LOCK_STATUS {
	LOCK_STATUS_CLOSED,
	LOCK_STATUS_OPENED,
	LOCK_STATUS_CHARGING,
	LOCK_STATUS_CHARGED,
};

enum LOCK_PROCESS_ACTION {
	LOCK_PROCESS_ACTION_PLUGIN,
	LOCK_PROCESS_ACTION_CHARGEFULL,
	LOCK_PROCESS_ACTION_PLUGOUT,
	LOCK_PROCESS_ACTION_DISCHARGE_COMMAND,
	LOCK_PROCESS_ACTION_TEMPERATURE_WARNING,
	LOCK_PROCESS_ACTION_OVERLOAD,
	LOCK_PROGESS_ACTION_LEAKAGE
};

#endif

