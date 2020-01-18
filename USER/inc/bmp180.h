#ifndef __BMP_180_H_
#define __BMP_180_H_
#define ALTITUDE_ACTION_H0_SET   0x01
#define ALTITUDE_ACTION_HIGH_DETECT 0x02
#define ALTITUDE_ACTION_LOWER_DETECT 0x04
#define ALTITUDE_ACTION_SPEED_DETECT 0x08				/* need to detect drop */


#define ALTITUDE_STATUS_HIGH  0x02
#define ALTITUDE_STATUS_DROP 0x04
#define ALTITUDE_STATUS_HIGH5  0x08

typedef enum _Bmp180ProcessEvent {
	BMP180_PROCESS_EVENT_RESETH0,
} Bmp180ProcessEvent;


void stop_measure_altitude(void);
void start_measure_altitude(uint32_t action);
bool IsInLowMode(void);

#endif

