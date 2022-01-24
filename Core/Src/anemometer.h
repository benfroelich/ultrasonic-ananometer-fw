#ifndef SRC_ANEMOMETER_H_
#define SRC_ANEMOMETER_H_

typedef enum {NORTH = 0, SOUTH, EAST, WEST} tx_node;
typedef enum  {
	ANEMOMETER_RESULT_OK = 0, ANEMOMETER_RESULT_ERROR
} anemometer_status_t;
typedef enum {
	ANEMOMETER_IDLE,
	ANEMOMETER_BUSY,
	ANEMOMETER_UNINIT,
	ANEMOMETER_ERROR
} anemometer_state_t;
anemometer_status_t anemometer_init();
anemometer_status_t anemometer_issue_pulse(const tx_node node);
anemometer_state_t anemometer_get_state();

#endif /* SRC_ANEMOMETER_H_ */
