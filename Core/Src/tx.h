#ifndef SRC_TX_H_
#define SRC_TX_H_

typedef enum tx_node {NORTH = 0, SOUTH, EAST, WEST} tx_node;
typedef enum pulse_state {PULSE_IDLE, PULSE_BUSY, PULSE_OK, PULSE_ERROR} pulse_state;
pulse_state issue_pulse(const tx_node node);
pulse_state get_pulse_state();

#endif /* SRC_TX_H_ */
