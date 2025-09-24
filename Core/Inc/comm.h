#ifndef INC_COMM_H_
#define INC_COMM_H_

#include <stdint.h>
#include <stdbool.h>

// Appelé à chaque réception de données USB
void USBProtocol_Receive(uint8_t* Buf, uint32_t Len);

// À appeler régulièrement pour renvoyer les évènements non ACK
void check_event_timeouts(void);

// À appeler pour générer un événement à envoyer à la Rasp
void queue_event(const char *evt, const char *info);

// ==== Stubs à implémenter toi-même ====

int get_servo_position(int id); // Renvoie la position actuelle du servo n
bool move_servo_to_position(int servo_id, int pos_id); // déplace le servo
bool arm_tirette();
bool disarm_tirette();


#endif /* INC_COMM_H_ */
