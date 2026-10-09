#ifndef MB_SERVER_H
#define MB_SERVER_H

#include <stdbool.h>
#include <stdint.h>

void mb_server_init(void);

/* Handle at most one request, then run deferred actions and the
   communication timeout */
void mb_server_poll(uint32_t now_ms);

/* Communication timeout tripped (status bit 0) */
bool mb_server_comm_timeout(void);

#endif
