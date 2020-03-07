/**
 * Copyright (C) 2019 Piers Titus van der Torren
 *
 * This file is part of Striso Control.
 *
 * Striso Control is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Striso Control is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Striso Control. If not, see <http://www.gnu.org/licenses/>.
 */
#include "messaging.h"
#include "ch.h"
#include "hal.h"

#define BUFFERSIZE 240
static int msg_buffer[BUFFERSIZE];
static int msg_read = 0;
static int msg_write = 0;
static mutex_t msg_lock;
static thread_t *tpMsg = NULL;
int underruns = 0;

int msgSend(int size, int* msg) {
  chMtxLock(&msg_lock);

  int n;
  int old_write = msg_write;
  msg_buffer[msg_write] = size;
  msg_write = (msg_write + 1) % BUFFERSIZE;
  for (n = 0; n < size; n++) {
    if (msg_read == msg_write) {
      //msg_read = (msg_read + msg_buffer[msg_read] + 1) % BUFFERSIZE;
      msg_write = old_write;
      underruns++;
      chMtxUnlock(&msg_lock);
      return 1;
    }
    msg_buffer[msg_write] = msg[n];
    msg_write = (msg_write + 1) % BUFFERSIZE;
  }

  chMtxUnlock(&msg_lock);

  // Wake up msgGet thread
  chSysLock();
  if (tpMsg != NULL) {
    chSchReadyI(tpMsg);
    tpMsg = NULL;
  }
  chSysUnlock();
  return 0;
}

int msgGet(int maxsize, int* msg) {
  chMtxLock(&msg_lock);

  if (msg_read == msg_write) {
    chMtxUnlock(&msg_lock);
    // wait for new messages to arrive
    chSysLock();
    tpMsg = chThdGetSelfX();
    chSchGoSleepS(CH_STATE_SUSPENDED);
    chSysUnlock();
    chMtxLock(&msg_lock);
  }

  int size = msg_buffer[msg_read];

  if (size > maxsize) {
    chMtxUnlock(&msg_lock);
    return -10;
  }
  msg_read = (msg_read + 1) % BUFFERSIZE;

  int n;
  for (n = 0; n < size; n++) {
    if (msg_read == msg_write) {
      chMtxUnlock(&msg_lock);
      return -1;
    }
    msg[n] = msg_buffer[msg_read];
    msg_read = (msg_read + 1) % BUFFERSIZE;
  }

  chMtxUnlock(&msg_lock);
  return size;
}

void MessagingInit(void) {
  // init msg mutex
  chMtxObjectInit(&msg_lock);
}
