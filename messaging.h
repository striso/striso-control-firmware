#ifndef _MESSAGING_H_
#define _MESSAGING_H_

void MessagingInit(void);

int msgSend(int size, int* msg);
int msgGet(int maxsize, int* msg);

extern int underruns;

#endif
