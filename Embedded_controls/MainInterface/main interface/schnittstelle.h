#ifndef schnittstelle
#define schnittstelle

#include "main_interface.h"
#include "lib/gyro.h"
#include "schnittstelle.h"

//#include "stackmon.h"

void initSchnittstelle();
void waitForSerialCommandAndExecuteAndSendStatus();
void remove_spaces(char* s);
void sendStatus(struct Status status);

#endif
