#ifndef __diagport_h__
#define __diagport_h__   1

#include <stdbool.h>

void DIAGPORT_initialize( void );
void DIAGPORT_service( void );

void DIAGPORT_transmit_string(char *);

void MAIN_enable_health_led( bool );

#endif
