/* Hardware-in-the-loop: 1 = la scheda riceve dal PC le misure simulate,
 * esegue stimatore e rete e restituisce il comando. 
 * 0 = funzionamento normale.
 */
#ifndef HIL_H
#define HIL_H

#include "main.h"

#ifndef HIL_MODE
#define HIL_MODE 0
#endif

#if HIL_MODE
void Hil_Esegui(void);
#endif

#endif 
