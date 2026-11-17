/* Hardware-in-the-loop: 1 = la scheda NON muove il motore e non legge
 * l'encoder; riceve dal PC (hil.py, USART2 dell'ST-LINK) le misure simulate,
 * esegue stimatore e rete e restituisce il comando. 0 = funzionamento normale.
 * Rimettere a 0 prima di collegare il motore.
 * Unica definizione: la leggono sia main.c sia hil.c.
 */
#ifndef HIL_H
#define HIL_H

#include "main.h"

#ifndef HIL_MODE
#define HIL_MODE 1
#endif

#if HIL_MODE
/* Non ritorna. Chiamare dopo Rete_Init(), al posto di homing e controllo. */
void Hil_Esegui(void);
#endif

#endif /* HIL_H */
