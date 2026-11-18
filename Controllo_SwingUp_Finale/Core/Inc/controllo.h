/* Controllo del pendolo (controllo.c): pulsante B1, avvio e fault, rampa del
 * motore e predizione dello stimatore in SysTick, campionamento e rete nel
 * ciclo principale.
 */
#ifndef CONTROLLO_H
#define CONTROLLO_H

#include "main.h"

/* Periodo di SysTick per rampa del motore e predizione dello stimatore
 * (usato anche da hil.c). */
#define MOTOR_PERIOD_MS          2U

/* Il main e' l'unico contesto che puo' abilitare il controllo. */
extern volatile uint8_t controllo_abilitato;
/* Diagnostica osservabile dal debugger: 0=OK, 1=init rete, 2=inferenza,
 * 3=NaN/Inf, 4=finecorsa/limite, 5=deadline, 6=avvio PWM, 7=calibrazione theta,
 * 9..11 = ERRORE_* in controllo.c. */
extern volatile uint32_t controllo_errore;
/* 1 dopo l'homing: da qui B1 comanda il controllo (usato da stepper.c). */
extern volatile uint8_t pulsante_inizializzato;
/* Limiti di corsa del controllo in passi (controllati anche nell'ISR STEP). */
extern volatile int32_t limite_controllo_min_steps;
extern volatile int32_t limite_controllo_max_steps;
/* Comando della rete in Hz, scritto anche da hil.c. */
extern volatile float frequenza_richiesta_hz;
/* Stato del campionamento, inizializzato dal main dopo l'homing. */
extern volatile float velocita_angolare_rad_s;
extern uint32_t ultimo_campione_ms;
extern int32_t ultimo_carrello_steps;
extern float ultimo_theta_rad;

/* Dal main, dopo l'homing: limiti di corsa del controllo dalla corsa misurata. */
void Calcola_LimitiControllo(void);
/* Dal ciclo principale, a controllo fermo: avvio richiesto da B1. */
void Controllo_AccettaAvvio(void);
/* Dal ciclo principale: campionamento, stima e rete ogni CONTROL_PERIOD_MS. */
void Controllo_Campiona(void);

/* Da SysTick_Handler (stm32f4xx_it.c), in quest'ordine. */
void Controllo_ButtonTick(void);
void Controllo_MotorTick(void);
void Controllo_StimaTick(void);

/* Arresto immediato con codice di errore; da qualsiasi contesto. */
void Controllo_Fault(uint32_t errore);
/* Arresto senza errore (pulsante, Error_Handler). */
void Controllo_Disabilita(void);

#endif /* CONTROLLO_H */
