/* Controllo del pendolo: pulsante B1, avvio e fault, rampa del
 * motore e predizione dello stimatore, campionamento e rete.
 */
#ifndef CONTROLLO_H
#define CONTROLLO_H

#include "main.h"

/* Periodo della rampa del motore e di predizione dello stimatore */
#define MOTOR_PERIOD_MS          2U

extern volatile uint8_t controllo_abilitato;
/* Errori: 0=OK, 1=init rete, 2=inferenza,
 * 3=NaN/Inf, 4=finecorsa/limite, 5=deadline, 6=avvio PWM, 7=calibrazione theta,
 * 9..11 = ERRORE_* in controllo.c. */
extern volatile uint32_t controllo_errore;
/* Abilitazione del controllo tramite B1 */
extern volatile uint8_t pulsante_inizializzato;
/* Limiti di corsa del controllo in passi. */
extern volatile int32_t limite_controllo_min_steps;
extern volatile int32_t limite_controllo_max_steps;
/* Comando della rete in Hz. */
extern volatile float frequenza_richiesta_hz;

extern volatile float velocita_angolare_rad_s;
extern uint32_t ultimo_campione_ms;
extern int32_t ultimo_carrello_steps;
extern float ultimo_theta_rad;

/* Calcolo dei limiti di corsa del controllo. */
void Calcola_LimitiControllo(void);
/* Avvio richiesto da B1. */
void Controllo_AccettaAvvio(void);
/* Campionamento, stima e esecuzione della rete ogni CONTROL_PERIOD_MS. */
void Controllo_Esegui(void);

/* Da SysTick_Handler (stm32f4xx_it.c). */
void Controllo_ButtonTick(void);
void Controllo_MotorTick(void);
void Controllo_StimaTick(void);

/* Arresto immediato con codice di errore da qualsiasi contesto. */
void Controllo_Fault(uint32_t errore);
/* Arresto da pulsante o Error_Handler. */
void Controllo_Disabilita(void);

#endif /* CONTROLLO_H */
