/* Rete X-CUBE-AI, ingressi dalla stima e storia degli ingressi (rete.c).
 * L'inferenza la esegue il chiamante (Controllo_Esegui in controllo.c, Hil_Elabora)
 * con ai_network_run(rete, rete_input, rete_output).
 */
#ifndef RETE_H
#define RETE_H

#include "main.h"
#include "network.h"
#include "stima.h"
#include "config_controllo.h"  /* generato da esporta_firmware.py */

extern volatile uint8_t rete_pronta;
extern volatile float rete_osservazione[6];
extern volatile float rete_azione;
extern volatile uint32_t rete_inferenze;
extern volatile uint32_t rete_durata_ms;
extern volatile ai_error rete_ultimo_errore;
extern ai_handle rete;
extern ai_buffer *rete_input;
extern ai_buffer *rete_output;
/* Rete attiva fra quelle di reti_candidate.c: scrivere rete_scelta dal
 * debugger a controllo fermo, rete_attiva conferma il cambio. */
extern volatile uint32_t rete_scelta;
extern volatile uint32_t rete_attiva;

/* Crea la rete rete_attiva e imposta i guadagni dello stimatore.
 * Ritorna 1 se pronta (rete_pronta = 1), 0 in caso di errore. */
uint8_t Rete_Init(void);
/* Solo dal main e a controllo fermo: ricrea la rete se rete_scelta e' cambiata. */
void Rete_CambiaSeRichiesto(void);

/* I sei ingressi della rete dalla stima: stesse scale dell'addestramento. */
void Rete_Osservazione(const Stima *st, float applied_hz, volatile float obs[6]);
/* Uscita della rete [-1,1] -> frequenza STEP firmata, zona morta compresa. */
float Rete_AzioneInHz(float action);

#if CFG_STORIA_N > 0
/* 1 = al prossimo avvio la storia riparte dai primi ingressi. */
extern volatile uint8_t storia_da_azzerare;
void Storia_Reset(const volatile float obs[6]);
void Storia_Aggiungi(const volatile float obs[6], float azione);
void Storia_Ingressi(ai_float *input);
#endif

#endif /* RETE_H */
