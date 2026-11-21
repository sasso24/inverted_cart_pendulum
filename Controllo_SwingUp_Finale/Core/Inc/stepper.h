/* Motore stepper, finecorsa e homing del carrello. */
#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"
#include "config_controllo.h"  
#define MIRCOSTEP CFG_MICROSTEP
#define MIN_FREQUENCY CFG_FREQ_MIN_HZ
#define MAX_FREQUENCY CFG_FREQ_MAX_HZ
#define STEPPER_ACCELERATION_HZ_PER_S CFG_RAMPA_HZ_S

// Tutti i punti di riferimento sono da dietro
#define DIR_FORWARD  GPIO_PIN_SET   // SInistra (da dietro)
#define DIR_BACKWARD GPIO_PIN_RESET

/* Inizializzazione del motore stepper. */
extern float max_starting_frequency;
extern volatile uint32_t step_count;
extern volatile uint32_t step_target;
extern volatile uint8_t stepper_running;
extern volatile GPIO_PinState stepper_direction;
extern volatile uint8_t arresto_finecorsa;
extern volatile uint8_t stop_al_fronte_step;
extern volatile GPIO_PinState direzione_dopo_arresto;
extern volatile float frequenza_applicata_hz;

/* Riferimento del carrello. */
extern volatile int32_t posizione_carrello_steps;
extern volatile int32_t posizione_carrello_min_steps;
extern volatile int32_t posizione_carrello_max_steps;
extern volatile uint32_t corsa_totale_steps;
extern volatile uint8_t riferimento_carrello_valido;
extern volatile uint8_t carrello_inizializzato;
extern volatile HAL_StatusTypeDef inizializzazione_esito;

uint8_t FinecorsaSinistro_Premuto(void);
uint8_t FinecorsaDestro_Premuto(void);
void Stepper_SetDirection(GPIO_PinState dir);
void Stepper_Stop(void);
void Stepper_StartIfEnabled(void);
void Stepper_ApplyFrequency(float frequency);
void Stepper_PreparaAvvioContinuo(void);
float Stepper_FrequenzaMinimaHz(void);
float Stepper_FrequenzaProgrammataHz(void);
void Stepper_MoveSteps(uint32_t steps);
void Stepper_Move(GPIO_PinState direction, uint32_t steps);
void Stepper_set_velocity(GPIO_PinState direction, float velocity);
void Inizializza(void);

#endif /* STEPPER_H */
