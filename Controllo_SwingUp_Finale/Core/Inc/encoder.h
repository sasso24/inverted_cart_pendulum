/* Encoder incrementale del pendolo (TIM2 in modalita' encoder, 32 bit).
 * Angolo in (-180, +180] gradi, 0 = pendolo in alto; il riferimento viene
 * da Inizializza_Theta (pendolo appeso e fermo = 180 gradi) o da
 * Theta_Ricalibra. Solo dal main, mai da interrupt.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "config_controllo.h"  /* generato da esporta_firmware.py */

#define PI_F         3.14159265358979323846f

extern volatile float theta_deg;
extern volatile float theta_rad;
/* Le inizializzazioni statiche di theta non sono ancora un riferimento valido. */
extern volatile uint8_t theta_inizializzato;
/* Variazione dell'ultima lettura e tempo di quiete (una lettura per periodo). */
extern volatile int32_t encoder_ultimo_delta;
extern volatile uint32_t encoder_quiete_ms;
extern volatile uint32_t encoder_salti;          /* salti scartati, per il debugger */
extern volatile uint32_t theta_ricalibrazioni;

HAL_StatusTypeDef Encoder_Init(void);
void Leggi_theta(void);
uint8_t Encoder_Campiona(uint32_t elapsed_ms, uint32_t periodo_ms);
HAL_StatusTypeDef Inizializza_Theta(void);
void Theta_Ricalibra(void);

#endif /* ENCODER_H */
