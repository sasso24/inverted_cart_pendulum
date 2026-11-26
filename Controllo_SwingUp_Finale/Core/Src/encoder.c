/* Encoder incrementale del pendolo su TIM2 (vedi encoder.h). */
#include "encoder.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;

#define ENCODER_CPR  CFG_ENCODER_CPR
#define HALF_CPR     (CFG_ENCODER_CPR / 2)

#define THETA_STABILE_MS    1000U
#define THETA_TIMEOUT_MS   30000U

/* Verso dell'encoder. */
#define ENCODER_SIGN  -1

static uint32_t conteggio_precedente = 0;
static int32_t posizione = HALF_CPR;

volatile float theta_deg = 180.0f;
volatile float theta_rad = PI_F;

volatile uint8_t theta_inizializzato = 0;
/* Encoder: variazione dell'ultima lettura e tempo di quiete (una lettura per periodo). */
volatile int32_t encoder_ultimo_delta = 0;
volatile uint32_t encoder_quiete_ms = 0;
volatile uint32_t encoder_salti = 0;          // salti scartati, per debug
volatile uint32_t theta_ricalibrazioni = 0;

/* Inizializzazione dell'encoder. */
HAL_StatusTypeDef Encoder_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    conteggio_precedente = 0U;
    return HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void Leggi_theta(void){
	uint32_t conteggio = __HAL_TIM_GET_COUNTER(&htim2);

    int32_t delta = (int32_t)(conteggio - conteggio_precedente);
    conteggio_precedente = conteggio;
    encoder_ultimo_delta = delta;

	// Aggiorna la posizione e la riporta entro un giro.
	int32_t nuova_posizione = (posizione + ENCODER_SIGN * delta) % ENCODER_CPR;

	// Intervallo conteggi da -1199 a +1200.
	if (nuova_posizione > HALF_CPR)
	  nuova_posizione -= ENCODER_CPR;
	else if (nuova_posizione <= -HALF_CPR)
	  nuova_posizione += ENCODER_CPR;

	posizione = nuova_posizione;

	// Le due uscite richieste.
	theta_deg = (float)posizione * (360.0f / ENCODER_CPR);
	theta_rad = (float)posizione * (2.0f * PI_F / ENCODER_CPR);
}

/* Chiamata una volta per ciclo di controllo.
 * Aggiorna theta, conta da quanto il pendolo e' fermo 
 * e controlla che il movimento sia fisicamente possibile.
 * Ritorna 1 se il salto e' troppo grande. */

uint8_t Encoder_Campiona(uint32_t elapsed_ms, uint32_t periodo_ms)
{
    Leggi_theta();
    int32_t delta_encoder = encoder_ultimo_delta;
    uint32_t salto_max = (CFG_SALTO_ENCODER_MAX * elapsed_ms + periodo_ms - 1U) / periodo_ms;
    /* Quiete: al piu' un conteggio per lettura. */
    if (delta_encoder >= -1 && delta_encoder <= 1)
    {
        if (encoder_quiete_ms < 0x7FFFFFFFU)
            encoder_quiete_ms += elapsed_ms;
    }
    else
        encoder_quiete_ms = 0;
    /* Controllo dei salti */
    if ((uint32_t)(delta_encoder < 0 ? -delta_encoder : delta_encoder) > salto_max)
    {
        encoder_salti++;
        return 1;
    }
    return 0;
}

/* Prima dell'homing, a motore fermo: lasciare il pendolo libero verso il basso. */
HAL_StatusTypeDef Inizializza_Theta(void)
{
    theta_inizializzato = 0;
    uint32_t inizio = HAL_GetTick();
    uint32_t stabile_da = inizio;
    uint32_t riferimento = __HAL_TIM_GET_COUNTER(&htim2);

    while ((uint32_t)(HAL_GetTick() - inizio) < THETA_TIMEOUT_MS)
    {
        uint32_t conteggio = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t now = HAL_GetTick();
        // Ogni variazione del contatore fa ripartire l'attesa di un secondo.
        if (conteggio != riferimento)
        {
            riferimento = conteggio;
            stabile_da = now;
        }
        else if ((uint32_t)(now - stabile_da) >= THETA_STABILE_MS)
        {
            /* Usa il conteggio acquisito come origine senza resettare TIM2:
             * gli impulsi successivi, anche durante l'homing, restano validi. */
            conteggio_precedente = riferimento;
            posizione = HALF_CPR;
            theta_deg = 180.0f;
            theta_rad = PI_F;
            theta_inizializzato = 1;
            return HAL_OK;
        }
        HAL_Delay(1);
    }
    return HAL_TIMEOUT;
}

/* Imposta l'angolo corrente al valore iniziale dell'addestramento. */
void Theta_Ricalibra(void)
{
    int32_t conteggi = (int32_t)lroundf(CFG_ANGOLO_INIZIALE_RAD * (float)ENCODER_CPR / (2.0f * PI_F));
    conteggi %= ENCODER_CPR;
    if (conteggi > HALF_CPR)
        conteggi -= ENCODER_CPR;
    else if (conteggi <= -HALF_CPR)
        conteggi += ENCODER_CPR;
    conteggio_precedente = __HAL_TIM_GET_COUNTER(&htim2);
    posizione = conteggi;
    theta_deg = (float)posizione * (360.0f / ENCODER_CPR);
    theta_rad = (float)posizione * (2.0f * PI_F / ENCODER_CPR);
    theta_ricalibrazioni++;
}
