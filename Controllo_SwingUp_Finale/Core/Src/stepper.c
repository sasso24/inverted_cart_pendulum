/* Motore stepper, finecorsa e homing del carrello. */
#include "stepper.h"
#include "controllo.h"
#include <math.h>

#define STEP_CHANNEL TIM_CHANNEL_1

/* Configurazione Homing. */
#define HOMING_VELOCITY_HZ  (6400.0f)
#define HOMING_TIMEOUT_MS  120000U

extern TIM_HandleTypeDef htim3;

/* Soglia di partenza in Hz per la rampa. */
float max_starting_frequency = MIN_FREQUENCY;
static float stepper_target_frequency = MAX_FREQUENCY;
static float stepper_counter_clock;
static float stepper_start_frequency;
static uint32_t stepper_ramp_start_ms;
static uint32_t stepper_ramp_last_ms;
static volatile uint8_t stepper_ramp_active = 0;
volatile uint32_t step_count = 0;
volatile uint32_t step_target = 0;
volatile uint8_t stepper_running = 0;
volatile GPIO_PinState stepper_direction = DIR_FORWARD;
volatile uint8_t arresto_finecorsa = 0;
volatile uint8_t stop_al_fronte_step = 0;
/* Verso da impostare nell'ISR appena il timer si ferma al fronte STEP. */
volatile GPIO_PinState direzione_dopo_arresto = DIR_FORWARD;
volatile float frequenza_applicata_hz = 0.0f;

/* Riferimento del carrello: zero al finecorsa sinistro, positivo a destra.
 * Stima in impulsi STEP comandati.
 */
volatile int32_t posizione_carrello_steps = 0;
volatile int32_t posizione_carrello_min_steps = 0;
volatile int32_t posizione_carrello_max_steps = 0;
volatile uint32_t corsa_totale_steps = 0;
volatile uint8_t riferimento_carrello_valido = 0;
volatile uint8_t carrello_inizializzato = 0;
volatile HAL_StatusTypeDef inizializzazione_esito = HAL_ERROR;

/* Prima dell'abilitazione di B1, l'homing iniziale e' indipendente
 * dal comando del controllo. Dopo, ogni avvio del motore richiede l'abilitazione.
 */
static uint8_t Stepper_MovimentoConsentito(void)
{
    return !pulsante_inizializzato || controllo_abilitato;
}

/* Attesa interrompibile: Se il movimento viene vietato (per esempio per una pressione di B1) 
 * esce subito senza aspettare il resto.
 */
static uint8_t Stepper_Attendi(uint32_t durata_ms)
{
    uint32_t inizio = HAL_GetTick();
    while (Stepper_MovimentoConsentito()
           && (uint32_t)(HAL_GetTick() - inizio) < durata_ms)
        HAL_Delay(1);
    return Stepper_MovimentoConsentito();
}

/* Pull-down interno */
uint8_t FinecorsaSinistro_Premuto(void)
{
    return HAL_GPIO_ReadPin(FC_sx_GPIO_Port, FC_sx_Pin) == GPIO_PIN_SET;
}

/* Come il sinistro: pull-down interno, contatto premuto a livello alto. */
uint8_t FinecorsaDestro_Premuto(void)
{
    return HAL_GPIO_ReadPin(Fc_dx_GPIO_Port, Fc_dx_Pin) == GPIO_PIN_SET;
}

static uint8_t FinecorsaDirezione_Premuto(GPIO_PinState direction)
{
    return (direction == DIR_FORWARD) ? FinecorsaSinistro_Premuto()
                                      : FinecorsaDestro_Premuto();
}

void Stepper_SetDirection(GPIO_PinState dir)
{
    stepper_direction = dir;
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dir);
}

void Stepper_Stop(void)
{
    HAL_TIM_PWM_Stop_IT(&htim3, STEP_CHANNEL);
    stepper_running = 0;
    stepper_ramp_active = 0;
    frequenza_applicata_hz = 0.0f;
    stop_al_fronte_step = 0;
}

/* Check del movimento e avvio del PWM devono essere indivisibili rispetto all'ISR di stop.
 */
void Stepper_StartIfEnabled(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Stepper_MovimentoConsentito())
    {
        stepper_running = 1;
        if (HAL_TIM_PWM_Start_IT(&htim3, STEP_CHANNEL) != HAL_OK)
            Stepper_Stop();
    }
    else
        Stepper_Stop();
    __set_PRIMASK(primask);
}

static float Stepper_ClampFrequency(float frequency)
{
    if (!isfinite(frequency) || frequency < MIN_FREQUENCY)
        return MIN_FREQUENCY;
    if (frequency > MAX_FREQUENCY)
        return MAX_FREQUENCY;
    return frequency;
}

/* Applicazione di ARR e CCR al prossimo periodo PWM.
 */
void Stepper_ApplyFrequency(float frequency)
{
    float period = ceilf(stepper_counter_clock / frequency);
    if (period > 65536.0f)
        period = 65536.0f;
    else if (period < 2.0f)
        period = 2.0f;
    uint32_t ticks = (uint32_t)period;

    /* Protezione read/write di CR1 da uno stop concorrente di un interrupt. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    SET_BIT(htim3.Instance->CR1, TIM_CR1_UDIS);   // Disabilita l'aggiornamento automatico di ARR e CCR
    __HAL_TIM_SET_AUTORELOAD(&htim3, ticks - 1U);
    __HAL_TIM_SET_COMPARE(&htim3, STEP_CHANNEL, ticks / 2U);
    CLEAR_BIT(htim3.Instance->CR1, TIM_CR1_UDIS);
    __set_PRIMASK(primask);
}

/* Chiamata a timer fermo per ogni nuovo movimento.
 */
static void Stepper_PrepareRamp(void)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
        timer_clock *= 2U;
    stepper_counter_clock = (float)timer_clock / (htim3.Instance->PSC + 1U);

    stepper_target_frequency = Stepper_ClampFrequency(stepper_target_frequency);
    float starting_limit = Stepper_ClampFrequency(max_starting_frequency);
    stepper_start_frequency = fminf(stepper_target_frequency, starting_limit);
    stepper_ramp_active = stepper_target_frequency > stepper_start_frequency;
    Stepper_ApplyFrequency(stepper_start_frequency);
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
    stepper_ramp_start_ms = HAL_GetTick();
    stepper_ramp_last_ms = stepper_ramp_start_ms;
}

/* Aggiornamento chiamato nella callback degli impulsi STEP.
 */
static void Stepper_UpdateRamp(void)
{
    if (!stepper_ramp_active)
        return;
    uint32_t now = HAL_GetTick();
    if (now == stepper_ramp_last_ms)
        return;
    stepper_ramp_last_ms = now;
    float frequency = stepper_start_frequency
        + STEPPER_ACCELERATION_HZ_PER_S * 0.001f
        * (uint32_t)(now - stepper_ramp_start_ms);
    if (frequency >= stepper_target_frequency)
    {
        frequency = stepper_target_frequency;
        stepper_ramp_active = 0;
    }
    Stepper_ApplyFrequency(frequency);
}

/* Reset del timer prima di far ripartire il motore da fermo
 */
void Stepper_PreparaAvvioContinuo(void)
{
    step_count = 0;
    step_target = 0;
    stepper_ramp_active = 0; /* La rampa RL sostituisce quella di homing. */
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
}

/* Restituisce la frequenza minima che il motore puo' generare. */
float Stepper_FrequenzaMinimaHz(void)
{
    return stepper_counter_clock / 65536.0f;
}

/* Frequenza programmata in ARR, senza segno. */
float Stepper_FrequenzaProgrammataHz(void)
{
    return stepper_counter_clock / (float)(htim3.Instance->ARR + 1U);
}

/* Muove il motore di un certo numero di passi. */
void Stepper_MoveSteps(uint32_t steps)
{
    Stepper_Stop();
    step_count = 0;
    step_target = steps;
    arresto_finecorsa = 0;

    if (!Stepper_MovimentoConsentito())
        return;
    if (FinecorsaDirezione_Premuto(stepper_direction))
    {
        arresto_finecorsa = 1;
        return;
    }
    if (steps == 0U)
        return;

    Stepper_PrepareRamp();
    Stepper_StartIfEnabled();
}

/* Muove il motore in una direzione specifica per un certo numero di passi. */
void Stepper_Move(GPIO_PinState direction, uint32_t steps)
{
    Stepper_Stop();
    Stepper_SetDirection(direction);
    if (!Stepper_Attendi(1))
        return;
    Stepper_MoveSteps(steps);

    while (stepper_running)
    {
        if (FinecorsaDirezione_Premuto(direction))
        {
            Stepper_Stop();
            arresto_finecorsa = 1;
        }
    }
}

/* Imposta la velocità del motore in termini di impulsi STEP al secondo, limitati a MIN/MAX_FREQUENCY.
 */
void Stepper_set_velocity(GPIO_PinState direction, float velocity)
{
    Stepper_Stop();
    step_count = 0;
    step_target = 0; /* Zero identifica il movimento continuo. */
    arresto_finecorsa = 0;

    if (!Stepper_MovimentoConsentito() || !isfinite(velocity) || velocity <= 0.0f)
        return;

    stepper_target_frequency = Stepper_ClampFrequency(velocity);
    Stepper_SetDirection(direction);
    if (!Stepper_Attendi(1))
        return;
    if (FinecorsaDirezione_Premuto(direction))
    {
        arresto_finecorsa = 1;
        return;
    }

    Stepper_PrepareRamp();
    Stepper_StartIfEnabled();
}

/* Da qualsiasi posizione cerca lentamente il finecorsa destro e azzera
 * il conteggio, poi misura la corsa fino a sinistra e torna al centro.
 */
void Inizializza(void)
{
    Stepper_Stop();
    carrello_inizializzato = 0;
    riferimento_carrello_valido = 0;
    inizializzazione_esito = HAL_BUSY;
    arresto_finecorsa = 0;
    corsa_totale_steps = 0;
    posizione_carrello_min_steps = 0;
    posizione_carrello_max_steps = 0;
    posizione_carrello_steps = 0;


    Stepper_set_velocity(DIR_BACKWARD, HOMING_VELOCITY_HZ);
    uint32_t inizio = HAL_GetTick();
    while (stepper_running)
    {
        if ((uint32_t)(HAL_GetTick() - inizio) >= HOMING_TIMEOUT_MS)
        {
            Stepper_Stop();
            inizializzazione_esito = HAL_TIMEOUT;
            return;
        }
        HAL_Delay(1);
    }

    Stepper_Stop();
    HAL_Delay(20);
    if (!arresto_finecorsa || !FinecorsaDestro_Premuto()
        || FinecorsaSinistro_Premuto())
    {
        inizializzazione_esito = HAL_ERROR;
        return;
    }

    // Zero impostato al finecorsa destro.
    step_count = 0;
    posizione_carrello_steps = 0;
    Stepper_set_velocity(DIR_FORWARD, HOMING_VELOCITY_HZ);
    inizio = HAL_GetTick();
    while (stepper_running)
    {
        if ((uint32_t)(HAL_GetTick() - inizio) >= HOMING_TIMEOUT_MS)
        {
            Stepper_Stop();
            inizializzazione_esito = HAL_TIMEOUT;
            return;
        }
        HAL_Delay(1);
    }

    Stepper_Stop();
    HAL_Delay(20); 
    if (!arresto_finecorsa || !FinecorsaSinistro_Premuto()
        || step_count < 2U || step_count > INT32_MAX)
    {
        inizializzazione_esito = HAL_ERROR;
        return;
    }

    /* La misura viene salvata prima che il prossimo movimento azzeri step_count. */
    corsa_totale_steps = step_count;
    posizione_carrello_min_steps = 0;
    posizione_carrello_max_steps = (int32_t)corsa_totale_steps;
    posizione_carrello_steps = posizione_carrello_min_steps;
    riferimento_carrello_valido = 1;

    /* Con corsa dispari, il centro e' arrotondato verso sinistra. */
    uint32_t passi_centro = corsa_totale_steps / 2U;
    Stepper_SetDirection(DIR_BACKWARD);
    HAL_Delay(1);
    stepper_target_frequency = MAX_FREQUENCY;
    Stepper_MoveSteps(passi_centro);
    inizio = HAL_GetTick();
    while (stepper_running)
    {
        if ((uint32_t)(HAL_GetTick() - inizio) >= HOMING_TIMEOUT_MS)
        {
            Stepper_Stop();
            riferimento_carrello_valido = 0;
            inizializzazione_esito = HAL_TIMEOUT;
            return;
        }
        HAL_Delay(1);
    }

    if (arresto_finecorsa || step_count != passi_centro)
    {
        riferimento_carrello_valido = 0;
        inizializzazione_esito = HAL_ERROR;
        return;
    }
    carrello_inizializzato = 1;
    inizializzazione_esito = HAL_OK;
}

/* Alla fine di ogni impulso STEP effettua: conteggio della posizione, controllo dei finecorsa,
 * limiti del controllo, arresto al fronte e rampa di homing. 
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1
        || !stepper_running || !Stepper_MovimentoConsentito())
        return;

    step_count++;
    if (riferimento_carrello_valido)
        posizione_carrello_steps += (stepper_direction == DIR_FORWARD) ? -1 : 1;

    if (FinecorsaDirezione_Premuto(stepper_direction))
    {
        Stepper_Stop();
        arresto_finecorsa = 1;
        if (pulsante_inizializzato)
            Controllo_Fault(4);
        return;
    }

    if (pulsante_inizializzato
        && (posizione_carrello_steps <= limite_controllo_min_steps
            || posizione_carrello_steps >= limite_controllo_max_steps))
    {
        Controllo_Fault(4);
        return;
    }
    if (stop_al_fronte_step)
    {
        GPIO_PinState dir = direzione_dopo_arresto;
        Stepper_Stop();
        /* DIR cambia a fine impulso: almeno un periodo di rampa prima del successivo. */
        Stepper_SetDirection(dir);
        return;
    }

    /* Il movimento continuo non ha un numero di passi limite. */
    if (step_target != 0U && step_count >= step_target)
    {
        Stepper_Stop();
        return;
    }
    Stepper_UpdateRamp();
}
