/* Controllo del pendolo: pulsante B1, avvio e fault, rampa del
 * motore e predizione dello stimatore, campionamento e rete.
 * (vedi controllo.h). */
#include "controllo.h"
#include <math.h>
#include "network.h"
#include "stima.h"
#include "config_controllo.h"  /* generato da esporta_firmware.py */
#include "stepper.h"
#include "encoder.h"
#include "rete.h"

/* Swing-up con storia degli ingressi (Controllo_SwingUp_Finale). Tutti i numeri legati all'addestramento
 * (motore, rampa, scale degli ingressi, limite di corsa, stimatore, controlli
 * dell'encoder) sono in config_controllo.h, scritto da
 * esporta_firmware.py.
 * Frequenze, versi e homing del motore sono in stepper.h / stepper.c,
 * risoluzione e verso dell'encoder in encoder.c.
 */
#define BUTTON_RELEASE_MS  50U

/* Periodo della rete da config_controllo.h */
#ifndef CFG_PERIODO_CONTROLLO_MS
#define CFG_PERIODO_CONTROLLO_MS 20U
#endif
#ifndef CFG_TICK_FERMO_INVERSIONE
#define CFG_TICK_FERMO_INVERSIONE 0U
#endif
/* Attesa attiva dopo un cambio di DIR a motore fermo: circa 30 cicli for
 * (30*4 cicli di clock a 180 MHz = 0.67 us) per garantire setup (driver setup 0.65 us) */
#define DIR_SETUP_CICLI          30U
#define CONTROL_PERIOD_MS        CFG_PERIODO_CONTROLLO_MS
#define CONTROL_TIMEOUT_MS       (2U * CONTROL_PERIOD_MS)
#if CONTROL_PERIOD_MS < 4U || (CONTROL_PERIOD_MS % MOTOR_PERIOD_MS) != 0U
#error "Il periodo della rete deve essere un multiplo di 2 ms (almeno 4 ms)"
#endif
#define METRI_PER_STEP           CFG_METRI_PER_STEP

/* Codici aggiuntividi controllo_errore */
#define ERRORE_SALTO_ENCODER     9U   /* lettura encoder fisicamente impossibile */
#define ERRORE_ANGOLO           10U   /* pendolo oltre il limite (swing-up disabilitato) */
#define ERRORE_AVVIO_ANGOLO     11U   /* pendolo non fermo o non in posizione */


volatile uint8_t controllo_abilitato = 0;
static volatile uint8_t richiesta_avvio = 0;
static volatile uint8_t pulsante_pronto = 0;
volatile uint8_t pulsante_inizializzato = 0;
static volatile uint32_t pulsante_rilascio_ms = 0;

/* Errori: 0=OK, 1=init rete, 2=inferenza,
 * 3=NaN/Inf, 4=finecorsa/limite, 5=deadline, 6=avvio PWM, 7=calibrazione theta. */
volatile uint32_t controllo_errore = 0;
volatile float frequenza_richiesta_hz = 0.0f;
volatile float velocita_carrello_m_s = 0.0f;
volatile float velocita_angolare_rad_s = 0.0f;
static volatile float frequenza_rampa_hz = 0.0f;

static volatile uint32_t tick_fermo_inversione = 0;
static volatile uint32_t ultimo_comando_ms = 0;
static uint32_t ultimo_motore_ms = 0;
uint32_t ultimo_campione_ms = 0;
int32_t ultimo_carrello_steps = 0;
float ultimo_theta_rad = PI_F;

/* Stimatore dello stato (stima.c).
 * Predizione in SysTick ogni 2 ms, correzione in Controllo_Esegui ogni CONTROL_PERIOD_MS.
 * La rete riceve x, theta, v e omega stimati, non derivati dalle letture.
 */
Stima stima;                         
static volatile uint8_t stima_valida = 0;
static uint32_t ultimo_stima_ms = 0;

/* Limiti di corsa del controllo */
volatile int32_t limite_controllo_min_steps = 0;
volatile int32_t limite_controllo_max_steps = 0;

/* Tipo di partenza accettato da B1 (per debug):
 * 0 = nessuna, 1 = swing-up da appeso, 2 = equilibrio dall'alto. */
volatile uint8_t avvio_modo = 0;

/* Chiamata da SysTick durante i movimenti bloccanti nel main.
 * Controllo antirimbalzo del pulsante B1.
 */
void Controllo_ButtonTick(void)
{
    if (!pulsante_inizializzato)
        return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    // Rileva il pulsante premuto (PC13 basso)
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
        pulsante_rilascio_ms = 0;
    else if (pulsante_rilascio_ms < BUTTON_RELEASE_MS)
        ++pulsante_rilascio_ms;
    else
        pulsante_pronto = 1;
    __set_PRIMASK(primask);
}

void Controllo_Disabilita(void)
{
    controllo_abilitato = 0;
    frequenza_richiesta_hz = 0.0f;
    frequenza_rampa_hz = 0.0f;
    rete_azione = 0.0f;
#if CFG_STORIA_N > 0
    storia_da_azzerare = 1;  
#endif
    Stepper_Stop();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}


void Calcola_LimitiControllo(void)
{
    float centro = 0.5f * (float)corsa_totale_steps;
    float mezzo = CFG_LIMITE_CORSA_M / METRI_PER_STEP;
    int32_t lo = (int32_t)ceilf(centro - mezzo);
    int32_t hi = (int32_t)floorf(centro + mezzo);
    if (lo < posizione_carrello_min_steps)
        lo = posizione_carrello_min_steps;
    if (hi > posizione_carrello_max_steps)
        hi = posizione_carrello_max_steps;
    limite_controllo_min_steps = lo;
    limite_controllo_max_steps = hi;
}

/* Condizione sull'angolo per accettare B1, in base all'angolo misurato.
 */
static uint8_t Theta_ProntoPerAvvio(void)
{
    avvio_modo = 0;
// Swing-up attivo.
#if CFG_SWING_UP
    // Angolo entro CFG_ANGOLO_RICALIBRAZIONE_RAD da quello iniziale (pendolo appeso)
    if (fabsf(Stima_Wrap(theta_rad - CFG_ANGOLO_INIZIALE_RAD)) <= CFG_ANGOLO_RICALIBRAZIONE_RAD)   
    {
// Ricalibrazione dell'encoder se il pendolo e' fermo e appeso
#if CFG_RICALIBRA_THETA
        if (encoder_quiete_ms < CFG_TEMPO_QUIETE_MS)
            return 0;
        Theta_Ricalibra();
        ultimo_theta_rad = theta_rad;
#endif
        avvio_modo = 1;
        return 1;
    }
#endif
// Solo equilibrio: pendolo vicino alla verticale alta.
    if (fabsf(Stima_Wrap(theta_rad)) <= CFG_ANGOLO_MAX_AVVIO_RAD)
    {
        avvio_modo = 2;
        return 1;
    }
    return 0;
}

/* Chiamata solo dal main */
void Controllo_AccettaAvvio(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (richiesta_avvio)
    {
        richiesta_avvio = 0;
        if (rete_pronta && theta_inizializzato
            && carrello_inizializzato && riferimento_carrello_valido
            && !FinecorsaSinistro_Premuto() && !FinecorsaDestro_Premuto()
            && posizione_carrello_steps > limite_controllo_min_steps
            && posizione_carrello_steps < limite_controllo_max_steps)
        {
            if (!Theta_ProntoPerAvvio())
            {
                controllo_errore = ERRORE_AVVIO_ANGOLO;
            }
            else
            {
                controllo_errore = 0;
                arresto_finecorsa = 0;
                frequenza_richiesta_hz = 0.0f;
                frequenza_rampa_hz = 0.0f;
                tick_fermo_inversione = 0U;

                float x = ((float)posizione_carrello_steps - 0.5f * (float)corsa_totale_steps)* METRI_PER_STEP;
                Stima_Reset(&stima, x, theta_rad, 0.0f);
                stima_valida = 1;
                ultimo_comando_ms = HAL_GetTick();
                ultimo_motore_ms = ultimo_comando_ms;
                ultimo_stima_ms = ultimo_comando_ms;
                controllo_abilitato = 1;
                HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            }
        }
    }
    __set_PRIMASK(primask);
}

void Controllo_Fault(uint32_t errore)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    controllo_errore = errore;
    richiesta_avvio = 0;
    Controllo_Disabilita();
    __set_PRIMASK(primask);
}

/* Eseguita da SysTick
 * L'inversione attende il fronte di discesa STEP e lascia DIR stabile
 * almeno un intervallo di rampa prima di riavviare il timer.
 */
void Controllo_MotorTick(void)
{
    if (!pulsante_inizializzato || !controllo_abilitato)
        return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = HAL_GetTick();
    /* Ricontrollo dopo la mascheratura */
    if (!controllo_abilitato)
        goto done;          // rimanda all'etichetta done
    if (FinecorsaSinistro_Premuto() || FinecorsaDestro_Premuto()
        || posizione_carrello_steps <= limite_controllo_min_steps
        || posizione_carrello_steps >= limite_controllo_max_steps)
    {
        Controllo_Fault(4);
        goto done;
    }
    if ((uint32_t)(now - ultimo_comando_ms) >= CONTROL_TIMEOUT_MS)
    {
        Controllo_Fault(5);
        goto done;
    }
    if ((uint32_t)(now - ultimo_motore_ms) < MOTOR_PERIOD_MS)
        goto done;
    ultimo_motore_ms = now;

    /* Aggiornamento della frequenza della rampa. */
    float delta_max = STEPPER_ACCELERATION_HZ_PER_S * MOTOR_PERIOD_MS / 1000.0f;
    float delta = frequenza_richiesta_hz - frequenza_rampa_hz;
    float nuova = frequenza_rampa_hz + fmaxf(-delta_max, fminf(delta, delta_max));
#if CFG_TICK_FERMO_INVERSIONE > 0
    /* Inversione: la rampa si ferma a zero (arresto al fronte STEP e cambio DIR)
     * per CFG_TICK_FERMO_INVERSIONE periodi, poi riparte da zero nel nuovo verso. */
    if (nuova * frequenza_rampa_hz < 0.0f)
    {
        nuova = 0.0f;
        tick_fermo_inversione = CFG_TICK_FERMO_INVERSIONE - 1U;
    }
    else if (tick_fermo_inversione > 0U)
    {
        nuova = 0.0f;
        --tick_fermo_inversione;
    }
#endif
    float hz = fabsf(nuova);
    /* Verso del movimento: quello della rampa o, a rampa ferma, della richiesta. */
    float verso = (nuova != 0.0f) ? nuova : frequenza_richiesta_hz;
    GPIO_PinState dir = verso > 0.0f ? DIR_BACKWARD : DIR_FORWARD;
    uint8_t sotto_minimo = hz < Stepper_FrequenzaMinimaHz();

    /* Se il motore e' fermo e la rampa e' sotto il minimo, non fare nulla.
     * Se il motore e' in movimento ma la rampa scende sotto il minimo, arresta al fronte
     * e lascia DIR stabile per almeno un periodo di rampa prima di riavviare.
     * Se il motore e' in movimento ma la rampa cambia verso, arresta al fronte
     * e lascia DIR stabile per almeno un periodo di rampa prima di riavviare.
     */
    if (stepper_running && (sotto_minimo || dir != stepper_direction || stop_al_fronte_step))
    {
        direzione_dopo_arresto = dir;
        stop_al_fronte_step = 1;
        frequenza_rampa_hz = 0.0f;
        goto done;
    }
    frequenza_rampa_hz = nuova;
    if (sotto_minimo)
        goto done;
    if (!stepper_running && dir != stepper_direction)
    {
        /* Motore fermo nel verso opposto: nuovo DIR e attesa del tempo di setup del driver (>= 0.65 us)
         * prima del primo impulso. */
        Stepper_SetDirection(dir);
        for (volatile uint32_t k = 0; k < DIR_SETUP_CICLI; ++k)
        {
        }
    }
    Stepper_ApplyFrequency(hz);
    if (!stepper_running)
    {
        Stepper_PreparaAvvioContinuo();
        Stepper_StartIfEnabled();
        if (!stepper_running)
        {
            Controllo_Fault(6);
            goto done;
        }
    }
    /* Frequenza quantizzata. */
    frequenza_applicata_hz = (dir == DIR_BACKWARD ? 1.0f : -1.0f) * Stepper_FrequenzaProgrammataHz();

// riabilita le IRQ ed esce
done:
    __set_PRIMASK(primask);
}

/* Eseguita da SysTick dopo Controllo_MotorTick. Effettua la predizione dello stimatore
 * ogni 2 ms con la velocita' STEP effettivamente programmata.
 * Fuori dal controllo il carrello e' considerato fermo.
 */
void Controllo_StimaTick(void)
{
    if (!stima_valida)
        return;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - ultimo_stima_ms) < MOTOR_PERIOD_MS)
        return;
    ultimo_stima_ms = now;
    float v = (controllo_abilitato && stepper_running) ? frequenza_applicata_hz * METRI_PER_STEP : 0.0f;
    Stima_Predici(&stima, v, MOTOR_PERIOD_MS * 0.001f);
}

/* Campionamento anche a controllo fermo: lo stimatore resta aggiornato.
 * La posizione del carrello viene dagli STEP emessi,
 * le velocita' vengono dallo stimatore, non da differenze fra letture.
 */
void Controllo_Esegui(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = (uint32_t)(now - ultimo_campione_ms);
    if (elapsed < CONTROL_PERIOD_MS)
        return;
    /* Riallineamento del periodo in caso di deriva. */
    ultimo_campione_ms += CONTROL_PERIOD_MS;
    if ((uint32_t)(now - ultimo_campione_ms) >= CONTROL_PERIOD_MS)
        ultimo_campione_ms = now;
        
    if (Encoder_Campiona(elapsed, CONTROL_PERIOD_MS) && controllo_abilitato)
    {
        Controllo_Fault(ERRORE_SALTO_ENCODER);
        return;
    }
#if !CFG_SWING_UP
    if (controllo_abilitato && fabsf(theta_rad) > CFG_ANGOLO_MAX_CONTROLLO_RAD)
    {
        Controllo_Fault(ERRORE_ANGOLO);
        return;
    }
#endif
    int32_t cart_steps;
    float applied_hz;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cart_steps = posizione_carrello_steps;
    applied_hz = frequenza_applicata_hz;
    __set_PRIMASK(primask);
    float x_misurata = (cart_steps - 0.5f * (float)corsa_totale_steps) * METRI_PER_STEP;
    ultimo_theta_rad = theta_rad;
    ultimo_carrello_steps = cart_steps;

    /* Correzione dello stimatore. */
    Stima st;
    primask = __get_PRIMASK();
    __disable_irq();
    if (!stima_valida)
    {
        /* Prima lettura dopo l'homing: pendolo fermo, carrello fermo. */
        Stima_Reset(&stima, x_misurata, theta_rad, 0.0f);
        ultimo_stima_ms = HAL_GetTick();
        stima_valida = 1;
    }
    else
        Stima_Correggi(&stima, x_misurata, theta_rad);

    if (!controllo_abilitato)
        stima.x = x_misurata;
    st = stima;
    __set_PRIMASK(primask);
    velocita_carrello_m_s = st.velocita;
    velocita_angolare_rad_s = st.omega;

    Rete_Osservazione(&st, applied_hz, rete_osservazione);
    if (!controllo_abilitato || !rete_pronta)
        return;
    /* Definizione del puntatore agli ingressi della rete (ai_float alias di float) */
    ai_float *input = (ai_float *)rete_input[0].data; 
    for (uint32_t i = 0; i < 6U; ++i)
    {
        if (!isfinite(rete_osservazione[i]))
        {
            Controllo_Fault(3);
            return;
        }
        input[i] = rete_osservazione[i];
    }
#if CFG_STORIA_N > 0
    if (storia_da_azzerare)
    {
        Storia_Reset(rete_osservazione);
        storia_da_azzerare = 0;
    }
    Storia_Ingressi(input);
#endif
    uint32_t start = HAL_GetTick();
    /* Avvio della rete e restituzione codice di uscita (1 -> inferenza completata con successo
        altro -> errore) */
    ai_i32 batches = ai_network_run(rete, rete_input, rete_output);
    rete_durata_ms = (uint32_t)(HAL_GetTick() - start);
    if (batches != 1)
    {
        rete_ultimo_errore = ai_network_get_error(rete);
        rete_pronta = 0;
        Controllo_Fault(2);
        return;
    }
    /* Output della rete */
    float action = ((ai_float *)rete_output[0].data)[0];
    if (!isfinite(action))
    {
        Controllo_Fault(3);
        return;
    }
    action = fmaxf(-1.0f, fminf(action, 1.0f)); // Azione limitata a [-1;1]

#if CFG_STORIA_N > 0
    Storia_Aggiungi(rete_osservazione, action);
#endif

    /* Conversione azione in Hz */
    float requested = Rete_AzioneInHz(action);
    primask = __get_PRIMASK();
    __disable_irq();
    /* Aggiornamento dei valori della rete solo se il controllo è abilitato */
    if (controllo_abilitato)
    {
        if ((uint32_t)(HAL_GetTick() - now) >= CONTROL_PERIOD_MS)
            Controllo_Fault(5);
        else
        {
            rete_azione = action;
            frequenza_richiesta_hz = requested;
            ultimo_comando_ms = HAL_GetTick();
            ++rete_inferenze;
        }
    }
    __set_PRIMASK(primask);
}

/* Fronti di B1 (PC13): richiesta di avvio o arresto immediato.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != B1_Pin || !pulsante_inizializzato)
        return;
    /* Controllo antirimbalzo del pulsante. */
    pulsante_rilascio_ms = 0;
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET
        || !pulsante_pronto)
        return;
    pulsante_pronto = 0;
    if (controllo_abilitato || richiesta_avvio)
    {
        richiesta_avvio = 0;
        Controllo_Disabilita();
    }
    else
        richiesta_avvio = 1; /* Avvio del controllo nel ciclo principale. */
}
