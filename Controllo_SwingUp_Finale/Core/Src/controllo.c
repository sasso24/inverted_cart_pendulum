/* Controllo: pulsante B1, avvio e fault, rampa del motore e predizione
 * dello stimatore in SysTick, campionamento e rete nel ciclo principale
 * (vedi controllo.h). */
#include "controllo.h"
#include <math.h>
#include "network.h"
#include "stima.h"
#include "config_controllo.h"  /* generato da esporta_firmware.py */
#include "stepper.h"
#include "encoder.h"
#include "rete.h"

/* Swing-up con storia degli ingressi (Controllo_SwingUp_Finale, asta 0,78 / 0,20 m). Tutti i numeri legati all'addestramento
 * (motore, rampa, scale degli ingressi, limite di corsa, stimatore, controlli
 * dell'encoder) sono in Core/Inc/config_controllo.h, scritto da
 * cart_pendolo_velocita_rl/esporta_firmware.py dalla configurazione salvata
 * con la rete. Non correggerli qui: cambiare config.toml e riesportare.
 * Frequenze, versi e homing del motore sono in stepper.h / stepper.c,
 * risoluzione e verso dell'encoder in encoder.c.
 */
#define BUTTON_RELEASE_MS  50U

/* Periodo della rete da config_controllo.h ([episodio].periodo_controllo_s):
 * 20 ms per le reti fino al 14/10, 10 ms per quelle addestrate a 100 Hz. */
#ifndef CFG_PERIODO_CONTROLLO_MS
#define CFG_PERIODO_CONTROLLO_MS 20U
#endif
#ifndef CFG_TICK_FERMO_INVERSIONE
#define CFG_TICK_FERMO_INVERSIONE 0U
#endif
/* Attesa attiva dopo un cambio di DIR a motore fermo: ~300 iterazioni
 * volatili >= 6 us a 180 MHz (TB6600 e simili chiedono 5 us di setup). */
#define DIR_SETUP_CICLI          300U
#define CONTROL_PERIOD_MS        CFG_PERIODO_CONTROLLO_MS
#define CONTROL_TIMEOUT_MS       (2U * CONTROL_PERIOD_MS)
#if CONTROL_PERIOD_MS < 4U || (CONTROL_PERIOD_MS % MOTOR_PERIOD_MS) != 0U
#error "Il periodo della rete deve essere un multiplo di 2 ms (almeno 4 ms)"
#endif
#define METRI_PER_STEP           CFG_METRI_PER_STEP

/* Codici di controllo_errore aggiunti in questo progetto. */
#define ERRORE_SALTO_ENCODER     9U   /* lettura encoder fisicamente impossibile */
#define ERRORE_ANGOLO           10U   /* solo equilibrio: pendolo oltre il limite */
#define ERRORE_AVVIO_ANGOLO     11U   /* avvio rifiutato: pendolo non fermo o non in posizione */

/* Il main e' l'unico contesto che puo' abilitare il controllo.
 * L'ISR puo' solo richiedere un avvio oppure fermare immediatamente.
 */
volatile uint8_t controllo_abilitato = 0;
static volatile uint8_t richiesta_avvio = 0;
static volatile uint8_t pulsante_pronto = 0;
volatile uint8_t pulsante_inizializzato = 0;
static volatile uint32_t pulsante_rilascio_ms = 0;

/* Diagnostica osservabile dal debugger: 0=OK, 1=init rete, 2=inferenza,
 * 3=NaN/Inf, 4=finecorsa/limite, 5=deadline, 6=avvio PWM, 7=calibrazione theta. */
volatile uint32_t controllo_errore = 0;
volatile float frequenza_richiesta_hz = 0.0f;
volatile float velocita_carrello_m_s = 0.0f;
volatile float velocita_angolare_rad_s = 0.0f;
static volatile float frequenza_rampa_hz = 0.0f;
/* Periodi di rampa rimasti con frequenza ferma a zero dopo un'inversione. */
static volatile uint32_t tick_fermo_inversione = 0;
static volatile uint32_t ultimo_comando_ms = 0;
static uint32_t ultimo_motore_ms = 0;
static uint32_t ultimo_campione_ms = 0;
static int32_t ultimo_carrello_steps = 0;
static float ultimo_theta_rad = PI_F;

/* Stimatore dello stato (stima.c), lo stesso usato in addestramento.
 * Predizione in SysTick ogni 2 ms, correzione in Controllo_Campiona ogni CONTROL_PERIOD_MS.
 * La rete riceve x, theta, v, omega STIMATI, non derivati dalle letture.
 */
Stima stima;                         /* osservabile dal debugger */
static volatile uint8_t stima_valida = 0;
static uint32_t ultimo_stima_ms = 0;

/* Limiti di corsa del controllo: centro +/- CFG_LIMITE_CORSA_M, in passi,
 * gli stessi dell'addestramento (prima coincidevano con i finecorsa). */
volatile int32_t limite_controllo_min_steps = 0;
volatile int32_t limite_controllo_max_steps = 0;

/* Chiamata da SysTick durante i movimenti bloccanti nel main.
 * Nessun ritardo sul primo fronte di pressione; un'altra pressione viene
 * accettata solo dopo almeno 50 ms di rilascio stabile (PC13 alto).
 */
void Controllo_ButtonTick(void)
{
    if (!pulsante_inizializzato)
        return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
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
    storia_da_azzerare = 1;  /* al prossimo avvio la storia riparte dai primi ingressi */
#endif
    Stepper_Stop();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

/* Limiti del controllo: centro della corsa misurata +/- CFG_LIMITE_CORSA_M,
 * mai oltre i finecorsa. Python: x = (passi - corsa/2)*m_passo, stop a |x| >= limite. */
static void Calcola_LimitiControllo(void)
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

/* Tipo di partenza accettato da B1 (osservabile dal debugger):
 * 0 = nessuna, 1 = swing-up da appeso, 2 = equilibrio dall'alto. */
volatile uint8_t avvio_modo = 0;

/* Condizione sull'angolo per accettare B1, in base all'angolo LETTO.
 * Swing-up:
 *  - angolo entro CFG_ANGOLO_RICALIBRAZIONE_RAD da quello iniziale (-180 gradi):
 *    pendolo appeso. Con CFG_RICALIBRA_THETA e encoder fermo da
 *    CFG_TEMPO_QUIETE_MS l'angolo torna esattamente a -180 gradi (si cancellano
 *    gli errori accumulati dall'encoder), poi swing-up;
 *  - angolo entro CFG_ANGOLO_MAX_AVVIO_RAD da 0: pendolo portato su a mano,
 *    nessuna ricalibrazione, la rete parte direttamente con l'equilibrio;
 *  - altrimenti avvio rifiutato.
 * Solo equilibrio: pendolo vicino alla verticale alta.
 */
static uint8_t Theta_ProntoPerAvvio(void)
{
    avvio_modo = 0;
#if CFG_SWING_UP
    if (fabsf(Stima_Wrap(theta_rad - CFG_ANGOLO_INIZIALE_RAD)) <= CFG_ANGOLO_RICALIBRAZIONE_RAD)
    {
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
    if (fabsf(Stima_Wrap(theta_rad)) <= CFG_ANGOLO_MAX_AVVIO_RAD)
    {
        avvio_modo = 2;
        return 1;
    }
    return 0;
}

/* Chiamare solo dal main, dopo l'uscita da ogni sequenza precedente. */
static void Controllo_AccettaAvvio(void)
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
                /* Stimatore ripartito dalla lettura attuale: pendolo e carrello fermi,
                 * come al reset dell'episodio in addestramento. */
                float x = ((float)posizione_carrello_steps - 0.5f * (float)corsa_totale_steps)
                          * METRI_PER_STEP;
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

/* Eseguita da SysTick: rampa indipendente dal tempo di inferenza.
 * Non chiama HAL_Delay e non esegue la rete in interrupt.
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
    /* Ricontrollo dopo la mascheratura: B1 puo' aver interrotto SysTick. */
    if (!controllo_abilitato)
        goto done;
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

    /* Un solo passo da 2 ms; nessun salto di accelerazione dopo un ritardo.
     * Stessa logica di ControlloStepper.avanza (ambiente.py). */
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

    /* TIM3 a 16 bit: sotto la frequenza rappresentabile non emettere STEP.
     * Fermata o cambio di verso: arresto al prossimo fronte di discesa, l'ISR
     * imposta subito il nuovo DIR. Intanto la rampa resta a zero: alla
     * ripartenza nessun salto di frequenza. */
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
        /* Motore fermo nel verso opposto (partenza, o richiesta cambiata durante
         * l'arresto): nuovo DIR e attesa del tempo di setup del driver (>= 5 us)
         * prima del primo impulso, nello stesso periodo come in ambiente.py. */
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
    /* Frequenza quantizzata realmente programmata, non l'uscita della rete. */
    frequenza_applicata_hz = (dir == DIR_BACKWARD ? 1.0f : -1.0f)
        * Stepper_FrequenzaProgrammataHz();
done:
    __set_PRIMASK(primask);
}

/* Eseguita da SysTick dopo Controllo_MotorTick: predizione dello stimatore
 * ogni 2 ms con la velocita' STEP effettivamente programmata (zero se il
 * timer e' fermo, anche durante la pausa per il cambio di DIR).
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
    float v = (controllo_abilitato && stepper_running)
        ? frequenza_applicata_hz * METRI_PER_STEP : 0.0f;
    Stima_Predici(&stima, v, MOTOR_PERIOD_MS * 0.001f);
}

/* Campionamento anche a controllo fermo: lo stimatore resta aggiornato.
 * La posizione del carrello viene dagli STEP emessi (nessun encoder lineare);
 * le velocita' vengono dallo stimatore, non da differenze fra letture.
 */
static void Controllo_Campiona(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = (uint32_t)(now - ultimo_campione_ms);
    if (elapsed < CONTROL_PERIOD_MS)
        return;
    /* Periodo fisso senza deriva; se si e' in ritardo di un periodo intero, riallinea. */
    ultimo_campione_ms += CONTROL_PERIOD_MS;
    if ((uint32_t)(now - ultimo_campione_ms) >= CONTROL_PERIOD_MS)
        ultimo_campione_ms = now;
    /* Salto dell'encoder: a controllo fermo viene solo contato. */
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

    /* Correzione dello stimatore; SysTick non deve predire a meta' aggiornamento. */
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
    /* A controllo fermo il conteggio STEP e' esatto: niente deriva della stima. */
    if (!controllo_abilitato)
        stima.x = x_misurata;
    st = stima;
    __set_PRIMASK(primask);
    velocita_carrello_m_s = st.velocita;
    velocita_angolare_rad_s = st.omega;

    Rete_Osservazione(&st, applied_hz, rete_osservazione);
    if (!controllo_abilitato || !rete_pronta)
        return;
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
    ai_i32 batches = ai_network_run(rete, rete_input, rete_output);
    rete_durata_ms = (uint32_t)(HAL_GetTick() - start);
    if (batches != 1)
    {
        rete_ultimo_errore = ai_network_get_error(rete);
        rete_pronta = 0;
        Controllo_Fault(2);
        return;
    }
    float action = ((ai_float *)rete_output[0].data)[0];
    if (!isfinite(action))
    {
        Controllo_Fault(3);
        return;
    }
    action = fmaxf(-1.0f, fminf(action, 1.0f));
#if CFG_STORIA_N > 0
    Storia_Aggiungi(rete_osservazione, action);
#endif
    float requested = Rete_AzioneInHz(action);
    primask = __get_PRIMASK();
    __disable_irq();
    /* Uno stop durante ai_network_run non puo' essere annullato dal risultato. */
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

/* Dopo Encoder_Init() e Rete_Init(), solo dal main: calibrazione del pendolo,
 * homing del carrello, limiti del controllo, poi abilitazione di B1.
 * In caso di errore chiama Error_Handler() (non ritorna). */
void Controllo_Init(void)
{
    /* Acquisire il riferimento del pendolo PRIMA di muovere il carrello. */
    Stepper_Stop();
    if (Inizializza_Theta() != HAL_OK)
    {
        controllo_errore = 7;
        Error_Handler();
    }
    ultimo_theta_rad = theta_rad;
    velocita_angolare_rad_s = 0.0f;

    /* Homing automatico come prima, indipendente dal pulsante. */
    Inizializza();
    if (inizializzazione_esito != HAL_OK)
    {
        Error_Handler();
    }
    Calcola_LimitiControllo();

    /* Da qui B1 abilita/disabilita solo il controllo nel ciclo principale.
     * Le pressioni durante l'homing non vengono accodate.
     */
    Stepper_Stop();
    __HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    /* Aggiornamento dal riferimento pre-homing, NON una nuova calibrazione. */
    Leggi_theta();
    ultimo_theta_rad = theta_rad;
    ultimo_carrello_steps = posizione_carrello_steps;
    ultimo_campione_ms = HAL_GetTick();
    pulsante_inizializzato = 1;
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* Una iterazione del ciclo principale: a controllo fermo cambio rete e
 * avvio richiesto da B1, poi campionamento, stima e rete. */
void Controllo_Ciclo(void)
{
    if (!controllo_abilitato)
    {
        Rete_CambiaSeRichiesto();
        Controllo_AccettaAvvio();
    }
    Controllo_Campiona();
}

/* Fronti di B1 (PC13): richiesta di avvio o arresto immediato. Unica
 * definizione nel progetto: sostituisce quella __weak della HAL.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != B1_Pin || !pulsante_inizializzato)
        return;
    /* Entrambi i fronti azzerano il tempo di rilascio: anche un breve
     * rimbalzo fra due SysTick non puo' riarmare il pulsante. */
    pulsante_rilascio_ms = 0;
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET
        || !pulsante_pronto)
        return;
    pulsante_pronto = 0;
    if (controllo_abilitato || richiesta_avvio)
    {
        richiesta_avvio = 0;
        Controllo_Disabilita(); /* Stop PWM qui, senza attendere il main. */
    }
    else
        richiesta_avvio = 1; /* Avvio del controllo nel ciclo principale. */
}
