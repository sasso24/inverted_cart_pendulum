/* Hardware-in-the-loop su USART2 (vedi hil.h). */
#include "hil.h"
#include "controllo.h"
#include <math.h>
#include <string.h>
#include "stima.h"
#include "rete.h"
#include "stepper.h"

#if HIL_MODE
/* ------------------------------------------------------------------------
 * Hardware-in-the-loop Protocollo su USART2, little-endian:
 *   PC -> scheda: A5 5A | HilRichiesta (58 byte) | checksum
 *   scheda -> PC: A5 5A | HilRisposta  (60 byte) | checksum
 * checksum = complemento a due della somma dei byte del corpo.
*/
#define HIL_SYNC0 0xA5U
#define HIL_SYNC1 0x5AU
#define HIL_MAX_PREDIZIONI 10U

extern UART_HandleTypeDef huart2;

typedef struct __attribute__((packed))
{
    uint8_t comando;
    uint8_t n;
    uint32_t seq;
    float v[HIL_MAX_PREDIZIONI];  /* m/s dopo la rampa, uno ogni 2 ms */
    float x;                      /* m, misura rispetto al centro */
    float theta;                 
    float applied_hz;             /* frequenza STEP applicata, DIR codificata nel segno */
} HilRichiesta;

typedef struct __attribute__((packed))
{
    uint32_t seq;
    uint32_t errore;              /* stessi codici di controllo_errore, 8 = richiesta non valida */
    float azione;
    float richiesta_hz;
    float osservazione[6];
    float stima_x, stima_theta, stima_omega;
    uint32_t rete_us;             /* durata di ai_network_run */
    uint32_t totale_us;           /* stimatore + rete */
} HilRisposta;

/* Controllo della dimensione dei frame. 
* Se non è uguale, restituisce in compilazione il messaggio tra parentesi */
_Static_assert(sizeof(HilRichiesta) == 58, "HilRichiesta");
_Static_assert(sizeof(HilRisposta) == 60, "HilRisposta");

static Stima hil_stima;
static HilRisposta hil_risposta;
volatile uint32_t hil_pacchetti = 0;
volatile uint32_t hil_scartati = 0;

static uint8_t Hil_Checksum(const uint8_t *dati, uint32_t n)
{
    uint8_t somma = 0;
    for (uint32_t i = 0; i < n; ++i)
        somma = (uint8_t)(somma + dati[i]);
    return (uint8_t)(0U - somma);
}

static uint8_t Hil_Ricevi(HilRichiesta *richiesta)
{
    uint8_t b = 0;
    /* Check dei byte di sincronismo del frame ricevuto (un byte alla volta) */
    do
    {
        if (HAL_UART_Receive(&huart2, &b, 1, HAL_MAX_DELAY) != HAL_OK)
            return 0;
        if (b == HIL_SYNC0
            && HAL_UART_Receive(&huart2, &b, 1, 20) == HAL_OK && b == HIL_SYNC1)
            break;
    } while (1);
    uint8_t corpo[sizeof(HilRichiesta) + 1U]; // il +1 è per il checksum
    if (HAL_UART_Receive(&huart2, corpo, sizeof corpo, 100) != HAL_OK)
        return 0;
    /* Verifica del checksum */
    if (Hil_Checksum(corpo, sizeof(HilRichiesta)) != corpo[sizeof(HilRichiesta)])
        return 0;
    /* Sposta 'corpo' in 'richiesta' che è una struct HilRichiesta */
    memcpy(richiesta, corpo, sizeof *richiesta);
    return 1;
}

static void Hil_Invia(const HilRisposta *risposta)
{
    uint8_t pacchetto[2U + sizeof(HilRisposta) + 1U];
    pacchetto[0] = HIL_SYNC0;
    pacchetto[1] = HIL_SYNC1;
    /* Sposta 'risposta', che è una struct HilRisposta, in pacchetto[2] occupando i successivi 60 byte */
    memcpy(&pacchetto[2], risposta, sizeof *risposta);
    pacchetto[sizeof pacchetto - 1U] = Hil_Checksum(&pacchetto[2], sizeof(HilRisposta));
    HAL_UART_Transmit(&huart2, pacchetto, sizeof pacchetto, 100);
}

/* Conversione da cicli di CPU a microsecondi */
static uint32_t Hil_Microsecondi(uint32_t cicli)
{
    return cicli / (SystemCoreClock / 1000000U);
}

/* Elabora una richiesta valida: stesso stimatore e stessa rete del controllo. */
static void Hil_Elabora(const HilRichiesta *r, HilRisposta *out)
{
    memset(out, 0, sizeof *out);
    out->seq = r->seq;
    /* comando 0 = inizio episodio, 1 = passo di CONTROL_PERIOD_MS
    * 2 = ping.*/
    if (r->comando == 2U)
        return;
    uint8_t valida = (r->comando <= 1U) && r->n <= HIL_MAX_PREDIZIONI
        && isfinite(r->x) && isfinite(r->theta) && isfinite(r->applied_hz);
    for (uint32_t i = 0; valida && i < r->n; ++i)
        valida = isfinite(r->v[i]);
    /* Assegnazione codice di errore */
    if (!valida || !rete_pronta)
    {
        out->errore = !rete_pronta ? 2U : 8U;
        return;
    }
    /* Lettura del valore corrente di cicli della CPU */
    uint32_t inizio = DWT->CYCCNT;
    if (r->comando == 0U)
        Stima_Reset(&hil_stima, r->x, r->theta, 0.0f);
    else
    {
        for (uint32_t i = 0; i < r->n; ++i)
            Stima_Predici(&hil_stima, r->v[i], MOTOR_PERIOD_MS * 0.001f);
        Stima_Correggi(&hil_stima, r->x, r->theta);
    }
    Rete_Osservazione(&hil_stima, r->applied_hz, rete_osservazione);
    ai_float *input = (ai_float *)rete_input[0].data;
    for (uint32_t i = 0; i < 6U; ++i)
    {
        out->osservazione[i] = rete_osservazione[i];
        input[i] = rete_osservazione[i];
    }
#if CFG_STORIA_N > 0
    if (r->comando == 0U)
        Storia_Reset(rete_osservazione);
    Storia_Ingressi(input);
#endif
    out->stima_x = hil_stima.x;
    out->stima_theta = hil_stima.theta;
    out->stima_omega = hil_stima.omega;
    uint32_t inizio_rete = DWT->CYCCNT;
    ai_i32 batches = ai_network_run(rete, rete_input, rete_output);
    uint32_t fine = DWT->CYCCNT;
    out->rete_us = Hil_Microsecondi(fine - inizio_rete);
    out->totale_us = Hil_Microsecondi(fine - inizio);
    if (batches != 1)
    {
        rete_ultimo_errore = ai_network_get_error(rete);
        out->errore = 2U;
        return;
    }
    float action = ((ai_float *)rete_output[0].data)[0];
    if (!isfinite(action))
    {
        out->errore = 3U;
        return;
    }
    action = fmaxf(-1.0f, fminf(action, 1.0f));
#if CFG_STORIA_N > 0
    Storia_Aggiungi(rete_osservazione, action);
#endif
    out->azione = action;
    out->richiesta_hz = Rete_AzioneInHz(action);
    rete_azione = action;
    frequenza_richiesta_hz = out->richiesta_hz;
    ++rete_inferenze;
}


void Hil_Esegui(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Attiva il tracciamento del core Cortex-M
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Abilita il contatore cicli CYCCNT del blocco DWT
    Stepper_Stop();
    uint8_t prima = 1;
    HilRichiesta richiesta;
    while (1)
    {
        if (!Hil_Ricevi(&richiesta))
        {
            ++hil_scartati;
            continue;
        }
        /* 
        * Una richiesta con lo stesso seq della precedente non viene rieseguita, ma
        * si rispedisce la risposta precedente. */
        if (prima || richiesta.seq != hil_risposta.seq || richiesta.comando == 2U)
            Hil_Elabora(&richiesta, &hil_risposta);
        prima = 0;
        Hil_Invia(&hil_risposta);
        ++hil_pacchetti;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}
#endif 
