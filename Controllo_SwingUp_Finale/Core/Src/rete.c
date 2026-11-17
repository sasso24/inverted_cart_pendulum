/* Rete X-CUBE-AI: inizializzazione, cambio rete, ingressi dalla stima,
 * storia degli ingressi e conversione dell'uscita (vedi rete.h). */
#include "rete.h"
#include "controllo.h"
#include <math.h>
#include "network_data.h"
#include "reti_candidate.h"    /* generato da esporta_firmware.py (pesi delle reti) */
#include "stepper.h"

#define POSIZIONE_SCALA_M        CFG_POSIZIONE_SCALA_M
#define VELOCITA_SCALA_M_S       CFG_VELOCITA_SCALA_M_S
#define OMEGA_SCALA_RAD_S        CFG_OMEGA_SCALA_RAD_S

/* Ingressi: i sei attuali + 7 per ogni ritardo della storia (config_controllo.h). */
#if AI_NETWORK_IN_1_SIZE != CFG_STORIA_INGRESSI || AI_NETWORK_OUT_1_SIZE != 1
#error "La rete X-CUBE-AI non ha CFG_STORIA_INGRESSI ingressi e una uscita: rieseguire esporta_firmware.py"
#endif

volatile uint8_t rete_pronta = 0;
volatile float rete_osservazione[6];
volatile float rete_azione = 0.0f;
volatile uint32_t rete_inferenze = 0;
volatile uint32_t rete_durata_ms = 0;
ai_handle rete = AI_HANDLE_NULL;
/* Piu' reti nello stesso firmware (reti_candidate.c), stessa configurazione.
 * Per cambiarla: a controllo fermo scrivere rete_scelta in Live Expressions;
 * il cambio avviene subito nel ciclo principale e rete_attiva lo conferma.
 * reti_candidate_nomi[rete_attiva] dice quale rete e'. */
volatile uint32_t rete_scelta = 0;
volatile uint32_t rete_attiva = 0;
AI_ALIGNED(4) static ai_u8 rete_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
ai_buffer *rete_input;
ai_buffer *rete_output;
volatile ai_error rete_ultimo_errore;

#if CFG_STORIA_N > 0
/* Storia degli ingressi (ConStoria in ambiente.py): per ogni ritardo k la rete riceve i sei
 * ingressi di k periodi fa e l'azione decisa allora. Anello di CFG_STORIA_MAX
 * periodi: storia_testa = periodo precedente (ritardo 1). Ad ogni avvio la storia
 * e' riempita con i primi ingressi e azione 0, come all'inizio di un episodio. */
static const uint32_t storia_ritardi[CFG_STORIA_N] = CFG_STORIA_RITARDI;
static float storia_passato[CFG_STORIA_MAX][7];
static uint32_t storia_testa = 0;
volatile uint8_t storia_da_azzerare = 1;
#endif

uint8_t Rete_Init(void)
{
    const ai_handle activations[] = { AI_HANDLE_PTR(rete_activations) };
    const ai_handle pesi[] = { reti_candidate_pesi[rete_attiva] };
    /* Ogni rete ha i guadagni dello stimatore con cui e' stata provata. */
    stima_l_theta = reti_candidate_stima[rete_attiva].l_theta;
    stima_l_omega = reti_candidate_stima[rete_attiva].l_omega;
    stima_l_b = reti_candidate_stima[rete_attiva].l_b;
    stima_l_x = reti_candidate_stima[rete_attiva].l_x;
    rete_ultimo_errore = ai_network_create_and_init(&rete, activations, pesi);
    if (rete_ultimo_errore.type != AI_ERROR_NONE)
        return 0;
    rete_input = ai_network_inputs_get(rete, NULL);
    rete_output = ai_network_outputs_get(rete, NULL);
    if (rete_input == NULL || rete_output == NULL
        || rete_input[0].data == AI_HANDLE_NULL
        || rete_output[0].data == AI_HANDLE_NULL)
        return 0;
    rete_pronta = 1;
    return 1;
}

/* Solo dal main e a controllo fermo: distrugge la rete attuale e la ricrea
 * con i pesi di rete_scelta (stessa struttura X-CUBE-AI, cambia solo l'array). */
void Rete_CambiaSeRichiesto(void)
{
    uint32_t scelta = rete_scelta;
    if (controllo_abilitato || scelta == rete_attiva || scelta >= RETI_CANDIDATE_N)
        return;
    rete_pronta = 0;
    if (rete != AI_HANDLE_NULL)
        rete = ai_network_destroy(rete);
    rete_attiva = scelta;
    if (!Rete_Init())
        controllo_errore = 1;
}

/* I sei ingressi della rete dalla stima: stesse scale dell'addestramento. */
void Rete_Osservazione(const Stima *st, float applied_hz, volatile float obs[6])
{
    obs[0] = st->x / POSIZIONE_SCALA_M;
    obs[1] = sinf(st->theta);
    obs[2] = cosf(st->theta);
    obs[3] = st->velocita / VELOCITA_SCALA_M_S;
    obs[4] = st->omega / OMEGA_SCALA_RAD_S;
    obs[5] = applied_hz / MAX_FREQUENCY;
}

#if CFG_STORIA_N > 0
void Storia_Reset(const volatile float obs[6])
{
    for (uint32_t k = 0; k < CFG_STORIA_MAX; ++k)
    {
        for (uint32_t i = 0; i < 6U; ++i)
            storia_passato[k][i] = obs[i];
        storia_passato[k][6] = 0.0f;
    }
    storia_testa = 0;
}

/* Dopo ogni inferenza: gli ingressi e l'azione di adesso diventano il ritardo 1. */
void Storia_Aggiungi(const volatile float obs[6], float azione)
{
    storia_testa = (storia_testa + CFG_STORIA_MAX - 1U) % CFG_STORIA_MAX;
    for (uint32_t i = 0; i < 6U; ++i)
        storia_passato[storia_testa][i] = obs[i];
    storia_passato[storia_testa][6] = azione;
}

/* Ingressi 6.. della rete: [obs(t-k), u(t-k)] per ogni ritardo k, nell'ordine di ConStoria (ambiente.py). */
void Storia_Ingressi(ai_float *input)
{
    for (uint32_t j = 0; j < CFG_STORIA_N; ++j)
    {
        const float *campione = storia_passato[(storia_testa + storia_ritardi[j] - 1U) % CFG_STORIA_MAX];
        for (uint32_t i = 0; i < 7U; ++i)
            input[6U + 7U * j + i] = campione[i];
    }
}
#endif

/* Uscita della rete [-1,1] -> frequenza STEP firmata, come ControlloStepper (ambiente.py)
 * (ControlloStepper.azione_a_hz), zona morta compresa. */
float Rete_AzioneInHz(float action)
{
    float modulo = fabsf(action);
    if (modulo <= CFG_ZONA_MORTA)
        return 0.0f;
    float frazione = (modulo - CFG_ZONA_MORTA) / (1.0f - CFG_ZONA_MORTA);
    return copysignf(MIN_FREQUENCY + frazione * (MAX_FREQUENCY - MIN_FREQUENCY), action);
}
