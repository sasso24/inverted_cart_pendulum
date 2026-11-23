/* Stimatore dello stato del pendolo (costanti da config_controllo.h). */
#include "stima.h"
#include <math.h>

#define STIMA_PI     3.14159265358979323846f
#define STIMA_2PI    6.28318530717958647692f

float stima_l_theta = STIMA_L_THETA;
float stima_l_omega = STIMA_L_OMEGA;
float stima_l_b = STIMA_L_B;
float stima_l_x = STIMA_L_X;

/* Angolo riportato in (-pi, pi]. */
float Stima_Wrap(float angolo)
{
    angolo = remainderf(angolo, STIMA_2PI);
    if (angolo <= -STIMA_PI)
        angolo += STIMA_2PI;
    return angolo;
}

/* Prima lettura: pendolo considerato fermo. */
void Stima_Reset(Stima *s, float x, float theta, float velocita)
{
    s->x = x;
    s->theta = theta;
    s->omega = 0.0f;
    s->velocita = velocita;
    s->velocita_precedente = velocita;
    s->offset = 0.0f;
}

/* Predizione (a ogni passo della rampa, 2ms, velocita' applicata v):
 *   a      = (v - v_precedente) / dt
 *   alfa   = K_G*sin(theta) - K_A*cos(theta)*a - K_D*omega
 *   omega += dt*alfa 
 *   theta += dt*omega 
 *   x += dt*v*/
void Stima_Predici(Stima *s, float velocita_applicata, float dt)
{
    float accelerazione = (velocita_applicata - s->velocita_precedente) / dt;
    s->velocita_precedente = velocita_applicata;
    float alfa = STIMA_K_G * sinf(s->theta)
               - STIMA_K_A * cosf(s->theta) * accelerazione
               - STIMA_K_D * s->omega;
    s->omega += dt * alfa;
    s->theta += dt * s->omega;
    s->x += dt * velocita_applicata;
    s->velocita = velocita_applicata;
}

/* Correzione (una volta per periodo della rete, 20ms, usando posizione e angolo letti):
 *   r = wrap(theta_mis - theta - offset)
 *   offset += L_B*max(cos(theta), 0)*r   (limitato a [-B_MAX_RAD;B_MAX_RAD], se L_B = 0: niente offset)
 *   theta += L_THETA*r 
 *   omega += L_OMEGA*r 
 *   x += L_X*(x_mis - x) */
void Stima_Correggi(Stima *s, float x_misurata, float theta_misurato)
{
    float r = Stima_Wrap(theta_misurato - s->theta - s->offset); // Errore angolo
    if (stima_l_b != 0.0f)
    {
        float peso = fmaxf(cosf(s->theta), 0.0f); // 1 -> pendolo in alto, 0 -> pendolo in basso (non è booleano)
        /* Se il pendolo è in basso, non viene calcolato l'offset perchè la dinamica è stabile */
        s->offset = fmaxf(-STIMA_B_MAX_RAD,
                          fminf(STIMA_B_MAX_RAD, s->offset + stima_l_b * peso * r));
    }
    s->theta = Stima_Wrap(s->theta + stima_l_theta * r);
    s->omega += stima_l_omega * r;
    s->x += stima_l_x * (x_misurata - s->x);
}
