/* Stimatore dello stato del pendolo (costanti da config_controllo.h). */
#ifndef STIMA_H
#define STIMA_H

#include "config_controllo.h"

#ifndef STIMA_L_B
#define STIMA_L_B      0.0f
#endif
#ifndef STIMA_B_MAX_RAD
#define STIMA_B_MAX_RAD 0.2617994f
#endif

typedef struct
{
    float x;                    /* m, rispetto al centro della corsa */
    float theta;                /* angolo vero stimato in rad, 0 in alto, in (-pi, pi]      */
    float omega;                /* rad/s                             */
    float velocita;             /* m/s, ultima velocita' applicata   */
    float velocita_precedente;  /* m/s, per l'accelerazione          */
    float offset;               /* rad, errore stimato dell'encoder  */
} Stima;

/* Guadagni di correzione in uso: all'avvio quelli di config_controllo.h,
 * poi quelli della rete scelta. */
extern float stima_l_theta, stima_l_omega, stima_l_b, stima_l_x;

/* Prima lettura: pendolo considerato fermo. */
void Stima_Reset(Stima *s, float x, float theta, float velocita);

/* Predizione (a ogni passo della rampa, 2ms, velocita' applicata v):
 *   a      = (v - v_precedente) / dt
 *   alfa   = K_G*sin(theta) - K_A*cos(theta)*a - K_D*omega
 *   omega += dt*alfa 
 *   theta += dt*omega 
 *   x += dt*v*/
void Stima_Predici(Stima *s, float velocita_applicata, float dt);

/* Correzione (una volta per periodo della rete, 20ms, usando posizione e angolo letti):
 *   r = wrap(theta_mis - theta - offset)
 *   offset += L_B*max(cos(theta), 0)*r   (limitato a [-B_MAX_RAD;B_MAX_RAD], se L_B = 0: niente offset)
 *   theta += L_THETA*r 
 *   omega += L_OMEGA*r 
 *   x += L_X*(x_mis - x) */
void Stima_Correggi(Stima *s, float x_misurata, float theta_misurato);

/* Angolo riportato in (-pi, pi]. */
float Stima_Wrap(float angolo);

#endif
