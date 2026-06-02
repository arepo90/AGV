#include "sta.h"

#include <math.h>

void sta_init(sta_t *p, float k1, float k2,
              float u_min, float u_max, float u1_limit) {
    p->k1 = k1;
    p->k2 = k2;
    p->u_min = u_min;
    p->u_max = u_max;
    p->u1_limit = u1_limit;
    sta_reset(p);
}

void sta_set_gains(sta_t *p, float k1, float k2) {
    p->k1 = k1;
    p->k2 = k2;
}

void sta_reset(sta_t *p) {
    p->u1 = 0.0f;
}

float sta_step(sta_t *p, float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float s   = setpoint - measurement;
    float sgn = (s > 0.0f) - (s < 0.0f);   /* -1, 0, +1 */

    /* Integrator: u₁ += k₂·sign(s)·dt, with symmetric anti-windup. */
    p->u1 += p->k2 * sgn * dt;
    if (p->u1 >  p->u1_limit) p->u1 =  p->u1_limit;
    if (p->u1 < -p->u1_limit) p->u1 = -p->u1_limit;

    /* Output: continuous term + integrator. */
    float abs_s = s < 0.0f ? -s : s;
    float u     = p->k1 * sqrtf(abs_s) * sgn + p->u1;

    if (u > p->u_max) u = p->u_max;
    if (u < p->u_min) u = p->u_min;
    return u;
}
