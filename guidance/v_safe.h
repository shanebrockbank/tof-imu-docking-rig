#ifndef GUIDANCE_V_SAFE_H
#define GUIDANCE_V_SAFE_H

/* v_safe(r) = clamp(k*sqrt(r), v_floor, v_cap) — the glideslope-style
   range-gated closing-speed limit (docs/design.md §7.1). */
double v_safe(double range_m);

#endif /* GUIDANCE_V_SAFE_H */
