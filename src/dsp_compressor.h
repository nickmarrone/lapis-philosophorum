/**
 * dsp_compressor.h — Header-only log-domain feedforward compressor with
 * optional quadratic soft knee. Detector runs every sample even when
 * bypassed so metering / gain state stays warm across bypass toggles.
 */

#pragma once
#include <cmath>
#include "mastering_dsp.h"
#include "dsp_common.h"

namespace mastering_dsp {

struct Compressor {
    void Configure(const CompParams& p, float fs)
    {
        threshold_db = p.threshold_db;
        gr_slope     = 1.f / p.ratio - 1.f;
        att_coef     = MsToCoef(p.attack_ms, fs);
        rel_coef     = MsToCoef(p.release_ms, fs);
        makeup_lin   = DbToLin(p.makeup_db);
        mix          = p.mix;
        soft_knee    = p.soft_knee;
        bypass       = p.bypass;
    }

    void ProcessSample(float& l, float& r)
    {
        const float det = fmaxf(fabsf(l), fabsf(r));
        float target;
        if (det < kMinDetLin) {
            target = 0.f;
        } else {
            const float lvl_db = kLog10Scale * logf(det);
            const float over   = lvl_db - threshold_db;
            if (!soft_knee) {
                target = (over > 0.f) ? over * gr_slope : 0.f;
            } else {
                if (2.f*over <= -kKneeDb) {
                    target = 0.f;
                } else if (2.f*over >= kKneeDb) {
                    target = over * gr_slope;
                } else {
                    const float t = over + 0.5f*kKneeDb;
                    target = gr_slope * t * t / (2.f*kKneeDb);
                }
            }
        }
        const float coef = (target < gr_db) ? att_coef : rel_coef;
        gr_db += coef * (target - gr_db);
        const float g = (gr_db > -0.01f) ? 1.f : expf(kDbToLn * gr_db);
        if (!bypass) {
            const float wet_l = l * g * makeup_lin;
            const float wet_r = r * g * makeup_lin;
            l += mix * (wet_l - l);
            r += mix * (wet_r - r);
        }
    }

    float gr_db = 0.f;    // smoothed gain reduction in dB, always <= 0 (read externally for metering)

    float threshold_db = 0.f;
    float gr_slope      = 0.f;
    float att_coef       = 0.f;
    float rel_coef       = 0.f;
    float makeup_lin    = 1.f;
    float mix            = 1.f;
    bool  soft_knee     = false;
    bool  bypass         = false;
};

} // namespace mastering_dsp
