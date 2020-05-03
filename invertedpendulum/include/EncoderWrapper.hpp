#pragma once

#include "Encoder.h"

class EncoderWrapper {
    private:
        const double pulsePerRevolution;
        Encoder encoder;

    public:
        EncoderWrapper(Encoder e, double PPR);
        long read();
        double getPulsePerRevolution();
};