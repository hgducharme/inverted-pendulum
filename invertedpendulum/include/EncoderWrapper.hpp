#pragma once

#include "Encoder.h"

class EncoderWrapper {
    private:
        Encoder &encoder;
        const double pulsePerRevolution;

    public:
        EncoderWrapper(Encoder &e, double PPR);
        long read();
        double readRadians();
        double getPulsePerRevolution();
};