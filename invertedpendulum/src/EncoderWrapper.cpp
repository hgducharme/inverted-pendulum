#include "EncoderWrapper.hpp"

EncoderWrapper::EncoderWrapper(Encoder e, double PPR) : encoder(e), pulsePerRevolution(PPR) {}; 

long EncoderWrapper::read() {
    long value = encoder.read();
    return value;
}