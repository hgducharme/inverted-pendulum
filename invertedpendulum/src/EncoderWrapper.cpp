#include "EncoderWrapper.hpp"

EncoderWrapper::EncoderWrapper(Encoder &e, double PPR) : encoder(e), pulsePerRevolution(PPR) {}; 

long EncoderWrapper::read() {
    return encoder.read();
}

double EncoderWrapper::readRadians() {
    return (read() / pulsePerRevolution) * (2.0 * PI);
}

double EncoderWrapper::getPulsePerRevolution() {
    return pulsePerRevolution;
}