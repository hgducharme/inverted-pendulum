#pragma once

#include <StateVector.hpp>
#include "EncoderWrapper.hpp"

class StateUpdater {
    private:
        const double IDLER_PULLEY_RADIUS;
        const double SYSTEM_LOOP_RATE_SECONDS;
        EncoderWrapper &cartEncoder;
        EncoderWrapper &pendulumEncoder;

    public:
        StateUpdater(EncoderWrapper &c, EncoderWrapper &p, double pulleyRadius, double loopRate);
        void update(StateVector &state, const StateVector &previousState);
        void archiveCurrentState(const StateVector &currentState, StateVector &previousState);

        private: 
        double calculatePendulumAngle();
        double calculateCartPosition();
        double calculatePendulumAngularVelocity(double currentPendulumAngle, double previousPendulumAngle);
        double calculateCartVelocity(double currentCartPosition, double previousCartPosition);
        double normalizeAngle(double angle);
};