#ifdef UNIT_TEST
#include <unity.h>
#include <math.h>
#include <LQRController.hpp>
#include <StateVector.hpp>

static const double ANGLE_BOUND = 30.0 * (M_PI / 180.0); // radians
double gainVector[4] = {-2000.0, 900.0, -100.0, 300.0};

void setUp(void) {
    LQRController controller(gainVector, ANGLE_BOUND);
}

void test_controlInputIsCalculatedCorrectly(void)
{
    double pendulumAngle = 10.0 * (M_PI / 180.0);
    StateVector state(0.0872665, 0.1, 5.0, 6.0);

    double controlInput = controller.computeControlInput(state);

    double expectedResult = gainVector[0]*state.pendulumAngle + gainVector[1]*state.cartPosition + gainVector[2]*state.pendulumAngularVelocity + gainVector[3]*state.cartVelocity;
    TEST_ASSERT_EQUAL(expectedResult, controlInput);
}

void test_controlInputIsZeroWhenPendulumIsOutOfBounds(void) {
    double pendulumAngle = 31.0 * (M_PI / 180.0);
    StateVector state(pendulumAngle, 0.1, 5.0, 6.0);

    double controlInput = controller.computeControlInput(state);

    double expectedResult = 0;
    TEST_ASSERT_EQUAL(expectedResult, controlInput);
}

void runTests()
{
    UNITY_BEGIN();
    RUN_TEST(test_controlInputIsCalculatedCorrectly);
    RUN_TEST(test_controlInputIsZeroWhenPendulumIsOutOfBounds);
    UNITY_END();
}

int main(int argc, char **argv)
{
    runTests();
    return 0;
}

#endif