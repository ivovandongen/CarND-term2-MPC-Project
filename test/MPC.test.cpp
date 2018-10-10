#include <test.hpp>

#include <MPC.hpp>
#include <util.hpp>

#include <vector>

TEST(Core, PredictState) {
    MPC mpc;

    MPC::State state{0, 0, deg2rad(45), 1, 0.1, 0.1};
    MPC::Actuators actuators{deg2rad(5), 1};

    auto nextState = mpc.predict(state, actuators, 0.3);

    ASSERT_FLOAT_EQ(nextState.x, 0.212132);
    ASSERT_FLOAT_EQ(nextState.y, 0.212132);
    ASSERT_FLOAT_EQ(nextState.psi, 0.79520339);
    ASSERT_FLOAT_EQ(nextState.v, 1.3);
    ASSERT_FLOAT_EQ(nextState.cte, 0.12995);
    ASSERT_FLOAT_EQ(nextState.epsi, 0.10980522);
}