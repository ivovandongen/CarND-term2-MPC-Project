#pragma once

#include <vector>
#include <Eigen/Core>


class MPC {
public:
    // This value assumes the model presented in the classroom is used.
    //
    // It was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    //
    // This is the length from front to CoG that has a similar radius.
    static constexpr const double &Lf{2.67};

    MPC();

    virtual ~MPC();

    struct State {
        const double x;
        const double y;
        const double psi;
        const double v;
        const double cte;
        const double epsi;
    };

    struct Actuators {
        const double delta;
        const double a;
    };

    struct Point {
        const double x;
        const double y;
    };

    struct Result {
        const double steering;
        const double throttle;
        const double cost;
        const std::vector<Point> path;
    };

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    Result Solve(const State &state, const Eigen::VectorXd &coeffs);
};
