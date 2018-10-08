#pragma once

#include <vector>
#include <Eigen/Core>


class MPC {
public:
    MPC();

    virtual ~MPC();

    struct Actuations {
        const double steering;
        const double throttle;
    };

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    Actuations Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs);
};
