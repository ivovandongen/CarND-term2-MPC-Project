#pragma once

#include <vector>
#include <Eigen/Core>


class MPC {
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
