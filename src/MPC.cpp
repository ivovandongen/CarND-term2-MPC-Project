#include "MPC.hpp"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>

#include <utility>

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// Reference speed
double ref_v = 100;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:

    explicit FG_eval(Eigen::VectorXd coeffs_)
            : coeffs(std::move(coeffs_)) {}

    using ADvector = CppAD::vector<AD<double>>;

    void operator()(ADvector &fg, const ADvector &vars) {
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // Reference State Cost
        // Define the cost related the reference state and
        // any anything you think may be beneficial.

        // The part of the cost based on the reference state.
        for (int t = 0; t < N; t++) {
            fg[0] += CppAD::pow(vars[cte_start + t], 2) * mp_state_cte;
            fg[0] += CppAD::pow(vars[epsi_start + t], 2) * mp_state_epsi;
            fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2) * mp_state_v;
        }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
            fg[0] += CppAD::pow(vars[delta_start + t], 2) * mp_actuator_delta;
            fg[0] += CppAD::pow(vars[a_start + t], 2) * mp_actuator_a;
        }

        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
            fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2) * mp_seq_actuations_delta;
            fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2) * mp_seq_actuations_a;
        }

        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (int t = 1; t < N; t++) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + t];
            AD<double> y1 = vars[y_start + t];
            AD<double> psi1 = vars[psi_start + t];
            AD<double> v1 = vars[v_start + t];
            AD<double> cte1 = vars[cte_start + t];
            AD<double> epsi1 = vars[epsi_start + t];

            // The state at time t.
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> v0 = vars[v_start + t - 1];
            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> epsi0 = vars[epsi_start + t - 1];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];

            AD<double> f0 = 0;
            for (int i = 0; i < coeffs.size(); i++) {
                f0 += coeffs[i] * CppAD::pow(x0, i);
            }
            AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // Recall the equations for the model:
            // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
            // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
            // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
            // v_[t] = v[t-1] + a[t-1] * dt
            // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
            // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / MPC::Lf * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / MPC::Lf * dt);
        }
    }

private:
    // Fitted polynomial coefficients
    const Eigen::VectorXd coeffs;

    // Cost multipliers
    const int mp_state_cte = 100;
    const int mp_state_epsi = 1000;
    const int mp_state_v = 1;

    const int mp_actuator_delta = 500;
    const int mp_actuator_a = 5;

    const int mp_seq_actuations_delta = 300000;
    const int mp_seq_actuations_a = 500;
};

//
// MPC class definition implementation.
//
MPC::MPC() = default;

MPC::~MPC() = default;

MPC::State MPC::predict(const MPC::State &state, const MPC::Actuators &actuators, double dt) {
    return {
            // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
            state.x + state.v * cos(state.psi) * dt,
            // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
            state.y + state.v * sin(state.psi) * dt,
            // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
            state.psi + state.v / Lf * actuators.delta * dt,
            // v_[t] = v[t-1] + a[t-1] * dt
            state.v + actuators.a * dt,
            // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
            state.cte + state.v * sin(state.epsi) * dt,
            // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
            state.epsi + state.v / Lf * actuators.delta * dt
    };
}

MPC::Result MPC::Solve(const MPC::State &state, const Eigen::VectorXd &coeffs) {
    using Dvector = CppAD::vector<double>;

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    size_t n_vars = N * 6 + (N - 1) * 2;
    // Number of constraints
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (size_t i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (size_t i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (size_t i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for constraints
    // All of these should be 0 except the initial
    // state indices.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = state.x;
    constraints_lowerbound[y_start] = state.y;
    constraints_lowerbound[psi_start] = state.psi;
    constraints_lowerbound[v_start] = state.v;
    constraints_lowerbound[cte_start] = state.cte;
    constraints_lowerbound[epsi_start] = state.epsi;

    constraints_upperbound[x_start] = state.x;
    constraints_upperbound[y_start] = state.y;
    constraints_upperbound[psi_start] = state.psi;
    constraints_upperbound[v_start] = state.v;
    constraints_upperbound[cte_start] = state.cte;
    constraints_upperbound[epsi_start] = state.epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        std::cout << "Solution failed: " << solution.status << std::endl;
    }

    // Cost
    auto cost = solution.obj_value;

    // Path
    std::vector<Point> path;
    for (size_t i = 0; i < N - 1; i++) {
        path.push_back({solution.x[x_start + i], solution.x[y_start + i]});
    }

    return {solution.x[delta_start], solution.x[a_start], cost, std::move(path)};
}
