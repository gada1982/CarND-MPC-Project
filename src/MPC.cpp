/*
 * MPC.cpp
 *
 * Created on: June 09, 2017
 * Author: Daniel Gattringer
 * Mail: daniel@gattringer.biz
 */

#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

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
const double Lf = 2.67;

// NOTE: feel free to play around with this
// or do something completely different
double ref_v = 50.0;

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
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;
    
    // Reference State Cost
    // Define the cost related the reference state and
    // any anything you think may be beneficial.
    
    // The part of the cost based on the reference state.
    double mult_ref_cte_epsi = 150.0; // TODO
    double mult_ref_v = 1.0; // TODO
    for (int t = 0; t < N; t++) {
      fg[0] += mult_ref_cte_epsi * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += mult_ref_cte_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += mult_ref_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // Minimize the change-rate of using the actuators (steering, throttle/brake) to get a smoother driving
    double mult_change_rate = 1.0; // TODO
    for (int t = 0; t < N - 1; t++) {
      fg[0] += mult_change_rate * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += mult_change_rate * CppAD::pow(vars[a_start + t], 2);
    }
    
    // Minimize the value gap between sequential actuations.
    // The next control input should be similar to the current one.
    double mult_gap_actions = 450.0; // TODO
    for (int t = 0; t < N - 2; t++) {
      fg[0] += mult_gap_actions* CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += mult_gap_actions * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    // Minimize the value gap between sequential actuations.
    
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
      
      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*pow(x0, 2) + coeffs[3]*pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3] * pow(x0, 2));
      
      // Setup the rest of the model constraints
      fg[1 + x_start + t] = x1 - (x0 + (v0 * CppAD::cos(psi0) * dt));
      fg[1 + y_start + t] = y1 - (y0 + (v0 * CppAD::sin(psi0) * dt));
      fg[1 + psi_start + t] = psi1 - (psi0 + (v0 * delta0 / Lf * dt));
      fg[1 + v_start + t] = v1 - (v0 + (a0 * dt));
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + (v0 * delta0 / Lf * dt));
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6*N + 2*(N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = 6*N;
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -std::numeric_limits<double>::max();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }
  
  // Set lower and upper limits for steering angle
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  
  // Set lower and upper limits for throttle
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
  
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);
  //std::cout<<"FG_eval complete"<<std::endl;
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
  options += "Numeric max_cpu_time          1.0\n";
  
  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;
  
  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
                                        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                        constraints_upperbound, fg_eval, solution);
  //std::cout<<"Solve complete"<<std::endl;
  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  
  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  
  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> solution_x;
  
  for (int i = 0; i < n_vars; i++) {
    solution_x.push_back(solution.x[i]);
  }
  
  return solution_x;
}

void MPC::Transform_Map_to_Car(const vector<double>& map_ptsx, const vector<double>& map_ptsy, double map_px, double map_py, double map_psi, vector<double>& car_ptsx, vector<double>& car_ptsy) {
  
  assert(map_ptsx.size() == map_ptsy.size());
  
  car_ptsx.resize(map_ptsx.size());
  car_ptsy.resize(map_ptsy.size());
  
  const double cos_psi = cos(map_psi);
  const double sin_psi = sin(map_psi);
  
  for (int i = 0; i < car_ptsx.size(); i++) {
    const double map_x = map_ptsx[i] - map_px;
    const double map_y = map_ptsy[i] - map_py;
    car_ptsx[i] = map_x*cos_psi + map_y*sin_psi;
    car_ptsy[i] = -map_x*sin_psi + map_y*cos_psi;
  }
}

int MPC::getN() {
  return N;
}

vector<size_t> MPC::getIdx() {
  vector<size_t> idx = {x_start, y_start, psi_start, v_start, cte_start, epsi_start, delta_start, a_start };
  return idx;
}
