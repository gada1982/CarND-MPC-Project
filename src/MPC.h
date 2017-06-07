#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  // Transform waypoints from map's coordinate system to car's coordinate system
  void Transform_Map_to_Car(const vector<double>& map_ptsx, const vector<double>& map_ptsy, double map_px, double map_py, double map_psi, vector<double>& car_ptsx, vector<double>& car_ptsy);
};

#endif /* MPC_H */
