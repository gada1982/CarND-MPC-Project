#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
private:
  double steer_;
  double throttle_;
  const int max_steer_deg_ = 25;
  const double max_steer_rad_ = max_steer_deg_*(M_PI/180);
  
  
 public:
  vector<double> predicted_path_x_;
  vector<double> predicted_path_y_;
  
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  // Transform waypoints from map's coordinate system to car's coordinate system
  void Transform_Map_to_Car(const vector<double>& map_ptsx, const vector<double>& map_ptsy, double map_px, double map_py, double map_psi, vector<double>& car_ptsx, vector<double>& car_ptsy);
  
  /*
   * Return the value for steering
   */
  double ReturnSteerValue();
  
  /*
   * Return the value for throttle
   */
  double ReturnThrottleValue();
};

#endif /* MPC_H */
