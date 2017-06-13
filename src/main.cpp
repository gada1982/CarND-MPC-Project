/*
 * main.cpp
 *
 * Created on: June 09, 2017
 * Author: Daniel Gattringer
 * Mail: daniel@gattringer.biz
 */

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);
  
  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }
  
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  
  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;
  
  // Initialize model prediction control (MPC)
  MPC mpc;
  
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;  // Include for debugging
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> map_ptsx = j[1]["ptsx"];
          vector<double> map_ptsy = j[1]["ptsy"];
          double map_px = j[1]["x"];
          double map_py = j[1]["y"];
          double map_psi = j[1]["psi"];
          double v_miles = j[1]["speed"];
          // Convert from miles/h to m/s
          double v_m = v_miles * 0.44704;
          double steering_angle = j[1]["steering_angle"];
          
          vector<double> car_ptsx, car_ptsy;
          Eigen::VectorXd car_x, car_y;
          
          // Transform waypoints from map's coordinate system to car's coordinate system
          mpc.Transform_Map_to_Car(map_ptsx, map_ptsy, map_px, map_py, map_psi, car_ptsx, car_ptsy, car_x, car_y);
          
          // // Fit a polynomial
          // Third-degree polynomials are common since they can fit most roads.
          auto coeffs = polyfit(car_x, car_y, 3);
          
          // Handle system latency of 100ms
          // Predict state after latency
          double dt = 0.1;
          double car_px = v_m*dt;
          const double Lf = 2.67;
          double car_psi = -v_m*steering_angle*dt/Lf;
          
          // Evaluate a polynomial
          double cte = polyeval(coeffs, car_px);
          double epsi = -atan(coeffs[1] + 2*car_px*coeffs[2] + 3*pow(car_px, 2)*coeffs[3]);
          std::cout << "CTE: " << cte << std::endl;
          std::cout << "Epsi: " << epsi << std::endl;
          
          Eigen::VectorXd state(6);
          state << car_px, 0.0, car_psi, v_miles, cte, epsi;
          
          // Get start indexes for single parts of the vector
          vector<size_t> idx = mpc.getIdx();
          size_t x_start = idx[0];
          size_t y_start = idx[1];
          size_t psi_start = idx[2];
          size_t v_start = idx[3];
          size_t cte_start = idx[4];
          size_t epsi_start = idx[5];
          size_t delta_start = idx[6];
          size_t a_start = idx[7];
          
          // Solve the model given an initial state and polynomial coefficients
          auto vars = mpc.Solve(state, coeffs);
          
          // Normalize steer_value to [-1, 1]
          double steer_value = -vars[delta_start] / deg2rad(25);
          
          double throttle_value = vars[a_start];
          
          std::cout << "Steering Value: " << steer_value << std::endl;
          std::cout << "Throttle Value: " << throttle_value << std::endl << std::endl;
          
          json msgJson;
          
          // Send the steering value back [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          
          // Display the MPC predicted trajectory path as a green line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (int i = 1; i < mpc.getN(); i++) {
             mpc_x_vals.push_back(vars[x_start + i]);
             mpc_y_vals.push_back(vars[y_start + i]);
           }
          
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          
          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = car_ptsx;
          msgJson["next_y"] = car_ptsy;
          
          
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
  
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
  
  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
