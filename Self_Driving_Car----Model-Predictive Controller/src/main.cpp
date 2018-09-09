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


using json = nlohmann::json;
using namespace std;
using namespace Eigen;
using Eigen::VectorXd;
using Eigen::MatrixXd;

string hasData(string s) 
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) 
{
    return "";
  } 
else if (b1 != string::npos && b2 != string::npos) 
{
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double polyeval(VectorXd temp_coefficient, double x) 
{
  double final_value;
  final_value = 0.0;
  int integer1 = 0;
  for (integer1 = 0; integer1 < temp_coefficient.size(); integer1++)
 {
    final_value = final_value + temp_coefficient[integer1] * pow(x, integer1);
  }
  return final_value;
}

VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) 
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  MatrixXd A(xvals.size(), order + 1);
  int integer2 = 0;
  for (integer2 = 0; integer2 < xvals.size(); integer2++)  
  {
    A(integer2, 0) = 1.0;
  }
  int integer_i = 0;
  int integer_j = 0;
  for (integer_j = 0; integer_j < xvals.size(); integer_j++) 
  {
    for (integer_i = 0; integer_i < order; integer_i++) 
  {
      A(integer_j, integer_i + 1) = A(integer_j, integer_i) * xvals(integer_j);
    }
  }

  auto Q = A.householderQr();
  auto final_value = Q.solve(yvals);
  return final_value;
}

int main() 
{
  uWS::Hub h;
  MPC mpc;
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
{
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') 
    {
      string s = hasData(sdata);
      if (s != "") 
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") 
	{
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v_ms = j[1]["speed"]; 
          const double v = v_ms * 0.447;
          const double steering_angle = j[1]["steering_angle"];
          const double throttle = j[1]["throttle"];
          
          const int N = ptsx.size();
          const double cospsi = cos(-psi);
          const double sinpsi = sin(-psi);
          VectorXd vehicle_x(N);
          VectorXd vehicle_y(N);
          for(int i = 0; i < N; i++) 
	  {
            const double dx = ptsx[i] - px;
            const double dy = ptsy[i] - py;
            vehicle_x[i] = dx * cospsi - dy * sinpsi;
            vehicle_y[i] = dy * cospsi + dx * sinpsi;
          }
          
          auto coeffs = polyfit(vehicle_x, vehicle_y, 3);
          const double cte = coeffs[0];
          const double epsi = -atan(coeffs[1]); 
          const double px_act = v * DT;
          const double py_act = 0;
          const double psi_act = - v * steering_angle * DT / LF;
          const double v_act = v + throttle * DT;
          const double cte_act = cte + v * sin(epsi) * DT;
          const double epsi_act = epsi + psi_act; 
          VectorXd state(6);
          state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
          vector<double> mpc_results = mpc.Solve(state, coeffs);
          double degree_to_radian = 25* M_PI / 180;
          double steer_value = mpc_results[0]/ degree_to_radian; 
          double throttle_value = mpc_results[1];

          json msgJson;
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;
          vector<double> mpc_x_vals = mpc.mpc_x;
          vector<double> mpc_y_vals = mpc.mpc_y;
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for(int i = 0; i<ptsx.size();i++){
            next_x_vals.push_back(vehicle_x[i]);
            next_y_vals.push_back(vehicle_y[i]);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          this_thread::sleep_for(chrono::milliseconds(int(DT*1000)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
	else 
	{
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } 
else 
{
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
