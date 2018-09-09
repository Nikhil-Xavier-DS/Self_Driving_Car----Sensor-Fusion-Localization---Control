#ifndef MPC_H
#define MPC_H
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#define DT 0.1 
#define LF 2.67
#define W_DDELTA 0
#define W_DA 0.0

using namespace std;
class MPC {
 public:
  MPC();
  virtual ~MPC();
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif
