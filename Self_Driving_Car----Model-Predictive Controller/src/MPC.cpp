#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

const size_t N = 20;
class FG_eval 
{
 public:
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fitting_polynomial, const ADvector& variables) 
{
    fitting_polynomial[0] = 0;
    int index1 = 0;
    for (index1 = 0; index1 < 20; index1++) 
{
      fitting_polynomial[0] = fitting_polynomial[0] + 2 * CppAD::pow(variables[80 + index1], 2);
      fitting_polynomial[0] = fitting_polynomial[0] + 20 * CppAD::pow(variables[100 + index1], 2);
      fitting_polynomial[0] = fitting_polynomial[0] + CppAD::pow(variables[60 + index1] - 40, 2);
    }
    int index2 = 0;
    for (index2 = 0; index2 < 19; index2++) 
{
      fitting_polynomial[0] = fitting_polynomial[0] + 100000 * CppAD::pow(variables[120 + index2], 2);
      fitting_polynomial[0] = fitting_polynomial[0] + 20 * CppAD::pow(variables[139 + index2], 2);
    }
    int index3 = 0;
    for (index3 = 0; index3 < 18; index3++) 
{
      fitting_polynomial[0] = fitting_polynomial[0] + W_DDELTA * CppAD::pow(variables[120 + index3 + 1] - variables[120 + index3], 2);
      fitting_polynomial[0] = fitting_polynomial[0] + W_DA * CppAD::pow(variables[139 + index3 + 1] - variables[139 + index3], 2);
    }
    
    fitting_polynomial[1] = variables[0];
    fitting_polynomial[21] = variables[20];
    fitting_polynomial[41] = variables[40];
    fitting_polynomial[61] = variables[60];
    fitting_polynomial[81] = variables[80];
    fitting_polynomial[101] = variables[100];
    int index4 = 0;
    for (index4 = 0; index4 < 19; index4++) 
{
      AD<double> x1 = variables[index4 + 1];
	AD<double> x0 = variables[index4];

      AD<double> y1 = variables[21 + index4];
	AD<double> y0 = variables[20 + index4];

      AD<double> psi1 = variables[index4 + 41];
	AD<double> psi0 = variables[40 + index4];

      AD<double> v1 = variables[index4 + 61];
	AD<double> v0 = variables[60 + index4];

      AD<double> cte1 = variables[index4 + 81];
	AD<double> cte0 = variables[80 + index4];

      AD<double> epsi1 = variables[index4 + 101];
	AD<double> epsi0 = variables[100 + index4];

      AD<double> delta0 = variables[120 + index4];
      AD<double> a0 = variables[139 + index4];

      AD<double> f0 = 0.0;
	int index5 = 0;
      for (index5 = 0; index5 < coeffs.size(); index5++) 
	{
        f0 = f0 + coeffs[index5] * CppAD::pow(x0, index5);
      }
      AD<double> psides0 = 0.0;
	int index6 = 0;
      for (index6 = 1; index6 < coeffs.size(); index6++) {
        psides0 = psides0+ index6*coeffs[index6] * CppAD::pow(x0, index6-1); 
      }
      psides0 = CppAD::atan(psides0);

      fitting_polynomial[2+ index4] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
      fitting_polynomial[22 + index4] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
      fitting_polynomial[42 + index4] = psi1 - (psi0 + v0 * delta0 / LF * DT);
      fitting_polynomial[62 + index4] = v1 - (v0 + a0 * DT);
      fitting_polynomial[82 + index4] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * DT));
      fitting_polynomial[102 + index4] = epsi1 - ((psi0 - psides0) + v0 * delta0 / LF * DT);
    }
  }
};

MPC::MPC() 
{

}
MPC::~MPC() 
{

}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  size_t n_variable = 158;
  size_t n_constraints = 120;

  Dvector variable(n_variable);
  int index1 = 0;
  for (index1 = 0; index1 < n_variable; index1++) 
  {
    variable[index1] = 0;
  }

  Dvector variable_lowerbound(n_variable);
  Dvector variable_upperbound(n_variable);
  int index2 = 0;
  for (index2 = 0; index2 < 120; index2++) 
{
    variable_lowerbound[index2] = -1.0e3;
    variable_upperbound[index2] = 1.0e3;
  }
  int index3 = 120; 
  for (index3 = 120; index3 < 139; index3++) 
{
    variable_lowerbound[index3] = -0.436332;
    variable_upperbound[index3] = 0.436332;
  }
  int index4 = 139;
  for (index4  = 139; index4  < n_variable; index4++) 
{
    variable_lowerbound[index4] = -1.0;
    variable_upperbound[index4] = 1;
  }
  
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  int index5 = 0;
  for (index5 = 0; index5 < n_constraints; index5++) 
{
    constraints_lowerbound[index5] = 0;
    constraints_upperbound[index5] = 0;
  }
  constraints_lowerbound[0] = state[0];
constraints_upperbound[0] = state[0];

  constraints_lowerbound[20] = state[1];
constraints_upperbound[20] = state[1];

  constraints_lowerbound[40] = state[2];
constraints_upperbound[40] = state[2];

  constraints_lowerbound[60] = state[3];
constraints_upperbound[60] = state[3];

  constraints_lowerbound[80] = state[4];
constraints_upperbound[80] = state[4];

  constraints_lowerbound[100] = state[5];
  constraints_upperbound[100] = state[5];

  FG_eval fg_eval(coeffs);
  std::string options;
  options += "Integer print_level  0\n";

  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";
  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, variable, variable_lowerbound, variable_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  this->mpc_x = {};
  this->mpc_y = {};
  int index6 = 0;
  for (index6 = 0; index6 < N; index6++) {
    this->mpc_x.push_back(solution.x[index6]);
    this->mpc_y.push_back(solution.x[20 + index6]);
  }
  vector<double> final_value;
  final_value.push_back(solution.x[120]);
  final_value.push_back(solution.x[139]);
  return final_value;
}
