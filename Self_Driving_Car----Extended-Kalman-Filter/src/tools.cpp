#include <iostream>
#include "tools.h"
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::size_t;
VectorXd output;

Tools::Tools() 
{
	
}

Tools::~Tools() 
{
	
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) 
{
  /*
  RMSE
  */
  VectorXd output = Eigen::VectorXd::Zero(estimations[0].rows(), estimations[0].cols());
  for (size_t i = 0U; i < estimations.size(); ++i)
    {
        VectorXd error = ground_truth[i] - estimations[i];
        output = output + error.cwiseProduct(error);
    }

  output = output/estimations.size();
  VectorXd output_sqrt = output.cwiseSqrt();
  return output_sqrt;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
  /*
   Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

  //pre-compute a set of terms to avoid repeated calculation
  float sqr_sum = x_state(0)*x_state(0)+x_state(1)*x_state(1);
  float sqrt_sum = sqrt(sqr_sum);
  float tot_sum = (sqr_sum*sqrt_sum);

  //check division by zero
  if (fabs(sqr_sum) < 0.0001) 
  {
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (x_state(0)/sqrt_sum), (x_state(1)/sqrt_sum), 0, 0,
      -(x_state(1)/sqr_sum), (x_state(0)/sqr_sum), 0, 0,
      x_state(1)*(x_state(2)*x_state(1) - x_state(3)*x_state(0))/tot_sum, x_state(0)*(x_state(0)*x_state(3) - x_state(1)*x_state(2))/tot_sum, x_state(0)/sqrt_sum, x_state(1)/sqrt_sum;

  return Hj;
}
