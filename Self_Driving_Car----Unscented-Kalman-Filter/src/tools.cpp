#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
    VectorXd root_mean_square_error(4);
    root_mean_square_error << 0,0,0,0;
    if((estimations.size() == 0) || (estimations.size() != ground_truth.size()))
	{
        return root_mean_square_error;
    }
	int index1 = 0;
    for(index1=0; index1<estimations.size(); ++index1)
	{
        VectorXd root_mean_square_error_temp;
		root_mean_square_error_temp = estimations[index1] - ground_truth[index1];
        root_mean_square_error_temp = root_mean_square_error_temp.array()*root_mean_square_error_temp.array();
        root_mean_square_error = root_mean_square_error + root_mean_square_error_temp;
    }
    root_mean_square_error = root_mean_square_error/estimations.size();
    VectorXd root_mean_square_error_sqrt(4);
	root_mean_square_error_sqrt = root_mean_square_error.array().sqrt();
    return root_mean_square_error_sqrt;
}