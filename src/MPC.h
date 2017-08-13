#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
struct Results {
    vector<double> xs;
    vector<double> ys;
    vector<double> gaps;
    vector<double> accelerations;
};

class MPC {
public:
    MPC();

    virtual ~MPC();

    Results Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif