#ifndef IKCONSTRAINT_JOINTANGLEFULLCONSTRAINT_H
#define IKCONSTRAINT_JOINTANGLEFULLCONSTRAINT_H

#include <ik_constraint/IKConstraint.h>
#include <cnoid/EigenUtil>

namespace IK{
  class JointAngleFullConstraint : public IKConstraint
  {
  public:
    //jointのqとtargetqを一致させる.
    //  maxError: エラーの頭打ち
    //  precision: 収束判定の閾値
    //  weight: コスト関数の重み. error * weight^2 * error.

    // free joint を除いて初期化すること．calc_jacobian時にfree joint込のjacobianが作られる．
    const std::vector<cnoid::LinkPtr>& joints() const { return joints_;}
    std::vector<cnoid::LinkPtr>& joints() { return joints_;}
    const std::vector<double>& targetqs() const { return targetqs_;}
    std::vector<double>& targetqs() { return targetqs_;}
    const std::vector<double>& maxError() const { return maxError_;}
    std::vector<double>& maxError() { return maxError_;}
    const std::vector<double>& precision() const { return precision_;}
    std::vector<double>& precision() { return precision_;}
    const std::vector<double>& weight() const { return weight_;}
    std::vector<double>& weight() { return weight_;}

    bool checkConvergence() override;
    const Eigen::VectorXd& calc_error () override;
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& calc_jacobian (const std::vector<cnoid::LinkPtr>& joints) override;

  private:
    std::vector<cnoid::LinkPtr> joints_;
    std::vector<double> targetqs_;
    std::vector<double> precision_;
    std::vector<double> maxError_;
    std::vector<double> weight_;
  };
}

#endif
