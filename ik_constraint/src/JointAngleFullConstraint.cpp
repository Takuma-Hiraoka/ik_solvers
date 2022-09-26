#include <ik_constraint/JointAngleFullConstraint.h>
#include <iostream>

namespace IK{
  bool JointAngleFullConstraint::checkConvergence() {
    if(this->joints_.size() == 0) return true;

    if(this->error_.rows() != this->joints_.size()) {
      int row = 0;
      for(size_t i=0; i < this->joints_.size(); i++){
	row += this->getJointDOF(this->joints_[i]);
      }
      row += 6;
      this->error_ = Eigen::VectorXd(row);
    }
    
    for (int i=0;i<this->joints_.size();i++){
      this->error_[6+i] = this->weight_[i] * std::min(std::max(this->joints_[i]->q() - this->targetqs_[i], -this->maxError_[i]), this->maxError_[i]);
    }

    for (int i=0;i<this->joints_.size();i++){
      if (std::fabs(this->error_[6 + i]) > this->precision_[i]) return false;
    }
    return true;
  }

  const Eigen::VectorXd& JointAngleFullConstraint::calc_error() {
    if(this->debuglevel_>=1){
      std::cerr << "JointAngleConstraint" << std::endl;
      std::cerr << "error" << std::endl;
      std::cerr << this->error_ << std::endl;
    }

    return this->error_;
  }

  const Eigen::SparseMatrix<double,Eigen::RowMajor>& JointAngleFullConstraint::calc_jacobian (const std::vector<cnoid::LinkPtr>& joints) {
    if(!this->is_joints_same(joints,this->jacobian_joints_)){
      this->jacobian_joints_ = joints;
      this->jacobianColMap_.clear();
      int cols = 0;
      for(size_t i=0; i < this->jacobian_joints_.size(); i++){
        this->jacobianColMap_[this->jacobian_joints_[i]] = cols;
        cols += this->getJointDOF(this->jacobian_joints_[i]);
      }
      this->jacobian_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(cols,cols);

      for(int i=0;i<this->joints_.size();i++){
	if(this->jacobianColMap_.find(joints_[i]) != this->jacobianColMap_.end()){
	  if(this->joints_[i]->isRotationalJoint() || this->joints_[i]->isPrismaticJoint()){
	    this->jacobian_.insert(this->jacobianColMap_[this->joints_[i]],this->jacobianColMap_[this->joints_[i]]) = 1;
	  }
	}
      }
    }
    
    for(int i=0;i<this->joints_.size();i++){
	if(this->jacobianColMap_.find(joints_[i]) != this->jacobianColMap_.end()){
	  if(this->joints_[i]->isRotationalJoint() || this->joints_[i]->isPrismaticJoint()){
	    this->jacobian_.coeffRef(this->jacobianColMap_[this->joints_[i]],this->jacobianColMap_[this->joints_[i]]) = this->weight_[i];
	  }
	}
      }
    
    if(this->debuglevel_>=1){
      std::cerr << "JointAngleFullConstraint" << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return this->jacobian_;
  }
}
