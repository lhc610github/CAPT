#ifndef CAPT_HPP_
#define CAPT_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <memory>
// template <int NR, int NG, int D>
class CAPT {
    public:
        CAPT(int _nr, int _ng, int _dim, double _v_max = 1.0f, int _dyn = 2) {
            Phi.reset(new Eigen::MatrixXd(_nr, _ng));
            num_robot = _nr;
            num_goal = _ng;
            dim = _dim;
            dyn = _dyn;
            v_max = _v_max;
            max_tf = 0.0f;
        }
        ~CAPT() {
        }
        
        bool set_Phi(const std::vector<int>& _assgin) {
            if (_assgin.size() != num_robot) {
                std::cout << "[CAPT]: number of robot is wrong  :" << _assgin.size() << "|" << num_robot << std::endl;
                return false;
            }
            Phi->setZero();
            for (int i = 0; i < _assgin.size(); i++) {
                if (_assgin[i] >= 0) {
                    if (_assgin[i] < num_goal) {
                        (*Phi)(i,_assgin[i]) = 1;
                    } else {
                        std::cout << "[CAPT]: assgin is wrong" << std::endl;
                    }
                }
            }

            // std::cout << Phi->transpose() << std::endl;

            if (num_robot >= num_goal) {
                Eigen::MatrixXd _Phi_T_Phi = (*Phi).transpose() * (*Phi);
                // std::cout << _Phi_T_Phi << std::endl;
                Eigen::MatrixXd _tmp_I(num_goal, num_goal);
                _tmp_I.setIdentity();
                if (_Phi_T_Phi == _tmp_I) {
                    std::cout << "[CAPT]: Phi set done" << std::endl;
                    std::cout << (*Phi) << std::endl;
                    return true;
                }
            } else {
                Eigen::MatrixXd _Phi_Phi_T = (*Phi) * (*Phi).transpose();
                // std::cout << _Phi_Phi_T << std::endl;
                Eigen::MatrixXd _tmp_I(num_robot, num_robot);
                _tmp_I.setIdentity();
                if (_Phi_Phi_T == _tmp_I) {
                    std::cout << "[CAPT]: Phi set done" << std::endl;
                    std::cout << (*Phi) << std::endl;
                    return true;
                }
            }

            return false;

        }

        bool set_init_goal(const std::vector<std::vector<double>>& _init, const std::vector<std::vector<double>>& _goal) {
            if (_init.size() == num_robot && _goal.size() == num_goal) {
                for (int i = 0; i < dim; i++) {
                    Eigen::VectorXd tmp_X0(num_robot);
                    for (int j = 0; j < num_robot; j++) {
                        tmp_X0(j) = _init[j][i];
                    }
                    X_t0.push_back(tmp_X0);

                    Eigen::VectorXd tmp_G(num_goal);
                    for (int j = 0; j < num_goal; j++) {
                        tmp_G(j) = _goal[j][i];
                    }
                    G.push_back(tmp_G);
                }
                std::cout << "[CAPT]: set init and goal done" << std::endl;
                return true;
            } else {
                std::cout << "[CAPT]: set init and goal wrong" << std::endl;
                return false;
            }
        }

        double cal_tf() {
            Eigen::MatrixXd _tmp_distance(num_robot, dim);
            _tmp_distance.setZero();
            for (int j = 0; j < num_robot; j ++) {
                if ((*Phi).row(j).norm() != 0) {
                    for (int i = 0; i < dim; i++) {
                        _tmp_distance(j, i) = (*Phi).row(j) * G[i] - X_t0[i](j);
                    }
                }
            }
            // std::cout << _tmp_distance << std::endl;
            double _max_tf = (_tmp_distance.rowwise().norm() / v_max).maxCoeff();
            std::cout << "[CAPT]: cal tf done.  tf: " << _max_tf << "s" << std::endl;
            return _max_tf;
        }

        bool cal_beta() {
            double _max_tf = cal_tf();
            max_tf = _max_tf;
            switch (dyn) {
                case 2:
                    alpha.push_back(0.0f);
                    alpha.push_back(1.0f/_max_tf);
                    break;
                default:
                    return false;
            }
            std::cout << "[CAPT]: cal beta done. ready to plan trajectory" << std::endl;
            return true;
        }

        double get_tf() {
            return max_tf;
        }

        double get_beta_from_time( double t) {
            double _beta;
            if (t > 0 && t < max_tf) {
                switch (dyn) {
                    case 2:
                        _beta = alpha[0] + alpha[1]*t;
                        break;
                    default:
                        std::cout << "[CAPT]: beta cal wrong" << std::endl;
                        return 0.0f;
                }
            }
            return _beta;
        }

        bool cal_traj(double t, std::vector<std::vector<double>>& traj) {
            if (t > 0 && t < max_tf) {
                double _beta = get_beta_from_time(t);
                for (int i = 0; i < dim; i++) {
                    Eigen::VectorXd _tmp_gamma(num_robot);
                    Eigen::MatrixXd _tmp_I(num_robot, num_robot);
                    _tmp_I.setIdentity();
                    _tmp_gamma = X_t0[i] * (1 - _beta) + ((*Phi)*G[i] + (_tmp_I - (*Phi)*((*Phi).transpose()))*X_t0[i])*_beta;
                    std::vector<double> traj_single_dim;
                    for (int j = 0; j < num_robot; j++) {
                        traj_single_dim.push_back(_tmp_gamma(j));
                    }
                    traj.push_back(traj_single_dim);
                }
                return true;
            }
            return false;
        }

    private:
        // Eigen::Matrix<int, NR, NG> Phi;
        int num_robot;
        int num_goal;
        int dim;
        int dyn;
        double v_max;
        double max_tf;
        std::unique_ptr<Eigen::MatrixXd> Phi;
        std::vector<Eigen::VectorXd> X_t0;
        std::vector<Eigen::VectorXd> G;
        std::vector<double> alpha;
};

#endif