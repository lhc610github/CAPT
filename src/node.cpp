#include "Hungarian.h"
#include "capt.hpp"
#include <ros/ros.h>
#include <Eigen/Dense>

// #define NR = 10;
// #define NG = 10;
// #define D = 3;

typedef Eigen::Vector3d Pos;
typedef Eigen::Vector3d Vel;

bool cal_hungarian(const std::vector<Pos>& _init, const std::vector<Pos>& _goal, std::vector<int>& _assgin) {
    const int _num_robot = _init.size();
    const int _num_goal = _goal.size();
    if (_num_robot <= 0 || _num_goal <= 0 || _num_goal > _num_robot) {
        return false;
    }

    std::vector<std::vector<double>> _tmp_cost;
    for (int i = 0; i < _num_robot; i++) {
        std::vector<double> _tmp_robot_cost;
        for (int j = 0; j < _num_goal; j++) {
            double _cost = (_goal[j] - _init[i]).norm();
            _tmp_robot_cost.push_back(_cost);
        }
        _tmp_cost.push_back(_tmp_robot_cost);
    }

	HungarianAlgorithm HungAlgo;
	std::vector<int> assignment;

	double cost = HungAlgo.Solve(_tmp_cost, assignment);

    std::cout << "====================== Hungarian Result =======================" << std::endl;
	for (unsigned int i = 0; i <_tmp_cost.size(); i++) {
        if (assignment[i] != -1 ) {
            std::cout << "(" << _init[i].transpose() << ") -> (" << _goal[assignment[i]].transpose() << ")" << std::endl;
        }
    }
    _assgin = assignment;
	std::cout << "cost: " << cost << std::endl;
    std::cout << "===============================================================" << std::endl;
    return true;
}

int main(int argc, char** argv) {
    std::vector<Pos> _swarm_pos;
    std::vector<Pos> _swarm_goal;
    std::vector<std::vector<double>> _swarm_pos_vect;
    std::vector<std::vector<double>> _swarm_goal_vect;
    int _num_robot = 10;
    Pos _robot_p;
    _robot_p(0) = 0;
    _robot_p(1) = 0;
    _robot_p(2) = 0;
    for (int i = 0; i < _num_robot; i++) {
        std::vector<double> _tmp_pos_vect;
        _tmp_pos_vect.push_back(_robot_p(0));
        _tmp_pos_vect.push_back(_robot_p(1));
        _tmp_pos_vect.push_back(_robot_p(2));
        _swarm_pos_vect.push_back(_tmp_pos_vect);
        _swarm_pos.push_back(_robot_p);
        _robot_p(0) += 1.0;
        _robot_p(1) -= 1.0;
        _robot_p(2) += 0.2;
    }

    int _num_goal = 10;
    Pos _goal;
    _goal << -3.0, 3.0 , 3.0;
    for (int i = 0; i < _num_goal; i++) {
        std::vector<double> _tmp_goal_vect;
        _tmp_goal_vect.push_back(_goal(0));
        _tmp_goal_vect.push_back(_goal(1));
        _tmp_goal_vect.push_back(_goal(2));
        _swarm_goal_vect.push_back(_tmp_goal_vect);
        _swarm_goal.push_back(_goal);
        _goal(0) -= 1.0;
        _goal(1) += 3.0;
        _goal(2) -= 0.3;
    }
    
    // _swarm_goal.push_back(_goal);
    // _goal << -2.0, 2.0 , 0.0;
    // _swarm_goal.push_back(_goal);

	std::vector<int> _assign;
    cal_hungarian(_swarm_pos, _swarm_goal, _assign);

    CAPT _capt(_num_robot, _num_goal, 3);
    _capt.set_Phi(_assign);
    _capt.set_init_goal(_swarm_pos_vect, _swarm_goal_vect);
    _capt.cal_beta();
    double _max_tf = _capt.get_tf();
    for (double _t = 0.1f; _t <= _max_tf; _t += 0.1f) {
        std::vector<std::vector<double>> _traj;
        if (_capt.cal_traj(_t,_traj)) {
            printf("--------------------------------------------------%.2f s ----------------------------------------\n", _t);
            for (int i = 0; i < 3; i ++) {
                for (int j = 0; j < 10; j++) {
                    printf("%4.2f\t", _traj[i][j]);
                }
                printf("\n");
            }
        }
        usleep(100000);
    }
}