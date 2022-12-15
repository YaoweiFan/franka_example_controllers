#include <iomanip>
#include <iostream>
#include <vector>
#include <nlopt.hpp>
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <fstream>

#define NUM 15
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// 待求解变量
// gravity
// handcm2ft_pos

// 获取的数据
struct Data {
    Eigen::Matrix3d transform;
    Vector6d ft_filtered;
};

Data data[NUM];

// Eigen::Vector3d ee2ft_pos{0, 0, -0.123};
Eigen::Vector3d ee2ft_pos{0, 0, -0.114};
// Eigen::Vector3d handcm2ft_pos{0.01, 0, -0.050};

Vector6d computeExternalFt(const Eigen::Matrix3d& transform, Vector6d& ft_filtered, double gravity, Eigen::Vector3d& handcm2ft_pos) {
    // 参数计算
    Eigen::Matrix<double, 3, 3> ee2ft_ori;
    ee2ft_ori << 0.7071, -0.7071, 0,
                0.7071,  0.7071, 0,
                    0,       0, 1;
    Eigen::Matrix<double, 3, 3> cee2ft_pos;
    cee2ft_pos << 0, -ee2ft_pos[2], ee2ft_pos[1],
                ee2ft_pos[2], 0, -ee2ft_pos[0],
                -ee2ft_pos[1], ee2ft_pos[0], 0;
    Eigen::Matrix<double, 6, 6> Adee2ft;
    Adee2ft.topLeftCorner(3,3) = ee2ft_ori;
    Adee2ft.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    Adee2ft.bottomLeftCorner(3,3) = cee2ft_pos * ee2ft_ori;
    Adee2ft.bottomRightCorner(3,3) = ee2ft_ori;

    Eigen::Matrix<double, 3, 3> handcm2ft_ori;
    handcm2ft_ori << 0.7071, -0.7071, 0,
                    0.7071,  0.7071, 0,
                        0,       0, 1;
    Eigen::Matrix<double, 3, 3> chandcm2ft_pos;
    chandcm2ft_pos << 0, -handcm2ft_pos[2], handcm2ft_pos[1],
                    handcm2ft_pos[2], 0, -handcm2ft_pos[0],
                    -handcm2ft_pos[1], handcm2ft_pos[0], 0;
    Eigen::Matrix<double, 6, 6> Adhandcm2ft;
    Adhandcm2ft.topLeftCorner(3,3) = handcm2ft_ori;
    Adhandcm2ft.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    Adhandcm2ft.bottomLeftCorner(3,3) = chandcm2ft_pos * handcm2ft_ori;
    Adhandcm2ft.bottomRightCorner(3,3) = handcm2ft_ori;

    // gripper重力对传感器读数的贡献
    Vector6d hand_gravity_handcm;
    Eigen::Matrix<double, 3, 3> base2handcm_ori = transform;
    hand_gravity_handcm.head(3) = base2handcm_ori.inverse() * Eigen::Vector3d{0, 0, 0}; // 力矩
    hand_gravity_handcm.tail(3) = base2handcm_ori.inverse() * Eigen::Vector3d{0, 0, gravity}; // 力
    Vector6d hand_gravity_contribution = Adhandcm2ft.transpose() * hand_gravity_handcm;
    // 外力对传感器读数的贡献
    Vector6d mf;
    mf.head(3) = ft_filtered.tail(3);
    mf.tail(3) = ft_filtered.head(3);
    Vector6d external_force_contribution = mf - hand_gravity_contribution;
    // 外力在末端坐标系下的表示
    Vector6d external_force_ee = Adee2ft.transpose().inverse() * external_force_contribution;
    // 外力在基坐标系下的表示
    Vector6d external_force_base;
    external_force_base.head(3) = transform * external_force_ee.tail(3);
    external_force_base.tail(3) = transform * external_force_ee.head(3);

    // 计算当前末端力与期望力之差
    return external_force_base;
}

double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    double gravity = x[0];
    Eigen::Vector3d handcm2ft_pos{x[1], x[2], x[3]};
    double res = 0;
    for(int i=0; i<NUM; ++i) {
        Vector6d err = computeExternalFt(data[i].transform, data[i].ft_filtered, gravity, handcm2ft_pos);
        res += err.norm();
    }
    // std::cout << res << std::endl;
    return res;
}

// double myvconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
// {
//     my_constraint_data *d = reinterpret_cast<my_constraint_data*>(data);
//     double a = d->a, b = d->b;
//     if (!grad.empty()) {
//         grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
//         grad[1] = -1.0;
//     }
//     return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
// }

int main(void) {
    // 载入数据
    std::ifstream infile;
    infile.open("/home/fyw/Documents/projects/panda_ros/franka_panda_control_ws/data/collect/record_panda_2.txt");
    std::vector<std::vector<double>> arr;
    std::string input;
    while(std::getline(infile, input)) {
        if(input.size()>0) {
            std::stringstream stringin(input);
            std::vector<double> v;
            double tmp;
            while(stringin >> tmp) v.push_back(tmp);
            arr.push_back(v);
        }
    }
    for(int i=0; i<NUM; ++i) {
        int start_line_num = 5 * i;
        data[i].transform << arr[start_line_num][0],   arr[start_line_num][1],   arr[start_line_num][2],
                             arr[start_line_num+1][0], arr[start_line_num+1][1], arr[start_line_num+1][2],
                             arr[start_line_num+2][0], arr[start_line_num+2][1], arr[start_line_num+2][2];
        // std::cout << data[i].transform << std::endl;
        data[i].ft_filtered << arr[start_line_num+4][0], arr[start_line_num+4][1], arr[start_line_num+4][2],
                               arr[start_line_num+4][3], arr[start_line_num+4][4], arr[start_line_num+4][5];
        // std::cout << data[i].ft_filtered << std::endl;
    }

    // 优化计算
    nlopt::opt opt(nlopt::LN_COBYLA, 4);
    // std::vector<double> lb(2);
    // lb[0] = -HUGE_VAL; lb[1] = 0;
    // opt.set_lower_bounds(lb);
    opt.set_min_objective(myfunc, NULL);
    // my_constraint_data data[2] = { {2,0}, {-1,1} };
    // opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
    // opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);
    opt.set_xtol_rel(1e-6);
    std::vector<double> x(4);
    x[0] = -7.85939; x[1] = 0.01; x[2] = 0;  x[3] = -0.0461069;
    double minf;

    try{
        nlopt::result result = opt.optimize(x, minf);
        std::cout << "found minimum at f(" << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] <<  ") = "
            << std::setprecision(10) << minf << std::endl;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
}

