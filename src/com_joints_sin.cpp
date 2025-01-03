#include <iostream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include <cstdlib>
#include <signal.h>

#include "panda_controllers/point.h"
#include "panda_controllers/desTrajEE.h"
#include "panda_controllers/flag.h"

// #include "utils/ThunderPanda.h"
#include "utils/utils_cartesian.h"

#define NJ 7

typedef Eigen::Vector3d vec3d;

using std::cout;
using std::cin;
using std::endl;

struct SinusoidParameters {
	int N;
	Eigen::VectorXd offsets;  // Amplitudes for each joint
    Eigen::VectorXd amplitudes;  // Amplitudes for each joint
    Eigen::VectorXd frequencies; // Frequencies for each joint
    Eigen::VectorXd phases;      // Phases for each joint
};

void computeTrajectory(const SinusoidParameters& params, double t, Eigen::VectorXd& positions, Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations);

void poseCallback(const panda_controllers::pointConstPtr& msg);
/* End-effector current position */
vec3d pose_EE;
vec3d pose_EE_start;

bool init_start = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "com_joints_sin_node");
	ros::NodeHandle node_handle;
    double frequency = 500;
	ros::Rate loop_rate(frequency);
    // double omega_sin_1 = M_PI/2;
    // double amp_1 = 0.30;
    // double omega_sin_2 = M_PI;
    // double amp_2 = 0.60;

    /* Publisher */
	ros::Publisher pub_des_jointState = node_handle.advertise<sensor_msgs::JointState>("/controller/command_joints", 1); 
    ros::Publisher pub_flag_joints = node_handle.advertise<panda_controllers::flag>("/controller/jointsFlag", 1);

    /* Subscriber */
	ros::Subscriber sub_pose = node_handle.subscribe<panda_controllers::point>("/controller/current_config", 1, &poseCallback);

    sensor_msgs::JointState command;
    panda_controllers::flag flag_joints;
    command.position.resize(NJ);
    command.velocity.resize(NJ);
    command.effort.resize(NJ);

    Eigen::VectorXd qr(NJ,1);
    Eigen::VectorXd dot_qr(NJ,1);
    Eigen::VectorXd ddot_qr(NJ,1);

    ros::Time t;
    double t_start;
    double dt = 0;
	/*Limiti del franka*/
	Eigen::VectorXd q_max_limit;
	Eigen::VectorXd q_min_limit;
	q_min_limit.resize(NJ);
	q_max_limit.resize(NJ);
	q_min_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
	q_max_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;	
	Eigen::VectorXd q_c = (q_min_limit + q_max_limit)/2;

	SinusoidParameters sin_par;
	sin_par.N = NJ;
	sin_par.offsets.resize(NJ);
	sin_par.amplitudes.resize(NJ);
    sin_par.frequencies.resize(NJ);
    sin_par.phases.resize(NJ);
	sin_par.offsets = q_c;
	sin_par.amplitudes << 1.2, 0.5, 1.2, 0.5, 1.2, 0.6, 1.2;
	sin_par.frequencies << 0.1, 0.15, 0.18, 0.2, 0.3, 0.09, 0.23;
	sin_par.phases << 0, 0, 0, 0, 0, 0, 0;

    t = ros::Time::now();
    t_start = t.toSec();
    while (ros::ok()){
        
        ros::spinOnce();
        t = ros::Time::now();
        if (dt == 0)
        {
            dt = t.toSec() - t_start;
            cout<<"tempo di inizio:"<<dt;
        }else
        {
            dt = t.toSec() - t_start;
        }
        
        // qr << amp_1*sin(1.5*(omega_sin_1)*dt), amp_1*sin(2*(omega_sin_1/2)*dt), amp_1*sin(2*(omega_sin_1/4)*dt), -1.5+2*amp_1*sin(2*(omega_sin_1/4)*dt), 0.0+amp_2*sin(2*(omega_sin_2/6)*dt), 1.5+amp_2*sin(2*(omega_sin_2/6)*dt), 0.0+amp_2*sin(2*(omega_sin_2/8)*dt);
        // dot_qr << 1.5*(omega_sin_1)*amp_1*cos(1.5*(omega_sin_1)*dt), 2*(omega_sin_1/2)*amp_1*cos(2*(omega_sin_1/2)*dt), 2*(omega_sin_1/4)*amp_1*cos(2*(omega_sin_1/4)*dt), 4*(omega_sin_1/4)*amp_1*cos(2*(omega_sin_1/4)*dt), 2*(omega_sin_2/6)*amp_2*cos(2*(omega_sin_2/6)*dt), 2*(omega_sin_2/6)*amp_2*cos(2*(omega_sin_2/6)*dt), 2*(omega_sin_2/8)*amp_2*cos(2*(omega_sin_2/8)*dt);
        // ddot_qr << -pow(1.5*(omega_sin_1),2)*amp_1*sin(1.5*(omega_sin_1)*dt), -pow(2*(omega_sin_1/2),2)*amp_1*sin(2*(omega_sin_1/2)*dt), +pow(2*(omega_sin_1/4),2)*amp_1*sin(2*(omega_sin_1/4)*dt), -pow(2*(omega_sin_1/4),2)*2*amp_1*sin(2*(omega_sin_1/4)*dt), -pow(2*(omega_sin_2/6),2)*amp_2*sin(2*(omega_sin_2/6)*dt), -pow(2*(omega_sin_2/6),2)*amp_2*sin(2*(omega_sin_2/6)*dt), -pow(2*(omega_sin_2/8),2)*amp_2*sin(2*(omega_sin_2/8)*dt);
        
		computeTrajectory(sin_par, dt, qr, dot_qr, ddot_qr);

        for(int i=0;i<NJ;i++){
            command.position[i] = qr(i);
            command.velocity[i] = dot_qr(i);
            command.effort[i] = ddot_qr(i);
        }
        flag_joints.flag = true;

        pub_flag_joints.publish(flag_joints);
        pub_des_jointState.publish(command);  
		loop_rate.sleep();

    }
    return 0;
}

void poseCallback(const panda_controllers::pointConstPtr& msg){

	double EE_x, EE_y, EE_z;
	
	EE_x = msg->xyz.x;
	EE_y = msg->xyz.y;
	EE_z = msg->xyz.z;

	pose_EE << EE_x, EE_y, EE_z;

	if (!init_start){
		pose_EE_start = pose_EE;
		init_start = true;
        std::cout<<"\n==============\n"<<"pose_EE_start:"<<"\n==============\n"<<pose_EE_start<<"\n==============\n";
	}
}

// Function to compute position, velocity, and acceleration
void computeTrajectory(const SinusoidParameters& params, double t, 
                       Eigen::VectorXd& positions, 
                       Eigen::VectorXd& velocities, 
                       Eigen::VectorXd& accelerations) 
{
    int n_joints = params.N;  // Number of joints
    
    // // Resize the output vectors
    // positions.resize(n_joints);
    // velocities.resize(n_joints);
    // accelerations.resize(n_joints);

    // Compute position, velocity, and acceleration for each joint
    for (int i = 0; i < n_joints; ++i) {
        double omega = 2 * M_PI * params.frequencies[i];  // Angular frequency
        positions[i] = params.offsets[i] + params.amplitudes[i] * std::sin(omega * t + params.phases[i]);
        velocities[i] = params.amplitudes[i] * omega * std::cos(omega * t + params.phases[i]);
        accelerations[i] = -params.amplitudes[i] * omega * omega * std::sin(omega * t + params.phases[i]);
    }
}