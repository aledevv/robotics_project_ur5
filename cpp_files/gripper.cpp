#include "kinematics.h"
#include <Eigen>


const int X_axis = 0;
const int Y_axis = 1;
const int Z_axis = 2 ;


bool is_save_position(ros::Publisher p);   // TODO FINISH


VectorXf get_robot_values(){        // TODO customize some names
    
    boost::shared_ptr<sensor_msgs::JointState const> mr;
    mr = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");

    
    VectorXf m(8);
    for (int i = 0; i < 8; ++i) m(i) = mr->position[i];
    return VectorXf {m(4), m(3), m(0), m(5), m(6), m(7), m(1), m(2)};
}

VectorXf get_joint_state(VectorXf mr){
    VectorXf joints(6);
    joints << mr(0), mr(1), mr(2), mr(3), mr(4), mr(5);
    return joints;
}

void error(){
    ROS_ERROR("[ERROR] Ops, position out of the workspace. Quitting");
    exit(1);
}

void validate_position(Vector3f pos){
    // check if pos is not beyond axis limits
    if (pos(X_axis) < -0.50 or pos(X_axis) > 0.5) error();
    else if (pos(Y_axis) < 0.10 or pos(Y_axis) > -0.4) error();
    if (pos(Z_axis) < 0. or pos(Z_axis) > 0.75) error();
    ROS_INFO("Position OK");
}

void translate_end_effector(Vector3f final_position, Matrix3f rotation, ros::Publisher p){       // TODO same of move_end_effector
    ...
}

void move(MatrixXf traj, ros::Publisher p){     // traj could be also Matrix<double, Eigen::Dynamic, 3>
    VectorXf joint_state(6);

    for(int i=0; i<traj.rows(); ++i){
        joint_state = get_joint_state();
        Matrix4f transf_matrix = directKin(joint_state);
        translate_end_effector(traj.row(i), Matrix3f::Identity(), p);              // TODO function to send joint state new positions
    }
}


void open_gripper(int a, ros::Publisher p){
    VectorXf gripper_trajectory(8);
    VectorXf mr(8);
    VectorXf joints(6);
    VectorXf gripper(2);

    mr = get_robot_values();
    joints = get_joint_state(mr);
    gripper << mr(6), mr(7);

    const int iterations = 50;
    const double opening_step = 0.80;
    const double gripper_r = gripper(0);
    const double gripper_l = gripper(1);

    // creating trajectory and moving clamps
    ros::Rate rate(120);

    for (int i=0; i<iterations; ++i){
        gripper(0) = gripper_r + i*(opening_step - opening_step)/iterations;
        gripper(1) = gripper_l + i*(opening_step - opening_step)/iterations;
        
        gripper_trajectory << gripper(0), gripper(1), gripper(2), gripper(3), gripper(4), gripper(5), gripper(0), gripper(1);
        
        // creating msg
        std_msgs::Float64MultiArray joint_msg;
        joint_msg.data.resize(8);
        for (int j = 0; j < 8; ++j) joint_msg.data[j]=gripper(j);
        
        p.publish(joint_msg);
        rate.sleep();
    }

}

void close_gripper(int a, ros::Publisher p){
    
}


void grasping_operation(Vector3f block_coords, Matrix3f block_pose, Vector3f final_coords, Matrix3f final_pose, ros::Publisher publisher){
    
    // TODO cambia nomi qui -> potrebbe servire scegliere un altezza a cui prendere e mollare il blocco se quella passata da vision non va bene
    const double motion_height = 0.50;


    // set robot arm in base config
    is_save_position(publisher);      

    Vector3f above_block_coords;    // position exactly above the block at motion_height
    above_block_coords << block_coords(X_axis), block_coords(Y_axis), motion_height;


    // CHECKING positions
    ROS_INFO("Validating grasping position...");
    validate_position(above_block_coords);
    ROS_INFO("Validating target position...");
    validate_position(final_coords);


    MatrixXf trajectory = get_trajectory(above_block_coords)       // TODO funzione per calcolare la traiettoria (restituisce matrice che ha per righe le posizioni dell'end effector)
    
    // move end effector just above the block
    move(trajectory, publisher);

    open_gripper(publisher);

    // lower end effector
    translate_end_effector(block_coords, block_pose, publisher);

    close_gripper(publisher);

    // move upwards
    translate_end_effector(above_block_coords, block_pose, publisher);

    Vector3f above_final_coords;    // position exactly above the final position
    above_final_coords << final_coords(X_axis), final_coords(Y_axis), motion_height;


    // trajectory to final position
    trajectory = get_trajectory(above_final_coords);
    
    move(trajectory, publisher);

    // lower end effector
    translate_end_effector(final_coords, final_pose, publisher);

    open_gripper(publisher);

    // move upwards
    translate_end_effector(above_block_coords, Matrix3f::Identity(), publisher);

}

