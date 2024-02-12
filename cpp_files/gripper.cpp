#include "kinematics_test.h"
#include "ros/ros.h"


const int X_axis = 0;
const int Y_axis = 1;
const int Z_axis = 2 ;


bool is_save_position(ros::Publisher p){
    return true;
}   // TODO FINISH



VectorXd get_robot_values(){        // TODO customize some names
    
    boost::shared_ptr<sensor_msgs::JointState const> mr;
    mr = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");

    
    VectorXd m(8);
    for (int i = 0; i < 8; ++i) m(i) = mr->position[i];
    
    VectorXd to_return(8);
    to_return << m(4), m(3), m(0), m(5), m(6), m(7), m(1), m(2);
    
    return to_return;
}



VectorXd get_joint_state(VectorXd mr){
    VectorXd joints(6);
    joints << mr(0), mr(1), mr(2), mr(3), mr(4), mr(5);
    return joints;
}

void error(){
    ROS_ERROR("[ERROR] Ops, position out of the workspace. Quitting");
    exit(1);
}

void validate_position(Vector3d pos){
    // check if pos is not beyond axis limits
    if (pos(X_axis) < -0.50 or pos(X_axis) > 0.5) error();
    else if (pos(Y_axis) < 0.10 or pos(Y_axis) > -0.4) error();
    if (pos(Z_axis) < 0. or pos(Z_axis) > 0.75) error();
    ROS_INFO("Position OK");
}

void translate_end_effector(Vector3d final_position, Matrix3d rotation, ros::Publisher p){       // TODO same of move_end_effector
    printf("BHO");
}


void move(MatrixX3d traj, ros::Publisher p){     // traj could be also Matrix<double, Eigen::Dynamic, 3>
    VectorXd joint_state(6);

    for(int i=0; i<traj.rows(); ++i){
        joint_state = get_joint_state(get_robot_values());
        Matrix4d transf_matrix = direct_kin(joint_state);
        translate_end_effector(traj.row(i), Matrix3d::Identity(), p);              // TODO function to send joint state new positions
    }
}



/*
VectorXd insert_new_path_instance(VectorXd p, V6d js, V2d gs)
{
    p.conservativeResize(p.rows() + 1, p.cols());
    p.row(p.rows() - 1) << js(0), js(1), js(2), js(3), js(4), js(5), gs(0), gs(1);
    return p;
}
*/

void open_gripper(ros::Publisher p){
     VectorXd gripper_trajectory(8);
    VectorXd mr(8);
    VectorXd joints(6);
    VectorXd gripper(2);

    mr = get_robot_values();
    joints = get_joint_state(mr);
    gripper << mr(6), mr(7);

    const int iterations = 50;
    const double opening_step = 0.80;
    const double gripper_r = gripper(0);
    const double gripper_l = gripper(1);

	
	printf("%f %f %f %f %f %f %f %f", mr(0), mr(1), mr(2), mr(3), mr(4), mr(5), mr(6), mr(7));
    // creating trajectory and moving clamps
    ros::Rate rate(120);

    for (int i=0; i<iterations; ++i){
        gripper(0) = gripper_r + i*(opening_step - gripper_r)/iterations;
        gripper(1) = gripper_l + i*(opening_step - gripper_l)/iterations;
        
        gripper_trajectory << joints(0), joints(1), joints(2), joints(3), joints(4), joints(5), gripper(0), gripper(1);
        
        // creating msg
        std_msgs::Float64MultiArray joint_msg;
        joint_msg.data.resize(8);
        for (int j = 0; j < 8; ++j) joint_msg.data[j]=gripper_trajectory(j);
        
        printf("msg: %f %f %f %f %f %f %f %f\n", joint_msg.data[0], 			joint_msg.data[1],joint_msg.data[2],joint_msg.data[3],joint_msg.data[4],joint_msg.data[5],joint_msg.data[6],joint_msg.data[7]);
        
        p.publish(joint_msg);
        rate.sleep();
    }
}

void close_gripper(ros::Publisher p){
    VectorXd gripper_trajectory(8);
    VectorXd mr(8);
    VectorXd joints(6);
    VectorXd gripper(2);

    mr = get_robot_values();
    joints = get_joint_state(mr);
    gripper << mr(6), mr(7);

    const int iterations = 50;
    const double opening_step = -0.50;
    const double gripper_r = gripper(0);
    const double gripper_l = gripper(1);

    // creating trajectory and moving clamps
    ros::Rate rate(120);

    for (int i=0; i<iterations; ++i){
        gripper(0) = gripper_r + i*(opening_step - gripper_r)/iterations;
        gripper(1) = gripper_l + i*(opening_step - gripper_l)/iterations;
        
        gripper_trajectory << joints(0), joints(1), joints(2), joints(3), joints(4), joints(5), gripper(0), gripper(1);
        
        // creating msg
        std_msgs::Float64MultiArray joint_msg;
        joint_msg.data.resize(8);
        for (int j = 0; j < 8; ++j) joint_msg.data[j]=gripper_trajectory(j);
        
        
        p.publish(joint_msg);
        rate.sleep();
    }
}

MatrixX3d get_trajectory(Vector3d b){
 return Matrix3d::Identity();
}


void grasping_operation(Vector3d block_coords, Matrix3d block_pose, Vector3d final_coords, Matrix3d final_pose, ros::Publisher publisher){
    
    // TODO cambia nomi qui -> potrebbe servire scegliere un altezza a cui prendere e mollare il blocco se quella passata da vision non va bene
    const double motion_height = 0.50;


    // set robot arm in base config
    is_save_position(publisher);      

    Vector3d above_block_coords;    // position exactly above the block at motion_height
    above_block_coords << block_coords(X_axis), block_coords(Y_axis), motion_height;


    // CHECKING positions
    ROS_INFO("Validating grasping position...");
    validate_position(above_block_coords);
    ROS_INFO("Validating target position...");
    validate_position(final_coords);


    MatrixX3d trajectory = get_trajectory(above_block_coords);       // TODO funzione per calcolare la traiettoria (restituisce matrice che ha per righe le posizioni dell'end effector)
    
    // move end effector just above the block
    move(trajectory, publisher);

    open_gripper(publisher);

    // lower end effector
    translate_end_effector(block_coords, block_pose, publisher);

    close_gripper(publisher);

    // move upwards
    translate_end_effector(above_block_coords, block_pose, publisher);

    Vector3d above_final_coords;    // position exactly above the final position
    above_final_coords << final_coords(X_axis), final_coords(Y_axis), motion_height;


    // trajectory to final position
    trajectory = get_trajectory(above_final_coords);
    
    move(trajectory, publisher);

    // lower end effector
    translate_end_effector(final_coords, final_pose, publisher);

    open_gripper(publisher);

    // move upwards
    translate_end_effector(above_block_coords, Matrix3d::Identity(), publisher);

}



int main(int argc, char** argv){

	ros::init(argc, argv, "publisher");
	ros::NodeHandle handler;
	ros::Publisher pub = handler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command",1);
	
	

	printf("Gripper: Ci sono bro!\n");
	
	
	open_gripper(pub);
	printf("verto\n");
	close_gripper(pub);
	printf("chiuso\n");
	
	
	return 0;
}
