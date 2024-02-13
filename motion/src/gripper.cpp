#include "kinematics_test.h"
#include "ros/ros.h"
#include <iostream>
#include "robotics_project_ur5/GetBrickPose.h"


const int X_axis = 0;
const int Y_axis = 1;
const int Z_axis = 2 ;
using namespace std;

double dt = 0.1;

bool is_save_position(ros::Publisher p){
    return true;
}   // TODO FINISH

/*
@ brief The function performs linear interpolation between two 3D vectors given a time parameter
@ param t The time parameter used for interpolation
@ param x1 The starting 3D vector.
@ param x2 The ending 3D vector.
@ param n_t Normalized time parameter calculated as t / d_path.
@ return The interpolated 3D vector at the specified time.
*/ 
V3d x(double t, V3d x1, V3d x2)
{
    const double n_t = t / d_path;
    if (n_t > 1) return x2;
    else return (n_t * x2) + ((1 - n_t) * x1);
}

/*
@ brief Inserts a new path instance into an existing path matrix
@ param p The existing path matrix to which a new instance will be added.
@ param js The 6D vector representing joint states.
@ param gs The 2D vector representing goal positions.
*/ 
Path insert_new_path_instance(Path p, V6d js, V2d gs)
{
    p.conservativeResize(p.rows() + 1, p.cols());
    p.row(p.rows() - 1) << js(0), js(1), js(2), js(3), js(4), js(5), gs(0), gs(1);
    ROS_INFO("insert_new_path done\n");
    return p;
}

V8d get_robot_values(){        // TODO customize some names
    
    boost::shared_ptr<sensor_msgs::JointState const> mr;
    mr = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
    
    V8d m;
    for (int i = 0; i < 8; i++) m(i) = mr->position[i];
    
    V8d to_return(8);
    to_return << m(4), m(3), m(0), m(5), m(6), m(7), m(1), m(2);
    
    return to_return;
}

/*
@ brief Extracts a 6D vector of joint states from an 8D vector.
@ param mr The 8D vector containing information, where the first six elements represent joint states.
@ return A 6D vector representing joint states extracted from the input vector.
*/ 
V6d get_joint_state(V8d mr){
    V6d joints(6);
    joints << mr(0), mr(1), mr(2), mr(3), mr(4), mr(5);
    return joints;
}

void error(){
    ROS_ERROR("[ERROR] Ops, position out of the workspace. Quitting");
    exit(1);
}

/*
@ brief Validates a 3D position vector against specified axis limits.
*/ 
void validate_position(Vector3d pos){
    // check if pos is not beyond axis limits
    if (pos(X_axis) < -0.50 or pos(X_axis) > 0.5) error();
    else if (pos(Y_axis) > 0.10 or pos(Y_axis) < -0.4) error();
    if (pos(Z_axis) < 0. or pos(Z_axis) > 0.75) error();
    ROS_INFO("Position OK");
}

/*
@ brief e spieghi in breve
@ param e spieghi e paraemtri se serve
@ retval valore di ritonro
*/ 
void move(Path mv, ros::Publisher pub)
{
    
    ros::Rate loop_rate(120);

    std::cout << mv;

    for (int i = 0; i < mv.rows(); i++)
    {
        
        V8d joint_state;
        joint_state << mv(i, 0), mv(i, 1), mv(i, 2), mv(i, 3), mv(i, 4), mv(i, 5), mv(i, 6), mv(i, 7);
        
        std_msgs::Float64MultiArray joint_statem;
        joint_statem.data.resize(8);
        for (int j = 0; j < 8; j++) joint_statem.data[j] = joint_state(j);

        
        pub.publish(joint_statem);
        loop_rate.sleep();
    }
}

/*
@ brief e spieghi in breve
@ param e spieghi e paraemtri se serve
@ retval valore di ritonro
*/ 
Path differential_inverse_kin_quaternions(V8d mr, V3d i_p, V3d f_p, Qd i_q, Qd f_q)
{
    V2d gs {mr(6), mr(7)};
    V6d js_k, js_dot_k; 
    V6d fv;
    Path path;
    M4d tm_k;
    V3d p_k;
    M3d rm_k;
    Qd q_k;
    V3d av_k, pv_k;
    Qd qv_k;
    Qd qerr_k;
    V3d perr_k;
    Jacobian j_k, invj_k;

    M3d Kp, Kq;
    Kp = M3d::Identity() * 10;
    Kq = M3d::Identity() * 1;

  

    for (int i = 0; i < 6; ++i) js_k(i) = mr(i);
    //js_k(5) = 3.49;
    for(int i=0;i<6;i++){
        std::cout << "js_k: " << js_k(i) << std::endl; 
    }
    path = insert_new_path_instance(path, js_k, gs);
    ROS_INFO("First path ok\n");

   
    for (double t = dt; t < d_path; t += dt) 
    {
        
        tm_k = mwtb() * direct_kin(js_k) * gripper_frame();
        p_k = tm_k.block(0, 3, 3, 1);
        rm_k = tm_k.block(0, 0, 3, 3);
        q_k = rm_k;

       
        pv_k = (x(t, i_p, f_p) - x(t - dt, i_p, f_p)) / dt;
        qv_k = slerp(t + dt, i_q, f_q) * slerp(t, i_q, f_q).conjugate(); 
	    av_k = (qv_k.vec() * 2) / dt;

       printf("QUI CON NOI\n");
       for(int i=0;i<6;i++){
        std::cout << "js_k2: " << js_k(i) << std::endl; 
    }
        j_k = jacobian(js_k);
        ROS_INFO("Jacobian passed\n");
        
        invj_k = j_k.transpose() * (j_k * j_k.transpose()  + Jacobian::Identity() * 0.0001).inverse();
        if (abs(j_k.determinant()) < 0.00001) 
        {
            ROS_WARN("Near singular configuration");
        }

        
      
        qerr_k = slerp(t, i_q, f_q) * q_k.conjugate();
        perr_k = x(t, i_p, f_p) - p_k;
        
       
        fv << pv_k + (Kp * perr_k), av_k + (Kq * qerr_k.vec());

       
        js_dot_k = invj_k * fv;
        js_k = js_k + (js_dot_k * dt);

       
        path = insert_new_path_instance(path, js_k, gs);
    }

    return path;
}


/*
@ brief e spieghi in breve
@ param e spieghi e paraemtri se serve
@ retval valore di ritonro
*/ 
void translate_end_effector(Vector3d final_position, Matrix3d rotation, ros::Publisher pub){       // TODO same of move_end_effector
   
    Qd final_quaternion(rotation);

  
    V8d robot_measures = get_robot_values();
    V6d joint_state = get_joint_state(robot_measures);

   
    M4d transformation_matrix = mwtb() * direct_kin(joint_state) * gripper_frame();
    M3d rotation_matrix = transformation_matrix.block(0, 0, 3, 3);
    V3d position = transformation_matrix.block(0, 3, 3, 1);
    Qd init_quaternion(rotation_matrix);

    
    Path p = differential_inverse_kin_quaternions(robot_measures, position, final_position, init_quaternion, final_quaternion);
    
    
    move(p, pub);
}

/*
@ brief e spieghi in breve
@ param e spieghi e paraemtri se serve
@ retval valore di ritonro
*/ 
void move2(MatrixX3d traj, ros::Publisher p){     // traj could be also Matrix<double, Eigen::Dynamic, 3>
    VectorXd joint_state(6);

    for(int i=0; i<traj.rows(); i++){
        
        joint_state = get_joint_state(get_robot_values());
        Matrix4d transf_matrix = mwtb() * direct_kin(joint_state) * gripper_frame();
         
        translate_end_effector(traj.row(i), Matrix3d::Identity(), p);              // TODO function to send joint state new positions
    }
}

/*
@ brief e spieghi in breve
@ param e spieghi e paraemtri se serve
@ retval valore di ritonro
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

    // creating trajectory and moving clamps
    ros::Rate rate(120);

    for (int i=0; i<iterations; i++){
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

/*
@ brief e spieghi in breve
@ param e spieghi e paraemtri se serve
@ retval valore di ritonro
*/ 
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

    for (int i=0; i<iterations; i++){
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


/*
@ brief e spieghi in breve
@ param e spieghi e paraemtri se serve
@ retval valore di ritonro
*/ 
MatrixX3d get_trajectory(Vector3d final_position){  
  
    int stationary_points_num = 7;
    Matrix<double, 7, 3> stationary_points;
    stationary_points << 0.3, 0.1, 0.5, 
                        0.4, 0, 0.5,
                        0.3, -0.3, 0.5,
                        0, -0.4, 0.5,
                        -0.3, -0.3, 0.5, 
                        -0.4, 0, 0.5,
                        -0.3, 0.1, 0.5;
        

 
    V6d joint_state = get_joint_state(get_robot_values());
    M4d transformation_matrix = mwtb() * direct_kin(joint_state) * gripper_frame();
    V3d init_position = transformation_matrix.block(0, 3, 3, 1);

   
    int starting_position = -1;
    double min_distance = -1;
    double possible_min_distance;
    for (int i = 0; i < stationary_points_num; i++)
    {
        possible_min_distance = abs(stationary_points(i, 0) - init_position(0));
        possible_min_distance = possible_min_distance + abs(stationary_points(i, 1) - init_position(1));
        if (min_distance == -1 || possible_min_distance < min_distance) 
        {
            min_distance = possible_min_distance; 
            starting_position = i;
        }
    }

    
    int ending_position = -1;
    min_distance = -1;
    for (int i = 0; i < stationary_points_num; i++)
    {
        possible_min_distance = abs(stationary_points(i, 0) - final_position(0));
        possible_min_distance = possible_min_distance + abs(stationary_points(i, 1) - final_position(1));
        if (min_distance == -1 || possible_min_distance < min_distance)
        {
            min_distance = possible_min_distance; 
            ending_position = i;
        }
    }

   
    bool trajectory_built = false;
    int i = starting_position;

    MatrixX3d trajectory;

    do
    {
        trajectory.conservativeResize(trajectory.rows() + 1, trajectory.cols());
        trajectory.row(trajectory.rows() - 1) = V3d {stationary_points(i, 0), stationary_points(i, 1), stationary_points(i, 2)};
        if (i == ending_position) trajectory_built = true;
        if (ending_position < starting_position) i = i - 1; else i = i + 1;
    }
    while (!trajectory_built);

    trajectory.conservativeResize(trajectory.rows() + 1, trajectory.cols());
    trajectory.row(trajectory.rows() - 1) = V3d {final_position(0), final_position(1), final_position(2)};

    return trajectory;

}

/*
@ brief e spieghi in breve
@ param e spieghi e paraemtri se serve
@ retval valore di ritonro
*/ 
void grasping_operation(Vector3d block_coords, Matrix3d block_pose, Vector3d final_coords, Matrix3d final_pose, ros::Publisher publisher){
    
    // TODO cambia nomi qui -> potrebbe servire scegliere un altezza a cui prendere e mollare il blocco se quella passata da vision non va bene
    const double motion_height = 0.50;


    // set robot arm in base config
    is_save_position(publisher);      



    Vector3d above_block_coords;    // position exactly above the block at motion_height
    above_block_coords << block_coords(X_axis), block_coords(Y_axis), motion_height;


    // CHECKING positions
    ROS_INFO("Validating grasping position...\n");
   // validate_position(above_block_coords);
    ROS_INFO("Validating target position...\n");
    //validate_position(final_coords);


    
    MatrixX3d trajectory = get_trajectory(above_block_coords);       // TODO funzione per calcolare la traiettoria (restituisce matrice che ha per righe le posizioni dell'end effector)
    

    // move end effector just above the block
    move2(trajectory, publisher);


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
    
    move2(trajectory, publisher);

    // lower end effector
    translate_end_effector(final_coords, final_pose, publisher);

    open_gripper(publisher);

    // move upwards
    translate_end_effector(above_block_coords, Matrix3d::Identity(), publisher);

}



int main(int argc, char** argv){

	ros::init(argc, argv, "ur5_joint_position_publisher");
	ros::NodeHandle node_handler;
	ros::Publisher pub = node_handler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command",1);
	ros::ServiceClient service_client = node_handler.serviceClient<robotics_project_ur5::GetBrickPose>("GetBrickPose");
    robotics_project_ur5::GetBrickPose srv;
	

    if (service_client.call(srv))
     {  
         for (int i = 0; i < srv.response.numBricks; ++i)
        { 
            printf("Gripper: Ci sono bro!\n");
	
            Vector3d block;
            //block << 0.3151, 0.5602, 0.8699;
            block << srv.response.pose[i].position.x, srv.response.pose[i].position.y, srv.response.pose[i].position.z;

	         //M3d pose = rotation_matrix_z_axis(2.489469);
            M3d pose = rotation_matrix_z_axis(srv.response.pose[i].orientation.z);

            Vector3d final_pos;
            final_pos << 0.5, 0.5, 0.8899;

            M3d final_pose = rotation_matrix_z_axis(0);

            printf("Inizio grasping...\n");
	        //grasping_operation(block, pose, final_pos, final_pose, pub);
            grasping_operation(block, pose, final_pos, final_pose, pub);
            // ROS_INFO("pose: %ld, %ld, %ld, %ld", srv.response.pose[0].position.x, srv.response.pose[0].position.y, srv.response.pose[0].position.z, srv.response.pose[0].orientation.z);
            // ROS_INFO("length: %ld", srv.response.numBricks[0]);
            // ROS_INFO("label: %s", srv.response.label[0]);
            std::cout << "pose x brick " << i + 1 << ":\n" << srv.response.pose[i].position.x << std::endl;
            std::cout << "pose y brick " << i + 1 << ":\n" << srv.response.pose[i].position.y << std::endl;
            std::cout << "pose z brick " << i + 1 << ":\n" << srv.response.pose[i].position.z << std::endl;
            std::cout << "orientation z brick " << i + 1 << ":\n" << srv.response.pose[i].orientation.z << std::endl;
        }
        
        ROS_INFO("COMPLETED");
     }

    ros::spin();
	
	
	return 0;
}
