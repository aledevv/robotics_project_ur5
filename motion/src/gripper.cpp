#include "kinematics_test.h"
#include "ros/ros.h"
#include <iostream>
#include "robotics_project_ur5/GetBrickPose.h"


const int X_axis = 0;
const int Y_axis = 1;
const int Z_axis = 2 ;
using namespace std;

double dt = 0.1;
V6d start_config;
start_config << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;

void translate_end_effector(V3d final_position, M3d rotation, ros::Publisher pub);
void set_start_position(ros::Publisher pub);
Path insert_new_path(Path p, V6d js, V2d gripperState);
V8d get_robot_values();
V6d get_joint_state(V8d realMeasures);
void error();
void validate_position(V3d pos);
void move(Path mv, ros::Publisher pub);
Path differential_inverse_kin_quaternions(V8d realMeasures, V3d initPos, V3d finalPos, Qd initQuat, Qd finalQuat);
void move2(MatrixX3d traj, ros::Publisher p);
void open_gripper(ros::Publisher p);
void close_gripper(ros::Publisher p);
Matrix<double, 6, 4>  get_trajectory(double t_init, double t_final, V6d joints_init, V6d joints_final, double speed);
void grasping_operation(Vector3d block_coords, Matrix3d block_pose, Vector3d final_coords, Matrix3d final_pose, ros::Publisher publisher);


//from robot joint state, finds end effector position and rotation matrix, uses them to compute
//new path (new jointvalues), calls move that gets js and publishes them with a rate -> who uses them?
void translate_end_effector(V3d final_position, M3d rotation, ros::Publisher pub){       // TODO same of move_end_effector
   
    Qd final_quaternion(rotation);

  
    V8d robot_measures = get_robot_values();
    V6d joint_state = get_joint_state(robot_measures);
   
    M4d transformation_matrix = mwtb() * direct_kin(joint_state) * gripper_frame();
    M3d rotation_matrix = transformation_matrix.block(0, 0, 3, 3);
    V3d position = transformation_matrix.block(0, 3, 3, 1);
    Qd init_quaternion(rotation_matrix);

    Path p = differential_inverse_kin_quaternions(robot_measures, position, final_position, init_quaternion, final_quaternion);
    
    (p, pub);
}


void set_start_position(ros::Publisher pub){
    M4d transformation_matrix = mwtb() * direct_kin(start_config) * gripper_frame();
    M3d rotation_matrix = transformation_matrix.block(0, 0, 3, 3);
    V3d position = transformation_matrix.block(0, 3, 3, 1);

    translate_end_effector(position, rotation_matrix, pub);
}   // TODO FINISH


// V3d linearInterpolation(double t, V3d x1, V3d x2)
// {
//     const double n_t = t / d_path;
//     if (n_t > 1) return x2;
//     else return ((n_t * x2) + ((1 - n_t) * x1));
// }
// V3d linearInterpolation(double t, const V3d& x1, const V3d& x2) {
//     const double n_t = t / d_path;
//     return (n_t > 1.0) ? x2 : (n_t * x2 + (1.0 - n_t) * x1);
// }

Path insert_new_path(Path p, V6d js, V2d gripperState)
{
    p.conservativeResize(p.rows() + 1, p.cols());
    p.row(p.rows() - 1) << js(0), js(1), js(2), js(3), js(4), js(5), gripperState(0), gripperState(1);
    ROS_INFO("insert_new_path done\n");
    return p;
}

V8d get_robot_values(){        
    
    boost::shared_ptr<sensor_msgs::JointState const> realMeasures;
    realMeasures = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
    
    V8d m;
    for (int i = 0; i < 8; i++) 
        m(i) = realMeasures->position[i];
    
    V8d to_return(8);
    to_return << m(4), m(3), m(0), m(5), m(6), m(7), m(1), m(2);        //ur5 joint order
    
    return to_return;
}


V6d get_joint_state(V8d realMeasures){
    V6d joints(6);
    joints << realMeasures(0), realMeasures(1), realMeasures(2), realMeasures(3), realMeasures(4), realMeasures(5);
    return joints;
}

void error(){
    ROS_ERROR("[ERROR] Ops, position out of the workspace. Quitting");
    exit(1);
}

void validate_position(V3d pos){
    // check if pos is not beyond axis limits
    if (pos(X_axis) < -0.50 or pos(X_axis) > 0.5) error();
    else if (pos(Y_axis) > 0.10 or pos(Y_axis) < -0.4) error();
    if (pos(Z_axis) < 0. or pos(Z_axis) > 0.75) error();
    ROS_INFO("Position OK");
}

//takes next joint pos in path and sends them every 120 hz
void move(Path mv, ros::Publisher pub)
{
    ros::Rate loop_rate(120);
    std::cout << mv;

    for (int i = 0; i < mv.rows(); i++){
        
        //takes joint state to reach stored in the i-th row of path matrix
        V8d joint_state;
        joint_state << mv(i, 0), mv(i, 1), mv(i, 2), mv(i, 3), mv(i, 4), mv(i, 5), mv(i, 6), mv(i, 7);
        
        //joint state to reach, sent with publisher for robot to get
        std_msgs::Float64MultiArray joint_statem;
        joint_statem.data.resize(8);
        for (int j = 0; j < 8; j++) 
            joint_statem.data[j] = joint_state(j);

        pub.publish(joint_statem);
        loop_rate.sleep();
    }
}

Path differential_inverse_kin_quaternions(V8d realMeasures, V3d initPos, V3d finalPos, Qd initQuat, Qd finalQuat)
{
    V2d gripperState {realMeasures(6), realMeasures(7)};
    V6d jointState_k, jsDer_k; 
    V6d correctedPOvel;
    Path path;
    M4d transMat_k;
    V3d pe_k;
    M3d rotMat_k;
    Qd quat_k;
    V3d angVel_k, posVel_k;
    Qd quatVel_k;
    Qd qerr_k;
    V3d perr_k;
    Jacobian jac_k, invjac_k, pseudoInvjac_k;

    M3d Kp, Kq;
    Kp = M3d::Identity() * 10;
    Kq = M3d::Identity() * 1;
  

    for (int i = 0; i < 6; ++i) jointState_k(i) = realMeasures(i);
    //jointState_k(5) = 3.49;
    for(int i=0;i<6;i++){
        std::cout << "jointState_k: " << jointState_k(i) << std::endl; 
    }
    path = insert_new_path(path, jointState_k, gripperState);  //insert initial values
    ROS_INFO("First path ok\n");

   
    for (double t = dt; t < d_path; t += dt)    //procedi di dt in dt per trovare d_path/dt joint configurations
    {
        //get actual end effector position and rotation matrix from world frame
        transMat_k = mwtb() * direct_kin(jointState_k) * gripper_frame();
        pe_k = transMat_k.block(0, 3, 3, 1);    //end effector position
        rotMat_k = transMat_k.block(0, 0, 3, 3);    //rotation matrix
        //quat_k = Quaternion(rotMat_k);  //quaternion si rotation matrix  ???works???
        quat_k = rotMat_k;
       
       //compute end eff velocity at instant k and angular velocity from quaternion
        posVel_k = ((linearInterpolation(t, initPos, finalPos)) - (linearInterpolation(t - dt, initPos, finalPos))) / dt;
        //quaternione (rot matrix) at t=t+dt * quaternione a tempo t ("initiale") coniugato
        quatVel_k = slerp(t + dt, initQuat, finalQuat) * slerp(t, initQuat, finalQuat).conjugate(); 
	    angVel_k = (quatVel_k.vec() * 2) / dt;  //from relation between quaternion and angular velocity (rotation around w axe)

        printf("QUI CON NOI\n");
            for(int i=0;i<6;i++){
            std::cout << "jointState_k2: " << jointState_k(i) << std::endl; 
        }

        //compute jacobian
        jac_k = jacobian(jointState_k);
        ROS_INFO("Jacobian passed\n");

        //compute pseudo inverse jac with dumped least square value of 0.0001
        //pseudoInvjac_k = jac_k.block(0,0,6,3).transpose()*((jac_k.block(0,0,6,3) * jac_k.block(0,0,6,3).transpose())).inverse();
        pseudoInvjac_k = jac_k.transpose() * (jac_k * jac_k.transpose()  + Jacobian::Identity() * 0.0001).inverse();    //dumped least squaare value
        if (abs(jac_k.determinant()) < 0.00001){
            ROS_WARN("Near singular configuration");
        }

        //compute orientation error (qd * qe*) and pos error (pd - pe)
        //orientation error = qd * qe* -> slerp desired * rotMat given actual joint state
        qerr_k = slerp(t, initQuat, finalQuat) * quat_k.conjugate();    
        //position error = linear interpolation - effective pos
        perr_k = linearInterpolation(t, initPos, finalPos) - pe_k;
        
       //compute corrected pos and ang vel with errors and K factors
        correctedPOvel << posVel_k + (Kp * perr_k), angVel_k + (Kq * qerr_k.vec());

       //compute joint velocities at instant k from end effector velocities at k
        //jsDer_k = invjac_k * correctedPOvel;
        jsDer_k = pseudoInvjac_k * correctedPOvel;//.transpose() + (Jacobian::Identity().block(0,0,6,3) - pseudoInvjac_k.block(0,0,6,3) * jac_k.block(0,0,6,3)) * quatVel_k;
        //upgrade joint position at k: previous one + new one
        jointState_k = jointState_k +  jsDer_k * dt;

       
        path = insert_new_path(path, jointState_k, gripperState);
    }

    return path;
}


//move 
void move2(MatrixX3d traj, ros::Publisher p)    { // traj could be also Matrix<double, Eigen::Dynamic, 3>
    //move on selected point
}


void open_gripper(ros::Publisher p){
    VectorXd gripper_trajectory(8);
    VectorXd realMeasures(8);
    VectorXd joints(6);
    VectorXd gripper(2);

    realMeasures = get_robot_values();
    joints = get_joint_state(realMeasures);
    gripper << realMeasures(6), realMeasures(7);

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

void close_gripper(ros::Publisher p){
    VectorXd gripper_trajectory(8);
    VectorXd realMeasures(8);
    VectorXd joints(6);
    VectorXd gripper(2);

    realMeasures = get_robot_values();
    joints = get_joint_state(realMeasures);
    gripper << realMeasures(6), realMeasures(7);

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





 MatrixX3d  get_trajectory(double t_init, double t_final, V6d joints_init, V6d joints_final, double speed){
//     //initial speed = final speed = 0
    MatrixX3d a;
//     a << a0, a1, a2, a3;

//     V8d robot_measures = get_robot_values();
//     V6d joint_state = get_joint_state(robot_measures);

//     M4d times;
//     times << 1, t_init, pow(t_init, 2), pow(t_init, 3),
//            1, t_final, pow(t_final, 2), pow(t_final, 3),
//            0, 1, t_init, pow(t_init,2),
//            0, 1, t_final, pow(t_final, 2);
        
//     Matrix6d q; //to get from move pub

//     for(int j=0;j<6;j++)
//         for(int i = 0; i<6; i++)
//             a(j,i) = times.inverse() * q(js(i));
//     return  a; //missing to calculate polinomial trajectory with a coefficients
return a;

 }

void grasping_operation(Vector3d block_coords, Matrix3d block_pose, Vector3d final_coords, Matrix3d final_pose, ros::Publisher publisher){
    
    // TODO cambia nomi qui -> potrebbe servire scegliere un altezza a cui prendere e mollare il blocco se quella passata da vision non va bene
    const double motion_height = 0.20;

    // set robot arm in base config
    set_start_position(publisher);    

    Vector3d above_block_coords;    // position exactly above the block at motion_height
    above_block_coords << block_coords(X_axis), block_coords(Y_axis), motion_height;


    // CHECKING positions
    ROS_INFO("Validating grasping position...\n");
    validate_position(above_block_coords);
    ROS_INFO("Validating target position...\n");
    validate_position(final_coords);
    
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