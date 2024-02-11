#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <Eigen/Core>	
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <string.h>
#include <math.h>


using namespace std;
using namespace Eigen;

struct posrot // Struct to stor the end-effector position and rotation
{
    Matrix3f rot; //3x3 matrix with float value
    Vector3f xyz; //vector with 3 float value 
};


Matrix4f t10f(float th1) //First joint matrix
{
    Matrix4f mtx;
    VectorXf a(6);// Vector A with distances in meters
    VectorXf d(6);// Vector D with distances in meters

    mtx << cos(th1), -sin(th1), 0, 0,// th* are the angles of the joints
        sin(th1), cos(th1), 0, 0,
        0, 0, 1, d(0),
        0, 0, 0, 1;
    a << 0, -0.425, -0.3922, 0, 0, 0;
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;
    

    return mtx;
}


Matrix4f t21f(float th2) //Second joint matrix
{
    Matrix4f mtx;
    VectorXf a(6);
    VectorXf d(6);

    mtx << cos(th2), -sin(th2), 0, 0,
        0, 0, -1, 0,
        sin(th2), cos(th2), 0, 0,
        0, 0, 0, 1;
    a << 0, -0.425, -0.3922, 0, 0, 0;
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;
    

    return mtx;
}


Matrix4f t32f(float th3) //Third joint matrix
{
    Matrix4f mtx;
    VectorXf a(6);
    VectorXf d(6);

    mtx << cos(th3), -sin(th3), 0, a(1),
        sin(th3), cos(th3), 0, 0,
        0, 0, 1, d(2),
        0, 0, 0, 1;
    a << 0, -0.425, -0.3922, 0, 0, 0;
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;
    

    return mtx;
}


Matrix4f t43f(float th4) //Fourth joint matrix
{
    Matrix4f mtx;
    VectorXf a(6);
    VectorXf d(6);

    mtx << cos(th4), -sin(th4), 0, a(2),
        sin(th4), cos(th4), 0, 0,
        0, 0, 1, d(3),
        0, 0, 0, 1;
    a << 0, -0.425, -0.3922, 0, 0, 0;
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;
    

    return mtx;
}


Matrix4f t54f(float th5) //Fifth joint matrix
{
    Matrix4f mtx;
    VectorXf a(6);
    VectorXf d(6);

    mtx << cos(th5), -sin(th5), 0, 0,
        0, 0, -1, -d(4),
        sin(th5), cos(th5), 0, 0,
        0, 0, 0, 1;
    a << 0, -0.425, -0.3922, 0, 0, 0;
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;
    

    return mtx;
}


Matrix4f t65f(float th6) //Sixth joint matrix
{
    Matrix4f mtx;
    VectorXf a(6);
    VectorXf d(6);

    mtx << cos(th6), -sin(th6), 0, 0,
        0, 0, 1, d(5),
        -sin(th6), -cos(th6), 0, 0,
        0, 0, 0, 1;
    a << 0, -0.425, -0.3922, 0, 0, 0;
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;
    

    return mtx;
}


frame directKin(VectorXf th)//with this frame we compute the direct kinematics
{
    frame ret;
    Matrix4f T60 = t10f(th(0)) * t21f(th(1)) * t32f(th(2)) * t43f(th(3)) * t54f(th(4)) * t65f(th(5));

    ret.rot = T60.block(0, 0, 3, 3); //block of size 3x3 from 00 position
    ret.xyz = T60.block(0, 3, 3, 1); //block of size 3x1 from 03 position

    return ret;
}


MatrixXf invKin(frame &frame)//Compute the inverse kinematics 
{

    VectorXf a(6);
    VectorXf d(6);
    a << 0, -0.425, -0.3922, 0, 0, 0;
    d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;

    Matrix4f T60;
    T60.setZero();
    T60.block(0, 0, 3, 3) = frame.rot;
    T60.block(0, 3, 3, 1) = frame.xyz;
    T60(3, 3) = 1;

    
    //Finding th1
    Vector4f p50;
    p50 = T60 * Vector4f(0, 0, -d(5), 1);//Matlab start from 1, C from 0 (M-6 C-5)
    //Trascrizione diretta da matlab
    float th1_1 = real(atan2(p50(1), p50(0)) + acos(d(3) / hypot(p50(1), p50(0)))) + M_PI / 2;
    float th1_2 = real(atan2(p50(1), p50(0)) - acos(d(3) / hypot(p50(1), p50(0)))) + M_PI / 2;

    
    //Finding th5
    float th5_1 = +real(acos((frame.xyz(0) * sin(th1_1) - frame.xyz(1) * cos(th1_1) - d(3)) / d(5)));
    float th5_2 = -real(acos((frame.xyz(0) * sin(th1_1) - frame.xyz(1) * cos(th1_1) - d(3)) / d(5)));
    float th5_3 = +real(acos((frame.xyz(0) * sin(th1_2) - frame.xyz(1) * cos(th1_2) - d(3)) / d(5)));
    float th5_4 = -real(acos((frame.xyz(0) * sin(th1_2) - frame.xyz(1) * cos(th1_2) - d(3)) / d(5)));

    
    //Related to th11 a th51
    Matrix4f t06;
    t06 = T60.inverse();//Inverse of T60

    Vector3f Xhat;
    Xhat = t06.block(0, 0, 3, 1);//Take the vector from the T06 matrix from the position 00 to 20
    Vector3f Yhat;
    Yhat = t06.block(0, 1, 3, 1);//Take the vector from the T06 matrix from the position 01 to 21

    float th6_1 = real(atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1))) / sin(th5_1), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1))) / sin(th5_1)));
    //Related to th11 a th52
    float th6_2 = real(atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1))) / sin(th5_2), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1))) / sin(th5_2)));
    //Related to th12 a th53
    float th6_3 = real(atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2))) / sin(th5_3), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2))) / sin(th5_3)));
    //Related to th12 a th54
    float th6_4 = real(atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2))) / sin(th5_4), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2))) / sin(th5_4)));

    Matrix4f t41m;
    Vector3f p41_1;
    Vector3f p41_2;
    Vector3f p41_3;
    Vector3f p41_4;
    float p41xz_1;
    float p41xz_2;
    float p41xz_3;
    float p41xz_4;

    t41m = t10f(th1_1).inverse() * T60 * t65f(th6_1).inverse() * t54f(th5_1).inverse();
    p41_1 = t41m.block(0, 3, 3, 1);
    p41xz_1 = hypot(p41_1(0), p41_1(2));

    t41m = t10f(th1_1).inverse() * T60 * t65f(th6_2).inverse() * t54f(th5_2).inverse();
    p41_2 = t41m.block(0, 3, 3, 1);
    p41xz_2 = hypot(p41_2(0), p41_2(2));

    t41m = t10f(th1_2).inverse() * T60 * t65f(th6_3).inverse() * t54f(th5_3).inverse();
    p41_3 = t41m.block(0, 3, 3, 1);
    p41xz_3 = hypot(p41_3(0), p41_3(2));

    t41m = t10f(th1_2).inverse() * T60 * t65f(th6_4).inverse() * t54f(th5_4).inverse();
    p41_4 = t41m.block(0, 3, 3, 1);
    p41xz_4 = hypot(p41_4(0), p41_4(2));

    //8 possible configuration of th3

    float th3_1;

    if ((pow(p41xz_1, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) > 1)
    {
        th3_1 = 0;
    }
    else if ((pow(p41xz_1, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) < -1)
    {
        th3_1 = M_PI;
    }
    else
    {
        th3_1 = acos((pow(p41xz_1, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)));
    }

    float th3_2;

    if ((pow(p41xz_2, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) > 1)
    {
        th3_2 = 0;
    }
    else if ((pow(p41xz_2, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) < -1)
    {
        th3_2 = M_PI;
    }
    else
    {
        th3_2 = acos((pow(p41xz_2, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)));
    }

    float th3_3;

    if ((pow(p41xz_3, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) > 1)
    {
        th3_3 = 0;
    }
    else if ((pow(p41xz_3, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) < -1)
    {
        th3_3 = M_PI;
    }
    else
    {
        th3_3 = acos((pow(p41xz_3, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)));
    }

    float th3_4;

    if ((pow(p41xz_4, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) > 1)
    {
        th3_4 = 0;
    }
    else if ((pow(p41xz_4, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) < -1)
    {
        th3_4 = M_PI;
    }
    else
    {
        th3_4 = acos((pow(p41xz_4, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)));
    }

    float th3_5 = -th3_1;
    float th3_6 = -th3_2;
    float th3_7 = -th3_3;
    float th3_8 = -th3_4;

    // find th2

    float th2_1 = atan2(-p41_1(2), -p41_1(0)) - asin((-a(2) * sin(th3_1)) / p41xz_1);
    float th2_2 = atan2(-p41_2(2), -p41_2(0)) - asin((-a(2) * sin(th3_2)) / p41xz_2);
    float th2_3 = atan2(-p41_3(2), -p41_3(0)) - asin((-a(2) * sin(th3_3)) / p41xz_3);
    float th2_4 = atan2(-p41_4(2), -p41_4(0)) - asin((-a(2) * sin(th3_4)) / p41xz_4);

    float th2_5 = atan2(-p41_1(2), -p41_1(0)) - asin((a(2) * sin(th3_1)) / p41xz_1);
    float th2_6 = atan2(-p41_2(2), -p41_2(0)) - asin((a(2) * sin(th3_2)) / p41xz_2);
    float th2_7 = atan2(-p41_3(2), -p41_3(0)) - asin((a(2) * sin(th3_3)) / p41xz_3);
    float th2_8 = atan2(-p41_4(2), -p41_4(0)) - asin((a(2) * sin(th3_4)) / p41xz_4);

    // find th4

    Matrix4f t43m;
    Vector3f xhat43;
    t43m = t32f(th3_1).inverse() * t21f(th2_1).inverse() * t10f(th1_1).inverse() * T60 * t65f(th6_1).inverse() * t54f(th5_1).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    float th4_1 = atan2(xhat43(1), xhat43(0));

    t43m = t32f(th3_2).inverse() * t21f(th2_2).inverse() * t10f(th1_1).inverse() * T60 * t65f(th6_2).inverse() * t54f(th5_2).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    float th4_2 = atan2(xhat43(1), xhat43(0));

    t43m = t32f(th3_3).inverse() * t21f(th2_3).inverse() * t10f(th1_2).inverse() * T60 * t65f(th6_3).inverse() * t54f(th5_3).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    float th4_3 = atan2(xhat43(1), xhat43(0));

    t43m = t32f(th3_4).inverse() * t21f(th2_4).inverse() * t10f(th1_2).inverse() * T60 * t65f(th6_4).inverse() * t54f(th5_4).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    float th4_4 = atan2(xhat43(1), xhat43(0));

    t43m = t32f(th3_5).inverse() * t21f(th2_5).inverse() * t10f(th1_1).inverse() * T60 * t65f(th6_1).inverse() * t54f(th5_1).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    float th4_5 = atan2(xhat43(1), xhat43(0));

    t43m = t32f(th3_6).inverse() * t21f(th2_6).inverse() * t10f(th1_1).inverse() * T60 * t65f(th6_2).inverse() * t54f(th5_2).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    float th4_6 = atan2(xhat43(1), xhat43(0));

    t43m = t32f(th3_7).inverse() * t21f(th2_7).inverse() * t10f(th1_2).inverse() * T60 * t65f(th6_3).inverse() * t54f(th5_3).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    float th4_7 = atan2(xhat43(1), xhat43(0));

    t43m = t32f(th3_8).inverse() * t21f(th2_8).inverse() * t10f(th1_2).inverse() * T60 * t65f(th6_4).inverse() * t54f(th5_4).inverse();
    xhat43 = t43m.block(0, 0, 3, 1);
    float th4_8 = atan2(xhat43(1), xhat43(0));

    MatrixXf TH(8, 6);
    TH << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
          th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
          th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
          th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
          th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
          th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
          th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
          th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

    return TH;
}


MatrixXf JacobianMatrix(VectorXf q)//From Ur5Jac we do the computation of Jacobian
{
    VectorXf A(6);
    VectorXf D(6);
    A << 0, -0.425, -0.3922, 0, 0, 0;
    D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14;


    MatrixXf J(6, 6);
    J.setZero(); //clean the matrix and set 0 everywhere


    MatrixXf J1(6, 1);
    J1 << D(4) * (cos(q(0)) * cos(q(4)) + cos(q(1) + q(2) + q(3)) * sin(q(0)) * sin(q(4))) + D(2) * cos(q(0)) + D(3) * cos(q(0)) - A(2) * cos(q(1) + q(2)) * sin(q(0)) - A(1) * cos(q(1)) * sin(q(0)) - D(4) * sin(q(1) + q(2) + q(3)) * sin(q(0)),
          D(4) * (cos(q(4)) * sin(q(0)) - cos(q(1) + q(2) + q(3)) * cos(q(0)) * sin(q(4))) + D(2) * sin(q(0)) + D(3) * sin(q(0)) + A(2) * cos(q(1) + q(2)) * cos(q(0)) + A(1) * cos(q(0)) * cos(q(1)) + D(4) * sin(q(1) + q(2) + q(3)) * cos(q(0)),
          0,
          0,
          0,
          1;


    MatrixXf J2(6, 1);
    J2 << -cos(q(0)) * (A(2) * sin(q(1) + q(2)) + A(1) * sin(q(1)) + D(4) * (sin(q(1) + q(2)) * sin(q(3)) - cos(q(1) + q(2)) * cos(q(3))) - D(4) * sin(q(4)) * (cos(q(1) + q(2)) * sin(q(3)) + sin(q(1) + q(2)) * cos(q(3)))),
          -sin(q(0)) * (A(2) * sin(q(1) + q(2)) + A(1) * sin(q(1)) + D(4) * (sin(q(1) + q(2)) * sin(q(3)) - cos(q(1) + q(2)) * cos(q(3))) - D(4) * sin(q(4)) * (cos(q(1) + q(2)) * sin(q(3)) + sin(q(1) + q(2)) * cos(q(3)))),
           A(2) * cos(q(1) + q(2)) - (D(4) * sin(q(1) + q(2) + q(3) + q(4))) / 2 + A(1) * cos(q(1)) + (D(4) * sin(q(1) + q(2) + q(3) - q(4))) / 2 + D(4) * sin(q(1) + q(2) + q(3)),
           sin(q(0)),
          -cos(q(0)),
           0;


    MatrixXf J3(6, 1);
    J3 << cos(q(0)) * (D(4) * cos(q(1) + q(2) + q(3)) - A(2) * sin(q(1) + q(2)) + D(4) * sin(q(1) + q(2) + q(3)) * sin(q(4))),
          sin(q(0)) * (D(4) * cos(q(1) + q(2) + q(3)) - A(2) * sin(q(1) + q(2)) + D(4) * sin(q(1) + q(2) + q(3)) * sin(q(4))),
          A(2) * cos(q(1) + q(2)) - (D(4) * sin(q(1) + q(2) + q(3) + q(4))) / 2 + (D(4) * sin(q(1) + q(2) + q(3) - q(4))) / 2 + D(4) * sin(q(1) + q(2) + q(3)),
          sin(q(0)),
         -cos(q(0)),
          0;


    MatrixXf J4(6, 1);
    J4 << D(4) * cos(q(0)) * (cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3)) * sin(q(4))),
          D(4) * sin(q(0)) * (cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3)) * sin(q(4))),
          D(4) * (sin(q(1) + q(2) + q(3) - q(4)) / 2 + sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4)) / 2),
          sin(q(0)),
         -cos(q(0)),
          0;


    MatrixXf J5(6, 1);
    J5 << -D(4) * sin(q(0)) * sin(q(4)) - D(4) * cos(q(1) + q(2) + q(3)) * cos(q(0)) * cos(q(4)),
           D(4) * cos(q(0)) * sin(q(4)) - D(4) * cos(q(1) + q(2) + q(3)) * cos(q(4)) * sin(q(0)),
          -D(4) * (sin(q(1) + q(2) + q(3) - q(4)) / 2 + sin(q(1) + q(2) + q(3) + q(4)) / 2),
         sin(q(1) + q(2) + q(3)) * cos(q(0)),
         sin(q(1) + q(2) + q(3)) * sin(q(0)),
        -cos(q(1) + q(2) + q(3));


    MatrixXf J6(6, 1);
    J6 << 0,
          0,
          0,
         cos(q(4)) * sin(q(0)) - cos(q(1) + q(2) + q(3)) * cos(q(0)) * sin(q(4)),
        -cos(q(0)) * cos(q(4)) - cos(q(1) + q(2) + q(3)) * sin(q(0)) * sin(q(4)),
        -sin(q(1) + q(2) + q(3)) * sin(q(4));


    J << J1, J2, J3, J4, J5, J6;


    return J; //Is the result of all the J matrix (column vector) shifted to the left
}


Matrix3f eul2rotm(Vector3f rpy) 
{
    Matrix3f R;
    R = AngleAxisf(rpy(0), Vector3f::UnitZ()) * AngleAxisf(rpy(1), Vector3f::UnitY()) * AngleAxisf(rpy(2), Vector3f::UnitX());
    return R;
}


#endif