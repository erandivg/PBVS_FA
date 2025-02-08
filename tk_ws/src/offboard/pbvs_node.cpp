#include "utils.h"
#include "vision_utils.h"
#include <Eigen/Dense>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdlib.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp> // Para conversión entre OpenCV y Eigen


cv::Mat refImage, currentImage;
double t0,t, theta;
Vector3D_t u_0;

static const std::string REF_IMAGE_NAME = "/home/erandi/Documents/ros/tk_ws/src/Archive/offboard/image_reference/reference_image.jpg";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cvPtr;

	try
	{ 
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
  cvPtr->image.copyTo(currentImage);
}

void Rodriguez_theta_u(const Eigen::Matrix3d &R, Eigen::Vector3d &u, double &theta) {
  cv::Mat Rcv, omega_cv;
  cv::eigen2cv(R, Rcv);
  cv::Rodrigues(Rcv, omega_cv);
  Eigen::Vector3d omega;
  cv::cv2eigen(omega_cv, omega);

  theta = omega.norm(); // Magnitud del vector de rotación
  //std::cout <<"theta es: " << theta << " rad " << std::endl;

  if (theta > 1e-6) { // Evitar división por cero
      u_0 = omega / theta;
  } else {
      u_0 = Eigen::Vector3d::Zero();
  }
}

Eigen::Matrix3d compute_L_theta_u(double theta, const Eigen::Vector3d &u) {
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d u_x;
  
  // Matriz skew-symmetric [u]_x
  u_x <<  0, -u.z(),  u.y(),
          u.z(),  0, -u.x(),
         -u.y(), u.x(),  0;
  
  double sinc_theta = sin(theta) / theta;
  double sinc_theta_half = sin(theta / 2.0) / (theta / 2.0);

  Eigen::Matrix3d L_theta_u = I - (theta / 2.0) * u_x +
                             (1 - sinc_theta / (sinc_theta_half * sinc_theta_half)) * (u_x * u_x);

  return L_theta_u;
}

Eigen::Vector3d compute_vc(const Eigen::Vector3d& tc, double lt, double k1, double k2, const Eigen::Matrix3d& R) {
  double norm_tc2 = tc.squaredNorm();
  double factor1 = k1 * (lt * lt - norm_tc2);
  double factor2 = k2 / (lt * lt - norm_tc2);
  return -R.inverse() * (factor1 * tc + factor2 * tc);
}

Eigen::Vector3d compute_wc(const Eigen::Vector3d& theta_u, double lr, double k1, double k2, const Eigen::Matrix3d& L_theta_u) {
  double norm_theta_u2 = theta_u.squaredNorm();
  double factor1 = k1 * (lr * lr - norm_theta_u2);
  double factor2 = k2 / (lr * lr - norm_theta_u2);
  return -L_theta_u.inverse() * (factor1 * theta_u + factor2 * theta_u);
}

int main(int argc, char **argv)
{
    Matrix_t K(3,3); //Calibration of the camera matrix
    Matrix_t H(3,3); // Homography matrix
    Matrix_t R(3,3); // Rotation between current frame and the reference frame
    Matrix_t L_theta_u(3,3);
    
    Vector3D_t t;    // Translation between current frame and the reference frame
    Vector3D_t n;    // normal vector in the reference plane
    Vector3D_t u;   // Rodrigues vector
    Vector3D_t Uv;   // Control input for the translational velocity
    Vector3D_t Uw;   // Control input for the rotational velocity

    Vector3D_t vc;
    Vector3D_t wc;

    //double lt=4.6;
    //double lr=1.4;
    //double k1=0.008; 
    //double k2=0.012;

    double lt=5.6;
    double lr=3.0;
    double k1=0.008; 
    double k2=0.009;

    FPTYPE d;       // Distance between the camera and the reference plane
    homographySolution homSolution;
    int counter = 0;


    K << 554.382713,     0.0   , 320.0, 
         0.0       , 554.382713, 240.0, 
         0.0       ,     0.0   ,   1.0;
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <topic_name> <lambdav> <lambdaw>" << std::endl;
        return 1;
    }

    
    FPTYPE lambdav = atof(argv[2]);
    FPTYPE lambdaw = atof(argv[3]);

    refImage = cv::imread(REF_IMAGE_NAME, cv::IMREAD_COLOR);
    if(refImage.empty())
    {
        std::cout << "Could not read the image: " << REF_IMAGE_NAME << std::endl;
        return 1;
    }
    else{
        std::cout << "Ref image is ready: " << REF_IMAGE_NAME << std::endl;
    }

    ros::init(argc, argv, "pbvs_node");
    ros::NodeHandle nh;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    
    std::string topicName;
    if(argc != 1){
      topicName = argv[1];
    } 
    else
    {
      topicName = "/my_image";
      std::cout <<"No topic name given. The default topic name will be used: " << topicName << std::endl;
    }

    ros::Publisher homography_pub   = nh.advertise<std_msgs::Float64MultiArray>("/homograpy_numerical",10);
    ros::Publisher ck_t_ct_pub      = nh.advertise<std_msgs::Float64MultiArray>("/ck_t_ct",10);
    ros::Publisher n_pub            = nh.advertise<std_msgs::Float64MultiArray>("/n_plane_vector",10);
    ros::Publisher Uv_pub           = nh.advertise<std_msgs::Float64MultiArray>("/Uv",10);
    ros::Publisher Uw_pub           = nh.advertise<std_msgs::Float64MultiArray>("/Uw",10);
    ros::Publisher d_pub            = nh.advertise<std_msgs::Float64>("/d_value",10);


    image_transport::Subscriber sub = it.subscribe(topicName, 1, imageCallback);

    //ros::spin();
    ros::Rate loop_rate(30);
    ros::Duration(2).sleep(); //Retardo  de tiempo

    std_msgs::Float64MultiArray H_msg,ck_t_ct_msg,n_msg, Uv_msg, Uw_msg;
    std_msgs::Float64 d_msg;

    do{
          if(currentImage.empty())
          {
              std::cout << "Could not read the current image: "  << std::endl;
          }
          else
          {
              //std::cout << "current image is ready: "  << std::endl;
              cv::imwrite("/home/erandi/Documents/ros/tk_ws/src/Archive/offboard/src/current_image.jpg",currentImage);
              EstimateHomography(refImage, currentImage, K, H, counter );
              RecoverFromHomography(H,R,t,n,d, counter, homSolution);
              //Control basico
              Rodriguez(R,u);
              PBVSController(R,t,u,Uv,Uw,lambdav,lambdaw);

              //control dado
              Rodriguez_theta_u(R,u,theta);
              L_theta_u = compute_L_theta_u(theta, u_0);
              //std::cout <<"L_theta_u es: " << L_theta_u << std::endl;
              vc=compute_vc(t, lt, k1, k2, R);
              std::cout <<"vc es: " << vc << std::endl;
              std::cout <<"Uv es: " << Uv << std::endl;
              wc=compute_wc(u, lr, k1, k2, L_theta_u);
              std::cout <<"wc es: " << wc << std::endl;
              std::cout <<"Uw es: " << Uw << std::endl;
              
              tf::matrixEigenToMsg(vc, Uv_msg);//esto es lo que se cambia de Uv a vc
              tf::matrixEigenToMsg(wc, Uw_msg);//de Uw a wc

              tf::matrixEigenToMsg(H, H_msg);
              tf::matrixEigenToMsg(t, ck_t_ct_msg);
              tf::matrixEigenToMsg(n, n_msg);
              homography_pub.publish(H_msg);
              ck_t_ct_pub.publish(ck_t_ct_msg);
              n_pub.publish(n_msg);
              d_msg.data = d;
              d_pub.publish(d_msg);
              Uv_pub.publish(Uv_msg);
              Uw_pub.publish(Uw_msg);
              
          }
          counter++;
          ros::spinOnce();   //Activa callbacks
          loop_rate.sleep(); //Espera hasta completar el loop rate
    }while(ros::ok());

    //cv::destroyWindow("Video Stream");
    return 0;
}
