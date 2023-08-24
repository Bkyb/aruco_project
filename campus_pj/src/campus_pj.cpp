#include "campus_pj.h"
#include "ui_campus_pj.h"


campus_pj::campus_pj(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::campus_pj)
{
  ui->setupUi(this);

  int argc = 0; char **argv = NULL;

  ros::init(argc, argv,"ros_pj");
  ros::NodeHandlePtr n;
  n.reset(new ros::NodeHandle("~"));

  ros_timer = new QTimer(this);
   connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
   ros_timer->start(1);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

  color_image_sub_ = n->subscribe("/camera/color/image_raw", 1000, &campus_pj::color_image_sub_cb, this);
  depth_image_sub_ = n->subscribe("/camera/aligned_depth_to_color/image_raw", 1000, &campus_pj::depth_image_sub_cb, this);
  //color_image_sub_ = n->subscribe("/camera/color/image_raw", 1000, &campus_pj::color_image_sub_cb, this);
  color_camera_info_sub_ = n->subscribe("/camera/color/camera_info", 1000, &campus_pj::color_camera_info_sub_cb, this);
   ui->stackedWidget->setCurrentIndex(0);




   /*cv::FileStorage fs1(filename, cv::FileStorage::READ);
   std::cout << "reading R and T" << std::endl;
   fs1["R"] >> R;
   fs1["T"] >> T;
   std::cout << "R = " << R << "\n";
   std::cout << "T = " << T << std::endl;
   fs1.release();*/
}

int color_info_count = 0;
double intrinsic_parameter[9];
double discoeffs[5];
int match_count = 0;
rs2_intrinsics RS_camera_info_;

int squaresX = 8;//인쇄한 보드의 가로방향 마커 갯수
int squaresY = 5;//인쇄한 보드의 세로방향 마커 갯수
float squareLength = 30;//검은색 테두리 포함한 정사각형의 한변 길이, mm단위로 입력
float markerLength_chess = 23;//인쇄물에서의 마커 한변의 길이, mm단위로 입력
float markerLength = 30;//single인쇄물에서의 마커 한변의 길이, mm단위로 입력
int dictionaryId = 11;//DICT_6X6_250=10
//std::string outputFile = "output.txt";

int calibrationFlags = 0;
float aspectRatio = 1;

cv::Mat A(3, 3, CV_64FC1, intrinsic_parameter);	// camera matrix
cv::Mat distCoeffs(5, 1, CV_64FC1, discoeffs);
//cv::Mat rvec, tvec;	// rotation & translation vectors
cv::Mat R;

cv::Mat c2g_rvec = (cv::Mat_<float>(3, 3));
cv::Mat  c2g_tvec = (cv::Mat_<float>(3, 1));

cv::Mat t2c_rvec = (cv::Mat_<float>(3, 3));
cv::Mat  t2c_tvec = (cv::Mat_<float>(3, 1));

cv::Mat g2b_rvec = (cv::Mat_<float>(3, 3));
cv::Mat g2b_tvec = (cv::Mat_<float>(3, 1));

campus_pj::~campus_pj()
{
  ROS_INFO("Campus ampus SHUTDOWN!");
  delete ui;
}

void campus_pj::spinOnce()
{
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

int campus_pj::movej(float *fTargetPos, float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType)
{
  ui->textEdit_log->append("Move_joint START!");
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
      ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs::MoveJoint>( "/dsr01m1013/motion/move_joint");



      dsr_msgs::MoveJoint srv;

      for(int i=0; i<6; i++)
          srv.request.pos[i] = fTargetPos[i];
      srv.request.vel = fTargetVel;
      srv.request.acc = fTargetAcc;
      srv.request.time = fTargetTime;
      srv.request.radius = fBlendingRadius;
      srv.request.mode = nMoveMode;
      srv.request.blendType = nBlendingType;
      srv.request.syncType = nSyncType;


      QString text_for_append;

      text_for_append.sprintf("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
      ui->textEdit_log->append(text_for_append);

      text_for_append.sprintf("  <vel> %7.3f , <acc> %7.3f, <time> %7.3f",srv.request.vel, srv.request.acc, srv.request.time);
      ui->textEdit_log->append(text_for_append);

      text_for_append.sprintf("  <mode> %d , <radius> %7.3f, <blendType> %d",srv.request.mode, srv.request.radius, srv.request.blendType);
      ui->textEdit_log->append(text_for_append);

      if(srvMoveJoint.call(srv))
      {
         text_for_append.sprintf("  receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
          ui->textEdit_log->append(text_for_append);
          return (srv.response.success);
      }
      else
      {        
           ui->textEdit_log->append("  Failed to call service dr_control_service : move_joint");
          ros::shutdown();
          return -1;
      }

      return 0;

}

int campus_pj::movel(float *fTargetPos, float *fTargetVel, float *fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType)
{
      ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
      ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs::MoveLine>( "/dsr01m1013/motion/move_line");
      dsr_msgs::MoveLine srv;

      for(int i=0; i<6; i++)
          srv.request.pos[i] = fTargetPos[i];
      for(int i=0; i<2; i++){
          srv.request.vel[i] = fTargetVel[i];
          srv.request.acc[i] = fTargetAcc[i];
      }
      srv.request.time = fTargetTime;
      srv.request.radius = fBlendingRadius;
      srv.request.ref  = nMoveReference;
      srv.request.mode = nMoveMode;
      srv.request.blendType = nBlendingType;
      srv.request.syncType = nSyncType;


       QString text_for_append;

       text_for_append.sprintf("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
       ui->textEdit_log->append(text_for_append);
      text_for_append.sprintf("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",srv.request.vel[0],srv.request.vel[1],srv.request.acc[0],srv.request.acc[1], srv.request.time);
      ui->textEdit_log->append(text_for_append);
      text_for_append.sprintf("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);
      ui->textEdit_log->append(text_for_append);

      if(srvMoveLine.call(srv))
      {
          text_for_append.sprintf("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
          ui->textEdit_log->append(text_for_append);
          return (srv.response.success);
      }
      else
      {
          text_for_append.sprintf("Failed to call service dr_control_service : move_line\n");
          ui->textEdit_log->append(text_for_append);
          ros::shutdown();
          return -1;
      }

      return 0;
}

void campus_pj::color_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
  color_image_raw = cv_ptr->image.clone();
  color_image = cv_ptr->image.clone();
}

void campus_pj::depth_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::TYPE_16UC1);
  cv_ptr->image.convertTo(depth_image, CV_32F, 0.001);

}

void campus_pj::color_camera_info_sub_cb(const sensor_msgs::CameraInfoConstPtr &depth_camera_info)
{
    if(color_info_count >0) return;

    for(int i=0;i<9;i++)
    {
      intrinsic_parameter[i] = depth_camera_info->K[i];
      //std::cout << intrinsic_parameter[i] << std::endl;
    }

    RS_camera_info_.width = depth_camera_info->width;    // Image Resoltusion width
    RS_camera_info_.height = depth_camera_info->height;  // Image Resoltusion height
    RS_camera_info_.fx = depth_camera_info->K[0];        // 초점거리 x
    RS_camera_info_.fy = depth_camera_info->K[4];        // 초점거리 y
    RS_camera_info_.ppx = depth_camera_info->K[2];       // 주점 x
    RS_camera_info_.ppy = depth_camera_info->K[5];       // 주점 y
    RS_camera_info_.model = RS2_DISTORTION_MODIFIED_BROWN_CONRADY;

    for(int i=0;i<5;i++)
    {
      discoeffs[i]= depth_camera_info->D[i];
      RS_camera_info_.coeffs[i] = depth_camera_info->D[i];
      //std::cout << RS_camera_info_.coeffs[i] << std::endl;
    }

    color_info_count++;
}


void campus_pj::calculateEnd2Base(float& x, float& y, float& z, float& r, float& p, float& yw){
    // 초기 설정
    float r_rad = r * M_PI / 180.0;
    float p_rad = p * M_PI / 180.0;
    float yw_rad = yw * M_PI / 180.0;   
 
 

    cv::Matx44d camera2end( 0.9995884550916401, -0.0286509350929972, -0.001429813206316577, -31.81668840797239,
                            0.02858327133778271, 0.9989768060104955, -0.03504764831909967, -99.62247870764079,
                            0.002432498127190477, 0.03499235589904547, 0.9993846196442566, -2.546049086854508,
                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00

    );
    

    // base to camera의 변환 행렬 계산
    cv::Vec3d translation(x, y, z);

    cv::Matx33d Rz1(
        std::cos(r_rad), -std::sin(r_rad), 0,
        std::sin(r_rad), std::cos(r_rad), 0,
        0, 0, 1
    );
    cv::Matx33d Ry(
        std::cos(p_rad), 0, std::sin(p_rad),
        0, 1, 0,
        -std::sin(p_rad), 0, std::cos(p_rad)
    );
    cv::Matx33d Rz2(
        std::cos(yw_rad), -std::sin(yw_rad), 0,
        std::sin(yw_rad), std::cos(yw_rad), 0,
        0, 0, 1
    );

    cv::Matx33d R_cam2b = Rz1 * Ry * Rz2;
    cv::Matx44d T_cam2b = cv::Matx44d::eye();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T_cam2b(i, j) = R_cam2b(i, j);
        }
        T_cam2b(i, 3) = translation(i);
    }

    // end effector to camera의 역변환 행렬
    cv::Matx44d T_end2camera = camera2end.inv();

    // end effector의 위치 추출
    cv::Matx44d T_end2base = T_cam2b * T_end2camera;
    x = T_end2base(0, 3);
    y = T_end2base(1, 3);
    z = T_end2base(2, 3);

    // end effector의 방향 추출
    cv::Matx33d R_end2base;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base(i, j) = T_end2base(i, j);
        }
    }
    
    p_rad = std::acos(R_end2base(2, 2));

    if (R_end2base(1, 2) / std::sin(p_rad) > 0) {
        r_rad = std::acos(R_end2base(0, 2) / std::sin(p_rad));
    } else {
        r_rad = -std::acos(R_end2base(0, 2) / std::sin(p_rad));
    }
    if (R_end2base(2, 1) / std::sin(p_rad) > 0) {
        yw_rad = std::acos(-R_end2base(2, 0) / std::sin(p_rad));
    } else {
        yw_rad = -std::acos(-R_end2base(2, 0) / std::sin(p_rad));
    }

    r = r_rad * 180.0 / M_PI;
    p = p_rad * 180.0 / M_PI;
    yw = yw_rad * 180.0 / M_PI;
}

///////////////////////////////////////////HOME////////////////////////////////////////////

void campus_pj::on_pushButton_widget_process_home_clicked()
{
  QMessageBox mb;

  mb.setText("Are you sure you want to return to home?");
  mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
  mb.setDefaultButton(QMessageBox::Cancel);
  mb.setIcon(QMessageBox::Icon::Warning);

  mb.move(470, 350);

  int ret = mb.exec();

  switch(ret)
  {
  case QMessageBox::Ok :
    ui->stackedWidget->setCurrentIndex(0);
    break;

  case QMessageBox::Cancel:
    break;
  }
}

void campus_pj::on_pushButton_calibration_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void campus_pj::on_pushButton_start_process_clicked()
{
    ui->stackedWidget->setCurrentIndex(3);
    // cv::Mat showimage;
    // showimage = color_image.clone();
    // cv::resize(showimage, showimage, cv::Size(640, 360));
    // ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
}


///////////////////////////////////////////CALIBRATION////////////////////////////////////////////////////

void campus_pj::on_pushButton_haneye_calibration_home_clicked()
{
  QMessageBox mb;

  mb.setText("Are you sure you want to return to home?");
  mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
  mb.setDefaultButton(QMessageBox::Cancel);
  mb.setIcon(QMessageBox::Icon::Warning);

  mb.move(470, 350);
  
  int ret = mb.exec();

  switch(ret)
  {
  case QMessageBox::Ok :
    ui->stackedWidget->setCurrentIndex(0);
    break;

  case QMessageBox::Cancel:
    break;
  }
}

void campus_pj::on_pushButton_haneye_calibration_intrpara_clicked()
{
  double ffx, ffy, ccx, ccy;

  QString fxText = ui->textEdit_get_fx->toPlainText();
  QString fyText = ui->textEdit_get_fy->toPlainText();
  QString cxText = ui->textEdit_get_cx->toPlainText();
  QString cyText = ui->textEdit_get_cy->toPlainText();

  ffx = !fxText.isEmpty() ? fxText.toDouble() : intrinsic_parameter[0];
  ffy = !fyText.isEmpty() ? fyText.toDouble() : intrinsic_parameter[4];
  ccx = !cxText.isEmpty() ? cxText.toDouble() : intrinsic_parameter[2];
  ccy = !cyText.isEmpty() ? cyText.toDouble() : intrinsic_parameter[5];


  intrinsic_parameter[0] = ffx; intrinsic_parameter[4] = ffy; // 초점거리 x y
  intrinsic_parameter[2] = ccx; intrinsic_parameter[5] = ccy; // 주점 x y

}

void campus_pj::on_pushButton_haneye_calibration_disto_clicked()
{
  double kk1, kk2, kk3, tt1, tt2;

  QString k1Text = ui->textEdit_get_k1->toPlainText();
  QString k2Text = ui->textEdit_get_k2->toPlainText();
  QString k3Text = ui->textEdit_get_k3->toPlainText();
  QString t1Text = ui->textEdit_get_t1->toPlainText();
  QString t2Text = ui->textEdit_get_t2->toPlainText();

  kk1 = !k1Text.isEmpty() ? k1Text.toDouble() : discoeffs[0];
  kk2 = !k2Text.isEmpty() ? k2Text.toDouble() : discoeffs[1];
  kk3 = !k3Text.isEmpty() ? k3Text.toDouble() : discoeffs[4];
  tt1 = !t1Text.isEmpty() ? t1Text.toDouble() : discoeffs[2];
  tt2 = !t2Text.isEmpty() ? t2Text.toDouble() : discoeffs[3];

  discoeffs[0] = kk1; discoeffs[1] = kk2; discoeffs[4] = kk3;
  discoeffs[2] = tt1; discoeffs[3] = tt2;

}

void campus_pj::on_pushButton_haneye_calibration_campara_clicked()
{
  color_info_count = 0;
}

void campus_pj::on_pushButton_haneye_calibration_showimage_clicked()
{
  cv::Mat showimage;

  showimage = color_image.clone();
  cv::resize(showimage, showimage, cv::Size(640, 480));
  ui->label_handeye_pic->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
}

void campus_pj::on_pushButton_haneye_calibration_findchess_clicked() //charuco
{
  cv::Mat image_findcorners = color_image_raw.clone();

  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength_chess, dictionary);

  cv::aruco::detectMarkers(image_findcorners, board->dictionary, corners, ids, detectorParams, rejected);
  if (ids.size() > 0)
  {
    cv::aruco::drawDetectedMarkers(image_findcorners, corners);
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::aruco::interpolateCornersCharuco(corners, ids, image_findcorners, board, charucoCorners, charucoIds);
    // if at least one charuco corner detected
    if (charucoIds.size() > 0)
    {
      cv::aruco::drawDetectedCornersCharuco(image_findcorners, charucoCorners, charucoIds, cv::Scalar(255, 255, 0));
      bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, A, distCoeffs, t2c_rvec, t2c_tvec);
      if (valid) cv::drawFrameAxes(image_findcorners, A, distCoeffs, t2c_rvec, t2c_tvec, 100);

      std::vector<cv::Point3f> axesPoints;
      axesPoints.push_back(cv::Point3f(0, 0, 0));
      axesPoints.push_back(cv::Point3f(100, 0, 0));
      axesPoints.push_back(cv::Point3f(0,100, 0));
      axesPoints.push_back(cv::Point3f(0, 0, 100));
      std::vector<cv::Point2f> imagePoints;
      cv::projectPoints(axesPoints, t2c_rvec, t2c_tvec, A, distCoeffs, imagePoints);

      float distance = depth_image.at<float>(imagePoints[0].y, imagePoints[0].x)*1000;

      std::cout << "distance = " << distance << std::endl;


      cv::Point2f center;

      center.x = imagePoints[0].x;
      center.y = imagePoints[0].y;

      cv::putText(image_findcorners, cv::format("(%f, %f)",imagePoints[0].x, imagePoints[0].y), center, cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
      float op_point[3];
      float pixel[2];

      pixel[0] = imagePoints[0].x;
       pixel[1] = imagePoints[0].y;
      rs2_deproject_pixel_to_point(op_point, &RS_camera_info_, pixel, distance);

      std::cout << " origin = " << t2c_tvec << std::endl;
      cv::Mat RR;
      cv::Rodrigues(t2c_rvec,RR);
      t2c_tvec = -RR.inv() * t2c_tvec;

      t2c_tvec.at<float>(0,0) = op_point[0];
      t2c_tvec.at<float>(1,0) = op_point[1];
      t2c_tvec.at<float>(2,0) = op_point[2];

      //cv::Rodrigues(rvec, R);

    }
  }
  cv::resize(image_findcorners, image_findcorners, cv::Size(640, 320));
  ui->label_handeye_pic->setPixmap(QPixmap::fromImage(QImage(image_findcorners.data, image_findcorners.cols, image_findcorners.rows, image_findcorners.step, QImage::Format_RGB888)));
}

void campus_pj::on_pushButton_haneye_calibration_getmatch_clicked()
{

  cv::Mat rvec;
  cv::Rodrigues(t2c_rvec.clone(), rvec);

  t2c_r.push_back(rvec.clone());
  t2c_t.push_back(t2c_tvec.clone());

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv;
  srv.request.space_type = 1;

  QString text_for_append;

  if(srvGetpose.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_pose[i] = srv.response.pos[i];
      }
      ui->textEdit_haneye_calibration_log->append(text_for_append.sprintf(
      " <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",robot_current_pose[0],robot_current_pose[1],robot_current_pose[2],robot_current_pose[3],robot_current_pose[4],robot_current_pose[5]));

      //return (srv.response.success);
  }
  else
  {
      ui->textEdit_haneye_calibration_log->append("fail!");
      ros::shutdown();
     // return -1;
  }

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

  float data[9];
  int k = 0;

  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[k] = srv2.response.rot_matrix[i].data[j] ;
         k++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  cv::Mat rotation_mat(3,3, CV_32FC1, data);

  std::cout << rotation_mat << std::endl;
  //rotation_mat = rotation_mat.inv();*/

  float data_trans[] = {robot_current_pose[0],robot_current_pose[1],robot_current_pose[2]};

  g2b_tvec.at<float>(0,0) = robot_current_pose[0];
  g2b_tvec.at<float>(1,0) = robot_current_pose[1];
  g2b_tvec.at<float>(2,0) = robot_current_pose[2];

  //cv::Mat trans_mat(3,1,CV_32FC1,data_trans);
  //std::cout << "g2b_t_raw(compare)" << std::endl << trans_mat<< std::endl;


  //trans_mat = -rotation_mat.inv() * -rotation_mat * trans_mat;
  //rotation_mat = rotation_mat.inv();
 //trans_mat = -rotation_mat.inv() * trans_mat;



  //rotation_mat.copyTo(g2b_r);
  //trans_mat.copyTo(g2b_t);
  cv::Mat g2b_rvec = rotation_mat.clone();
  //cv::Rodrigues(rotation_mat, g2b_rvec);

  /*g2b_rvec.at<float>(0,0) = floorf(g2b_rvec.at<float>(0,0)*100) / 100;
  g2b_rvec.at<float>(0,1) = floorf(g2b_rvec.at<float>(0,1)*100) / 100;
  g2b_rvec.at<float>(0,2) = floorf(g2b_rvec.at<float>(0,2)*100) / 100;

  g2b_tvec.at<float>(0,0) = floorf(g2b_tvec.at<float>(0,0)*100) / 100;
  g2b_tvec.at<float>(0,1) = floorf(g2b_tvec.at<float>(0,1)*100) / 100;
  g2b_tvec.at<float>(0,2) = floorf(g2b_tvec.at<float>(0,2)*100) / 100;*/

  g2b_r.push_back(g2b_rvec.clone());
  g2b_t.push_back(g2b_tvec.clone());



  cv::Mat test = g2b_tvec.clone();
  //g2b_tvec.at<float>(0,0) = -g2b_tvec.at<float>(0,0) ;

  g2b_tvec = - g2b_rvec * g2b_tvec;
  g2b_tvec = g2b_rvec.inv() * g2b_tvec + test;
  std::cout << "g2b_test = " << std::endl << g2b_tvec << std::endl;





  std::cout << "t2c_r" << std::endl << t2c_r[t2c_r.size()-1]<< std::endl;
  std::cout << "t2c_t" << std::endl << t2c_t[t2c_t.size()-1]<< std::endl;


  std::cout << "g2b_r" << std::endl << g2b_r[g2b_r.size()-1]<< std::endl;
  std::cout << "g2b_t" << std::endl << g2b_t[g2b_t.size()-1]<< std::endl;


  ui->listWidget_haneye_calibration_t2c->addItem(text_for_append.sprintf("Match %d ",match_count));
  match_count++;

  /*cv::calibrateHandEye(
        g2b_r,
        g2b_t,
        t2c_r,
        t2c_t,
        c2g_rvec,
        c2g_tvec,
        cv::CALIB_HAND_EYE_TSAI);
  std::cout << "RESULT!!!" << std::endl;
  std::cout << "c2g_rvec : " << c2g_rvec << ", c2g_tvec : "  << c2g_tvec << std::endl;*/

}

void campus_pj::on_pushButton_haneye_calibration_calculate_clicked()
{

  for(int i=0; i<g2b_r.size() ; i++)
  {
    std::cout << i+1 << " Match ===== " << std::endl;
    std::cout << "t2c_r" << std::endl <<t2c_r[i] << std::endl;
    std::cout << "t2c_t" << std::endl <<t2c_t[i] << std::endl;
    std::cout << "g2b_r" << std::endl <<g2b_r[i] << std::endl;
    std::cout << "g2b_t" << std::endl <<g2b_t[i] << std::endl;
  }
  cv::calibrateHandEye(
          g2b_r,
          g2b_t,
          t2c_r,
          t2c_t,
          c2g_rvec,
          c2g_tvec,cv::CALIB_HAND_EYE_DANIILIDIS);


    std::cout << "===========================" << std::endl;
    std::cout << "RESULT!!!_daniilids" << std::endl;
    std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;

     std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

     std::string filename = "src/campus_pj/config/test1.yaml";
     cv::FileStorage fs(filename, cv::FileStorage::WRITE);
     fs << "R_daniilids" << c2g_rvec;
     fs << "T_daniilids" << c2g_tvec;

     std::string filename1 = "src/campus_pj/config/test2.yaml";
     cv::FileStorage fs1(filename1, cv::FileStorage::WRITE);
     fs1 << "t2c_r" << t2c_r;
     fs1 << "t2c_t" << t2c_t;
     fs1 << "g2b_r" << g2b_r;
     fs1 << "g2b_t" << g2b_t;
     fs1.release();



    cv::calibrateHandEye(
            g2b_r,
            g2b_t,
            t2c_r,
            t2c_t,
            c2g_rvec,
            c2g_tvec,cv::CALIB_HAND_EYE_PARK);


      std::cout << "===========================" << std::endl;
      std::cout << "RESULT!!!_park" << std::endl;
      std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;

      fs << "R_park" << c2g_rvec;
      fs << "T_park" << c2g_tvec;


      std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

      cv::calibrateHandEye(
              g2b_r,
              g2b_t,
              t2c_r,
              t2c_t,
              c2g_rvec,
              c2g_tvec,cv::CALIB_HAND_EYE_TSAI);


        std::cout << "===========================" << std::endl;
        std::cout << "RESULT!!!_TSAI" << std::endl;
        std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
        fs << "R_tsai" << c2g_rvec;
        fs << "T_tsai" << c2g_tvec;


        std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

        cv::calibrateHandEye(
                g2b_r,
                g2b_t,
                t2c_r,
                t2c_t,
                c2g_rvec,
                c2g_tvec,cv::CALIB_HAND_EYE_HORAUD);


          std::cout << "===========================" << std::endl;
          std::cout << "RESULT!!!_horaud" << std::endl;
          std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
          fs << "R_horaud" << c2g_rvec;
          fs << "T_horaud" << c2g_tvec;
          std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

          cv::calibrateHandEye(
                  g2b_r,
                  g2b_t,
                  t2c_r,
                  t2c_t,
                  c2g_rvec,
                  c2g_tvec,cv::CALIB_HAND_EYE_ANDREFF);


            std::cout << "===========================" << std::endl;
            std::cout << "RESULT!!!_andreff" << std::endl;
            std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
            fs << "R_andreff" << c2g_rvec;
            fs << "T_andreff" << c2g_tvec;

                 fs.release();
            std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

}

void campus_pj::on_pushButton_currentPosx_clicked()
{
    ui->stackedWidget->setCurrentIndex(2);
}

void campus_pj::on_pushButton_currentPosx_get_clicked()
{
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetposx = node->serviceClient<dsr_msgs::GetCurrentPosx>("/dsr01m1013/aux_control/get_current_posx");
  dsr_msgs::GetCurrentPosx srv;
  srv.request.ref = 0;

  cv::Matx44d c2t(1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00, -1.00000000e+02,
                    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
  QString text_for_append;

  if(srvGetposx.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_posx[i] = srv.response.task_pos_info[0].data[i];
      }
      float r_rad = robot_current_posx[3] * M_PI / 180.0;
      float p_rad = robot_current_posx[4] * M_PI / 180.0;
      float yw_rad =robot_current_posx[5] * M_PI / 180.0;   


      // base to camera의 변환 행렬 계산
      cv::Vec3d translation(robot_current_posx[0], robot_current_posx[1], robot_current_posx[2]);

      cv::Matx33d Rz1(
          std::cos(r_rad), -std::sin(r_rad), 0,
          std::sin(r_rad), std::cos(r_rad), 0,
          0, 0, 1
      );
      cv::Matx33d Ry(
          std::cos(p_rad), 0, std::sin(p_rad),
          0, 1, 0,
          -std::sin(p_rad), 0, std::cos(p_rad)
      );
      cv::Matx33d Rz2(
          std::cos(yw_rad), -std::sin(yw_rad), 0,
          std::sin(yw_rad), std::cos(yw_rad), 0,
          0, 0, 1
      );

      cv::Matx33d R_t2b = Rz1 * Ry * Rz2;
      cv::Matx44d T_t2b = cv::Matx44d::eye();
      for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
              T_t2b(i, j) = R_t2b(i, j);
          }
          T_t2b(i, 3) = translation(i);
      }

      // end effector to camera의 역변환 행렬
      cv::Matx44d T_end2tool = c2t.inv();

      // end effector의 위치 추출
      cv::Matx44d T_end2base = T_t2b * T_end2tool;
      float xt = T_end2base(0, 3);
      float yt = T_end2base(1, 3);
      float zt = T_end2base(2, 3);

      ui->textEdit_currentPosx_log->append(text_for_append.sprintf(
      " <pos> %7.5f %7.5f %7.5f ",xt,yt,zt));

      //return (srv.response.success);
  }
  else
  {
      ui->textEdit_currentPosx_log->append("fail!");
      ros::shutdown();
     // return -1;
  }

}

void campus_pj::on_pushButton_currentPosx_home_clicked()
{
  QMessageBox mb;

  mb.setText("Are you sure you want to return to home?");
  mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
  mb.setDefaultButton(QMessageBox::Cancel);
  mb.setIcon(QMessageBox::Icon::Warning);

  // mb.move(470, 350);
  
  int ret = mb.exec();

  switch(ret)
  {
  case QMessageBox::Ok :
    ui->stackedWidget->setCurrentIndex(0);
    break;

  case QMessageBox::Cancel:
    break;
  }
}


void campus_pj::on_pushButton_process_start_clicked()
{

QString text;
//text_for_append.sprintf("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);
//ui->textEdit_process_log->append(text_for_append.sprintf("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d"));
 ui->textEdit_process_log->append(text.sprintf("Process Start!"));


 float velx[2] = {0,0};
 float accx[2] = {0,0};
 float joint_home[6] = {90.0, 0.0, 90.0, 0.0, 90.0, 0.0};
 float pos_home[6] = {650, 340, 865, 0, -180.0, 180.0};

 ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
 ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
 dsr_msgs::GetCurrentPose srv;

 srv.request.space_type = 1;

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

 ui->textEdit_process_log->append(text.sprintf("Move Home Position"));
 movej(joint_home,0,0,4.5,0,0,0,0);
 movel(pos_home,velx,accx,4.5,0,0,0,0,0); // move home position
 cv::waitKey(1);

 cv::Mat image_aruco = color_image_raw.clone();

 std::vector<int> markerIds;
 std::vector<int> markerIds_1st; 
 std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
 cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
 cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 

 int current_marker_num;
 int checking_markeids[5] = {-1,-1,-1,-1,-1};
 int current_marker_num_2;
 int checking_markeids_2[5] = {-1,-1,-1,-1,-1};
 

 ////////////////////////////////////
 // 1st world view
 ////////////////////////////////////
 cv::aruco::detectMarkers(image_aruco, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
 // aruco marker definition
 markerIds_1st = markerIds;

 std::vector<cv::Vec3d> rvecs, tvecs; //
 cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, A, distCoeffs, rvecs, tvecs); //

 cv::Mat outputImage = color_image_raw.clone();
 cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

 // markerIds.size() == 5

 cv::Matx44f c2g = {0.9995884550916401, -0.0286509350929972, -0.001429813206316577, -31.81668840797239,
                    0.02858327133778271, 0.9989768060104955, -0.03504764831909967, -99.62247870764079,
                    0.002432498127190477, 0.03499235589904547, 0.9993846196442566, -2.546049086854508,
                    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};

 if(srvGetpose.call(srv))
 {
     for(int i=0; i<6; i++)
     {
       campus_pj::robot_current_pose[i] = srv.response.pos[i];
     }
 }

  float data[9];
  int l = 0;

  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[l] = srv2.response.rot_matrix[i].data[j] ;
         l++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  l = 0;
  
 cv::Matx44f g2b = {
   data[0], data[1], data[2], robot_current_pose[0],
   data[3], data[4], data[5], robot_current_pose[1],
   data[6], data[7], data[8], robot_current_pose[2],
   0, 0, 0, 1};

 cv::Matx44f c2b = {9.99033876e-01 ,-4.03820491e-02, -1.73379364e-02, -32.13200871477923,
   3.98650088e-02, 9.98778398e-01, -2.91974786e-02, -99.3960836718246,
    1.84958104e-02, 2.84780933e-02, 9.99423285e-01, -7.012243399327414,
    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};


   cv::Matx33d convMatx = {-1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0};

  std::vector<cv::Point3f> objectPoints;



 for(int i = 0; i < markerCorners.size(); i++) cv::circle(outputImage, markerCorners[i][3], 2, cv::Scalar(0, 255, 0), 2);


 float distance;
 float distance2;
 float distance3;
 float aruco_2d[2], aruco_3d[3];

 std::vector<cv::Point3f> arucomarkers_camera_3d, arucomarkers_world_3d_1st, arucomarkers_world_3d_2nd, arucomarkers_world_3d_3rd;
 std::vector<cv::Matx44f> arucomarkers_Mat, arucomarkers_Mat2, arucomarkers_Mat_3;
 cv::Point3f arucomarker_3d;

 for(int i = 0 ; i < markerCorners.size() ; i++)
 {
   distance = depth_image.at<float>(markerCorners[i][3].y, markerCorners[i][3].x)*1000;

   aruco_2d[0] = markerCorners[i][3].x;
   aruco_2d[1] = markerCorners[i][3].y;

   rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance);
   cv::putText(outputImage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
  
   arucomarker_3d.x = aruco_3d[0];
   arucomarker_3d.y = aruco_3d[1];
   arucomarker_3d.z = aruco_3d[2];

   arucomarkers_camera_3d.push_back(arucomarker_3d);
 }

 cv::Mat showimage = outputImage.clone();
 cv::resize(showimage, showimage, cv::Size(640, 360));
 ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
 cv::imwrite("camera_3d_aruco.jpg", outputImage);


 cv::Matx41f aruco_cal_in;
 cv::Matx41f aruco_cal_out;
//  cv::Matx33d Rz1, Ry, Rz2;

 cv::Mat outputImage_world_3d = color_image_raw.clone();

 for(int i = 0 ; i < arucomarkers_camera_3d.size() ; i++)
 {
   aruco_cal_in.val[0] = arucomarkers_camera_3d[i].x;
   aruco_cal_in.val[1] = arucomarkers_camera_3d[i].y;
   aruco_cal_in.val[2] = arucomarkers_camera_3d[i].z;
   aruco_cal_in.val[3] = 1;

   aruco_cal_out = g2b * c2g * aruco_cal_in;

   cv::putText(outputImage_world_3d, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.35, cv::Scalar(255,255,225), 0);
   cv::drawFrameAxes(outputImage_world_3d, A, distCoeffs, rvecs[i], tvecs[i],100.0, 1); //
  //  cv::Matx33d Rz1(
  //     std::cos()
  //  );

       QString text_for_append000;

    text_for_append000.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[i], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
    ui->textEdit_process_log->append(text_for_append000);
  
  // here is maaaakrk
  // aruco_cal_out = c2b * g2b * c2g * aruco_cal_in;
   cv::Point3f arucomarker_world_3d;
   arucomarker_world_3d.x = aruco_cal_out.val[0];
   arucomarker_world_3d.y = aruco_cal_out.val[1];
   arucomarker_world_3d.z = aruco_cal_out.val[2];

   cv::Matx33d R_aruco; //
   cv::Matx44f T_aruco_out;//

  // change vector
  //  rvecs[i][0] = -rvecs[i][0];
   rvecs[i][1] = -rvecs[i][1];
   rvecs[i][2] = -rvecs[i][2];

   cv::Rodrigues(rvecs[i],R_aruco);//
   

    cv::Matx44f T_aruco(
        static_cast<float>(R_aruco(0, 0)), static_cast<float>(R_aruco(0, 1)), static_cast<float>(R_aruco(0, 2)), static_cast<float>(tvecs[i](0)),
        static_cast<float>(R_aruco(1, 0)), static_cast<float>(R_aruco(1, 1)), static_cast<float>(R_aruco(1, 2)), static_cast<float>(tvecs[i](1)),
        static_cast<float>(R_aruco(2, 0)), static_cast<float>(R_aruco(2, 1)), static_cast<float>(R_aruco(2, 2)), static_cast<float>(tvecs[i](2)),
        0.0f, 0.0f, 0.0f, 1.0f
    );


   T_aruco_out = g2b * c2g * T_aruco; //

   arucomarkers_world_3d_1st.push_back(arucomarker_world_3d);
   arucomarkers_Mat.push_back(T_aruco_out);//
  
   /*float coordinate_x, coordinate_y, coordinate_z;
   coordinate_x = aruco_cal_out.val[0];
   coordinate_y = aruco_cal_out.val[1];
   coordinate_z = aruco_cal_out.val[2];


   float targetpos[6] = {coordinate_x, coordinate_y, 150, 0, -180.0, 180.0};
   movel(targetpos,velx,accx,4.5,0,0,0,0,0);*/
 }

 showimage = outputImage_world_3d.clone();
 cv::resize(showimage, showimage, cv::Size(640, 360));
 ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

 cv::imwrite("world_3d_aruco.jpg", outputImage_world_3d);


 int index = 0;
 char buf[255];

////////////////////////////////////
// 1st 소시야
////////////////////////////////////
 for(int i = 0 ; i < 5; i++)
 {
   float p_rad, r_rad, yw_rad;
    ////
//    p_rad = std::acos(arucomarkers_Mat[i](2, 2));

//    if(arucomarkers_Mat[i](1, 2) / std::sin(p_rad) > 0){
//     r_rad = std::acos(arucomarkers_Mat[i](0,2)/std::sin(p_rad));
//    } else{
//     r_rad = -std::acos(arucomarkers_Mat[i](0,2)/std::sin(p_rad));
//    }
//    if(arucomarkers_Mat[i](2,1)/std::sin(p_rad) > 0){
//     yw_rad = std::acos(-arucomarkers_Mat[i](2,0)/std::sin(p_rad));
//    } else {
//     yw_rad = std::acos(-arucomarkers_Mat[i](2,0)/std::sin(p_rad));
//    }

//    float targetpos[6] = {arucomarkers_world_3d_1st[i].x, arucomarkers_world_3d_1st[i].y, 150, RADIAN_TO_DEGREE(r_rad), RADIAN_TO_DEGREE(p_rad), RADIAN_TO_DEGREE(yw_rad)};

   float xx, yy, zz, rr, pp, ww;
   xx = arucomarkers_world_3d_1st[i].x; 
   yy = arucomarkers_world_3d_1st[i].y; 
   zz = 250.0 + arucomarkers_world_3d_1st[i].z; 
   rr = 0.0; pp = -180.0; ww = 180.0;



      //  QString text_for_append38;

      // text_for_append38.sprintf("  < before targetpos > %f, %f, %f, %f, %f, %f",xx,yy,zz,rr,pp,ww);
      // ui->textEdit_process_log->append(text_for_append38);

   calculateEnd2Base(xx, yy, zz, rr, pp, ww);

    // QString text_for_append3;

    //   text_for_append3.sprintf("  < changed targetpos > %f, %f, %f, %f, %f, %f",xx,yy,zz,rr,pp,ww);
    //   ui->textEdit_process_log->append(text_for_append3);

   float targetpos[6] = {xx, yy, zz, rr, pp, ww};
   movel(targetpos,velx,accx,4.0,0,0,0,0,0);

    cv::waitKey(1);
    cv::Mat image_marker_1st = color_image_raw.clone();
    cv::aruco::detectMarkers(image_marker_1st, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    std::vector<cv::Vec3d> rvec, tvec; //
    cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, A, distCoeffs, rvec, tvec); //

    // QString text_for_append300;

    //   text_for_append300.sprintf(" < ids  > %d, %d, %d, %d, %d",markerIds[0],markerIds[1],markerIds[2], markerIds[3], markerIds[4]);
    //   ui->textEdit_process_log->append(text_for_append300);



    // here is problem
    // aruco_2d[0] = markerCorners[0][3].x;
    // aruco_2d[1] = markerCorners[0][3].y;
    bool foundDifference = false;

    for(int j = 0; j < 5; j++)
    {
      for(int k = 0; k < 5; k++)
      {
        if (markerIds[j] == checking_markeids[k]) {foundDifference = false; break;}
        if (k == 4 && markerIds[j] != checking_markeids[k] && markerIds_1st[i] == markerIds[j]) 
        {
          foundDifference = true;
          checking_markeids[i] = markerIds[j];
          current_marker_num = j; 
        }        
      }
      if(foundDifference) break;
    }

    // if(i == 0 || i == 1) {aruco_2d[0] = markerCorners[0][3].x; aruco_2d[1] = markerCorners[0][3].y;} 
    // else if(i == 2) {aruco_2d[0] = markerCorners[1][3].x; aruco_2d[1] = markerCorners[1][3].y;} 
    // else {aruco_2d[0] = markerCorners[2][3].x; aruco_2d[1] = markerCorners[2][3].y;} 
    aruco_2d[0] = markerCorners[current_marker_num][3].x;
    aruco_2d[1] = markerCorners[current_marker_num][3].y;


   if(srvGetpose.call(srv))
   {
       for(int i=0; i<6; i++)
       {
         campus_pj::robot_current_pose[i] = srv.response.pos[i];
       }
   }



  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[l] = srv2.response.rot_matrix[i].data[j] ;
         l++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  l = 0;
  
 cv::Matx44f g2b1 = {
   data[0], data[1], data[2], robot_current_pose[0],
   data[3], data[4], data[5], robot_current_pose[1],
   data[6], data[7], data[8], robot_current_pose[2],
   0, 0, 0, 1};


  distance2 = depth_image.at<float>(markerCorners[current_marker_num][3].y, markerCorners[current_marker_num][3].x)*1000;

  rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance2 );
  //  rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, robot_current_pose[2]-4.47);
   /*cv::putText(image_marker_1st, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
   cv::imwrite("aruco_id1_camera.jpg", image_marker_1st);*/

   aruco_cal_in.val[0] = aruco_3d[0];
   aruco_cal_in.val[1] = aruco_3d[1];
   aruco_cal_in.val[2] = aruco_3d[2];
   aruco_cal_in.val[3] = 1;

   aruco_cal_out = g2b1 * c2g * aruco_cal_in;

   cv::Matx33d R_aruco2; //
   cv::Matx33d R_ee2; //
   cv::Matx44f T_aruco_out2;//
  //  cv::Rodrigues(rvecs[i],R_aruco2);// 대시야에서 구한 값이야 이거는

  cv::Rodrigues(rvec[current_marker_num],R_aruco2);//

  // R_ee2 = convMatx * R_aruco2;
  
  rvec[current_marker_num][1] = -rvec[current_marker_num][1];
  rvec[current_marker_num][2] = -rvec[current_marker_num][2];
  cv::Rodrigues(rvec[current_marker_num],R_ee2);//
  cv::putText(image_marker_1st, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[current_marker_num][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
  cv::circle(image_marker_1st, markerCorners[current_marker_num][3], 2, cv::Scalar(0, 255, 0), 2);
  // QString text_for_append53;

  //   text_for_append53.sprintf("  < tvec > %f, %f, %f ", tvec[0][0], tvec[0][1], tvec[0][2]);
  //   ui->textEdit_process_log->append(text_for_append53);
  QString text_for_append001;

  text_for_append001.sprintf("\n id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[current_marker_num], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
  ui->textEdit_process_log->append(text_for_append001);

  cv::drawFrameAxes(image_marker_1st, A, distCoeffs, rvec[current_marker_num], tvec[current_marker_num],100.0, 1);

   
   sprintf(buf, "marker_id%06d_world_1st.jpg",index);
   cv::imwrite(buf, image_marker_1st);



  //  aruco_cal_out = c2b * g2b1 * c2g * aruco_cal_in;

   /*cv::Point3f arucomarker_world_3d;
   arucomarker_world_3d.x = aruco_cal_out.val[0];
   arucomarker_world_3d.y = aruco_cal_out.val[1];
   arucomarker_world_3d.z = aruco_cal_out.val[2];*/

   float aruco_2d_center[2] = {0,0}, aruco_3d_center[3]= {0,0,0};
   for(int j = 0; j < markerCorners[current_marker_num].size(); j++)
   {
     aruco_2d_center[0] += markerCorners[current_marker_num][j].x;
     aruco_2d_center[1] += markerCorners[current_marker_num][j].y;
   }

   aruco_2d_center[0] = aruco_2d_center[0] / 4.;
   aruco_2d_center[1] = aruco_2d_center[1] / 4.;


   rs2_deproject_pixel_to_point(aruco_3d_center, &RS_camera_info_, aruco_2d_center, distance3);

   std::cout << aruco_3d_center[0] << std::endl;
   std::cout << aruco_3d_center[1] << std::endl;


  //  cv::Matx33d R_aruco2, R_aruco3; //
  //  cv::Matx44f T_aruco_out2;//
  //  cv::Rodrigues(rvecs[i],R_aruco3);//

  //  R_aruco2 = R_aruco3.inv();
  // QString text_for_append003;

  // text_for_append003.sprintf("id %d : center = (%.5lf, %.5lf, %.5lf) ",markerIds[current_marker_num], aruco_3d_center[0], aruco_3d_center[1], aruco_3d_center[2] );
  // ui->textEdit_process_log->append(text_for_append003);


      showimage = image_marker_1st.clone();
 cv::resize(showimage, showimage, cv::Size(640, 360));
 ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));


  cv::Point3f arucomarker_world_3d_2nd;
   arucomarker_world_3d_2nd.x = aruco_cal_out.val[0];
   arucomarker_world_3d_2nd.y = aruco_cal_out.val[1];
   arucomarker_world_3d_2nd.z = aruco_cal_out.val[2];
  
  //  arucomarkers_world_3d_2nd.push_back(arucomarker_world_3d_2nd);
  //  arucomarkers_Mat.push_back(T_aruco_out);//
  cv::Point3d targetPosition(arucomarker_world_3d_2nd.x, arucomarker_world_3d_2nd.y, arucomarker_world_3d_2nd.z); 
  cv::Point3d endEffectorPosition = cv::Point3d(0.0, 0.0, -200.0);//mm
    cv::Matx31d rotatedVector = R_ee2 * cv::Matx31d(endEffectorPosition.x, endEffectorPosition.y, endEffectorPosition.z);
    cv::Point3d finalPosition(rotatedVector(0, 0) , rotatedVector(1, 0) , rotatedVector(2, 0) );

   
    cv::Matx44f T_aruco2(
        static_cast<float>(R_ee2(0, 0)), static_cast<float>(R_ee2(0, 1)), static_cast<float>(R_ee2(0, 2)), finalPosition.x + arucomarker_world_3d_2nd.x + aruco_3d_center[0],
        static_cast<float>(R_ee2(1, 0)), static_cast<float>(R_ee2(1, 1)), static_cast<float>(R_ee2(1, 2)), finalPosition.y + arucomarker_world_3d_2nd.y - aruco_3d_center[1],
        static_cast<float>(R_ee2(2, 0)), static_cast<float>(R_ee2(2, 1)), static_cast<float>(R_ee2(2, 2)), finalPosition.z + arucomarker_world_3d_2nd.z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
    
    

  //  QString text_for_append2;

  //     //  text_for_append2.sprintf("  <T_aruco2 > %f %f %f %f\n %f %f %f %f\n %f %f %f %f",R_aruco(0, 0), R_aruco(0, 1), R_aruco(0, 2),tvecs[i](0), R_aruco(1, 0), R_aruco(1,1), R_aruco(1,2),tvecs[i](1),R_aruco(2,0), R_aruco(2,1), R_aruco(2,2),tvecs[i](2));
  //     text_for_append2.sprintf("  <T_aruco2 > %f %f %f %f\n %f %f %f %f\n %f %f %f %f",T_aruco2(0, 0), T_aruco2(0, 1), T_aruco2(0, 2),T_aruco2(0, 3), T_aruco2(1, 0), T_aruco2(1,1), T_aruco2(1,2),T_aruco2(1, 3),T_aruco2(2,0), T_aruco2(2,1), T_aruco2(2,2),T_aruco2(2, 3));

  //      ui->textEdit_process_log->append(text_for_append2);

 
     float xxx, yyy, zzz, rrr, ppp, www;

    //  xxx = 0.0; yyy = 0.0; zzz = 0.0; rrr = 0.0; ppp = 0.0; www = 0.0;
  //  xxx = robot_current_pose[0] + aruco_3d_center[0]; 
  //  yyy = robot_current_pose[1] - aruco_3d_center[1]; 
  //  zzz = robot_current_pose[2]; 




  //  calculateEnd2Base2(xxx, yyy, zzz, rrr, ppp, www, T_aruco2);

    cv::Matx44f T_end2camera2 = c2g.inv();

    // end effector의 위치 추출
    cv::Matx44f T_end2base2 = T_aruco2 * T_end2camera2;
    xxx = T_end2base2(0, 3);
    yyy = T_end2base2(1, 3);
    zzz = T_end2base2(2, 3);// + 250;

    // end effector의 방향 추출
    cv::Matx33f R_end2base2;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base2(i, j) = T_end2base2(i, j);
        }
    }
    
    p_rad = std::acos(R_end2base2(2, 2));

    if (R_end2base2(1, 2) / std::sin(p_rad) > 0) {
        r_rad = std::acos(R_end2base2(0, 2) / std::sin(p_rad));
    } else {
        r_rad = -std::acos(R_end2base2(0, 2) / std::sin(p_rad));
    }
    if (R_end2base2(2, 1) / std::sin(p_rad) > 0) {
        yw_rad = std::acos(-R_end2base2(2, 0) / std::sin(p_rad));
    } else {
        yw_rad = -std::acos(-R_end2base2(2, 0) / std::sin(p_rad));
    }

    rrr = r_rad * 180.0 / M_PI;
    ppp = p_rad * 180.0 / M_PI;
    www = yw_rad * 180.0 / M_PI;
    

    // QString text_for_append5;

    //   text_for_append5.sprintf("  <chaangeddd targetpos__> %f, %f, %f, %f, %f, %f",xxx,yyy,zzz,rrr,ppp,www);
    //   ui->textEdit_process_log->append(text_for_append5);

  // calculateEnd2Base(xxx, yyy, zzz, rrr, ppp, www);
  
  //     QString text_for_append6;

  //     text_for_append6.sprintf("  <chaangeddd targetpos__> %f, %f, %f, %f, %f, %f",xxx,yyy,zzz,rrr,ppp,www);
  //     ui->textEdit_process_log->append(text_for_append6);
   float targetpos1[6] = {xxx, yyy, zzz, rrr, ppp, www};
  //  movel(targetpos,velx,accx,4.5,0,0,0,0,0);

  //  float targetpos1[6] = {robot_current_pose[0] + aruco_3d_center[0], robot_current_pose[1] - aruco_3d_center[1], robot_current_pose[2], rr, pp, ww};

  // QString text_for_append4;

  //   text_for_append4.sprintf("  <aruco_3d_center x, y> %f %f ",aruco_3d_center[0],aruco_3d_center[1]);
  //   ui->textEdit_process_log->append(text_for_append4);
   movel(targetpos1,velx,accx,4.5,0,0,0,0,0);
   cv::waitKey(1);

   cv::Mat image_marker_2nd = color_image_raw.clone();
   cv::aruco::detectMarkers(image_marker_2nd, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

   


  bool foundDifference_2 = false;

  for(int j = 0; j < 5; j++)
  {
    for(int k = 0; k < 5; k++)
    {
      if (markerIds[j] == checking_markeids_2[k]) {foundDifference_2 = false; break;}
      if ( k == 4 && markerIds[j] != checking_markeids_2[k] && markerIds_1st[i] == markerIds[j]) 
      {
        foundDifference_2 = true;
        checking_markeids_2[i] = markerIds[j];
        current_marker_num_2 = j; 
      }        
    }
    if(foundDifference_2) break;
  }
    distance3 = depth_image.at<float>(markerCorners[current_marker_num_2][3].y, markerCorners[current_marker_num_2][3].x)*1000;
   aruco_2d[0] = markerCorners[current_marker_num_2][3].x;
   aruco_2d[1] = markerCorners[current_marker_num_2][3].y;

   if(srvGetpose.call(srv))
   {
       for(int i=0; i<6; i++)
       {
         campus_pj::robot_current_pose[i] = srv.response.pos[i];
       }
   }



  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[l] = srv2.response.rot_matrix[i].data[j] ;
         l++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  l = 0;
  
 cv::Matx44f g2b2 = {
   data[0], data[1], data[2], robot_current_pose[0],
   data[3], data[4], data[5], robot_current_pose[1],
   data[6], data[7], data[8], robot_current_pose[2],
   0, 0, 0, 1};

   rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance3);
   /*cv::putText(image_marker_1st, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
   cv::imwrite("aruco_id1_camera.jpg", image_marker_1st);*/

   aruco_cal_in.val[0] = aruco_3d[0];
   aruco_cal_in.val[1] = aruco_3d[1];
   aruco_cal_in.val[2] = aruco_3d[2];
   aruco_cal_in.val[3] = 1;

   aruco_cal_out = g2b2 * c2g * aruco_cal_in;

   cv::putText(image_marker_2nd, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);

   sprintf(buf, "marker_id%06d_world_2nd.jpg",index);
   cv::imwrite(buf, image_marker_2nd);
    cv::waitKey(1);
  QString text_for_append002;

  text_for_append002.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[current_marker_num_2], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
  ui->textEdit_process_log->append(text_for_append002);
    /*float coordinate_x, coordinate_y, coordinate_z;

    coordinate_x = aruco_cal_out.val[0];
    coordinate_y = aruco_cal_out.val[1];
    coordinate_z = aruco_cal_out.val[2];

    float targetpos3[6] = {coordinate_x, coordinate_y, 150, 0, -180.0, 180.0};
    movel(targetpos3,velx,accx,2,0,0,0,0,0);

    float targetpos4[6] = {coordinate_x, coordinate_y, 111.5, 0, -180.0, 180.0};
    movel(targetpos4,velx,accx,1,0,0,0,0,0);

    cv::waitKey(1);

    float targetpos5[6] = {coordinate_x, coordinate_y, 150, 0, -180.0, 180.0};
    movel(targetpos5,velx,accx,1,0,0,0,0,0);*/

   index++;
 }
}


void campus_pj::on_pushButton_process_start_plat_clicked()
{


QString text;
//text_for_append.sprintf("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);
//ui->textEdit_process_log->append(text_for_append.sprintf("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d"));
 ui->textEdit_process_log->append(text.sprintf("Process Start!"));


 float velx[2] = {0,0};
 float accx[2] = {0,0};
 float joint_home[6] = {90.0, 0.0, 90.0, 0.0, 90.0, 0.0};
 float pos_home[6] = {650, 340, 865, 0, -180.0, 180.0};

 ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
 ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
 dsr_msgs::GetCurrentPose srv;

 srv.request.space_type = 1;

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

 ui->textEdit_process_log->append(text.sprintf("Move Home Position"));
 movej(joint_home,0,0,4.5,0,0,0,0);
 movel(pos_home,velx,accx,4.5,0,0,0,0,0); // move home position
 cv::waitKey(1);

 cv::Mat image_aruco = color_image_raw.clone();

 std::vector<int> markerIds;
 std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
 cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
 cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 

 int current_marker_num;
 int checking_markeids[5] = {-1,-1,-1,-1,-1};
 int current_marker_num_2;
 int checking_markeids_2[5] = {-1,-1,-1,-1,-1};
 

 ////////////////////////////////////
 // 1st world view
 ////////////////////////////////////
 cv::aruco::detectMarkers(image_aruco, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
 // aruco marker definition

 std::vector<cv::Vec3d> rvecs, tvecs; //
 cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, A, distCoeffs, rvecs, tvecs); //

 cv::Mat outputImage = color_image_raw.clone();
 cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

 // markerIds.size() == 5

 cv::Matx44f c2g = {0.9995884550916401, -0.0286509350929972, -0.001429813206316577, -31.81668840797239,
                    0.02858327133778271, 0.9989768060104955, -0.03504764831909967, -99.62247870764079,
                    0.002432498127190477, 0.03499235589904547, 0.9993846196442566, -2.546049086854508,
                    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};

 if(srvGetpose.call(srv))
 {
     for(int i=0; i<6; i++)
     {
       campus_pj::robot_current_pose[i] = srv.response.pos[i];
     }
 }

  float data[9];
  int l = 0;

  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[l] = srv2.response.rot_matrix[i].data[j] ;
         l++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  l = 0;
  
 cv::Matx44f g2b = {
   data[0], data[1], data[2], robot_current_pose[0],
   data[3], data[4], data[5], robot_current_pose[1],
   data[6], data[7], data[8], robot_current_pose[2],
   0, 0, 0, 1};

  std::vector<cv::Point3f> objectPoints;



 for(int i = 0; i < markerCorners.size(); i++) cv::circle(outputImage, markerCorners[i][3], 2, cv::Scalar(0, 255, 0), 2);


 float distance;
 float distance2;
 float distance3;
 float aruco_2d[2], aruco_3d[3];

 std::vector<cv::Point3f> arucomarkers_camera_3d, arucomarkers_world_3d_1st, arucomarkers_world_3d_2nd, arucomarkers_world_3d_3rd;
 std::vector<cv::Matx44f> arucomarkers_Mat, arucomarkers_Mat2, arucomarkers_Mat_3;
 cv::Point3f arucomarker_3d;

 for(int i = 0 ; i < markerCorners.size() ; i++)
 {
   distance = depth_image.at<float>(markerCorners[i][3].y, markerCorners[i][3].x)*1000;

   aruco_2d[0] = markerCorners[i][3].x;
   aruco_2d[1] = markerCorners[i][3].y;

   rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance);
   cv::putText(outputImage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
  
   arucomarker_3d.x = aruco_3d[0];
   arucomarker_3d.y = aruco_3d[1];
   arucomarker_3d.z = aruco_3d[2];

   arucomarkers_camera_3d.push_back(arucomarker_3d);
 }

 cv::Mat showimage = outputImage.clone();
 cv::resize(showimage, showimage, cv::Size(640, 360));
 ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
 cv::imwrite("camera_3d_aruco.jpg", outputImage);


 cv::Matx41f aruco_cal_in;
 cv::Matx41f aruco_cal_out;
//  cv::Matx33d Rz1, Ry, Rz2;

 cv::Mat outputImage_world_3d = color_image_raw.clone();

 for(int i = 0 ; i < arucomarkers_camera_3d.size() ; i++)
 {
   aruco_cal_in.val[0] = arucomarkers_camera_3d[i].x;
   aruco_cal_in.val[1] = arucomarkers_camera_3d[i].y;
   aruco_cal_in.val[2] = arucomarkers_camera_3d[i].z;
   aruco_cal_in.val[3] = 1;

   aruco_cal_out = g2b * c2g * aruco_cal_in;

   cv::putText(outputImage_world_3d, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.35, cv::Scalar(255,255,225), 0);
   cv::drawFrameAxes(outputImage_world_3d, A, distCoeffs, rvecs[i], tvecs[i],100.0, 1); //
  //  cv::Matx33d Rz1(
  //     std::cos()
  //  );

    QString text_for_append000;

    text_for_append000.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[i], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
    ui->textEdit_process_log->append(text_for_append000);
  
  // here is maaaakrk
   cv::Point3f arucomarker_world_3d;
   arucomarker_world_3d.x = aruco_cal_out.val[0];
   arucomarker_world_3d.y = aruco_cal_out.val[1];
   arucomarker_world_3d.z = aruco_cal_out.val[2];

   cv::Matx33d R_aruco; //
   cv::Matx44f T_aruco_out;//

  // change vector
  //  rvecs[i][0] = -rvecs[i][0];
   rvecs[i][1] = -rvecs[i][1];
   rvecs[i][2] = -rvecs[i][2];

   cv::Rodrigues(rvecs[i],R_aruco);//
   

    cv::Matx44f T_aruco(
        static_cast<float>(R_aruco(0, 0)), static_cast<float>(R_aruco(0, 1)), static_cast<float>(R_aruco(0, 2)), static_cast<float>(tvecs[i](0)),
        static_cast<float>(R_aruco(1, 0)), static_cast<float>(R_aruco(1, 1)), static_cast<float>(R_aruco(1, 2)), static_cast<float>(tvecs[i](1)),
        static_cast<float>(R_aruco(2, 0)), static_cast<float>(R_aruco(2, 1)), static_cast<float>(R_aruco(2, 2)), static_cast<float>(tvecs[i](2)),
        0.0f, 0.0f, 0.0f, 1.0f
    );


   T_aruco_out = g2b * c2g * T_aruco; //

   arucomarkers_world_3d_1st.push_back(arucomarker_world_3d);
   arucomarkers_Mat.push_back(T_aruco_out);//
  
   /*float coordinate_x, coordinate_y, coordinate_z;
   coordinate_x = aruco_cal_out.val[0];
   coordinate_y = aruco_cal_out.val[1];
   coordinate_z = aruco_cal_out.val[2];


   float targetpos[6] = {coordinate_x, coordinate_y, 150, 0, -180.0, 180.0};
   movel(targetpos,velx,accx,4.5,0,0,0,0,0);*/
 }

 showimage = outputImage_world_3d.clone();
 cv::resize(showimage, showimage, cv::Size(640, 360));
 ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

 cv::imwrite("world_3d_aruco.jpg", outputImage_world_3d);


 int index = 0;
 char buf[255];

////////////////////////////////////
// 1st 소시야
////////////////////////////////////
 for(int i = 0 ; i < 5; i++)
 {
   float p_rad, r_rad, yw_rad;


   float xx, yy, zz, rr, pp, ww;
   xx = arucomarkers_world_3d_1st[i].x; 
   yy = arucomarkers_world_3d_1st[i].y; 
   zz = 250.0 + arucomarkers_world_3d_1st[i].z; 
   rr = 0.0; pp = -180.0; ww = 180.0;


   calculateEnd2Base(xx, yy, zz, rr, pp, ww);



   float targetpos[6] = {xx, yy, zz, rr, pp, ww};
   movel(targetpos,velx,accx,4.0,0,0,0,0,0);

    cv::waitKey(1);
    cv::Mat image_marker_1st = color_image_raw.clone();
    cv::aruco::detectMarkers(image_marker_1st, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    std::vector<cv::Vec3d> rvec, tvec; //
    cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, A, distCoeffs, rvec, tvec); //

    // QString text_for_append300;

    //   text_for_append300.sprintf(" < ids  > %d, %d, %d, %d, %d",markerIds[0],markerIds[1],markerIds[2], markerIds[3], markerIds[4]);
    //   ui->textEdit_process_log->append(text_for_append300);



    // here is problem
    // aruco_2d[0] = markerCorners[0][3].x;
    // aruco_2d[1] = markerCorners[0][3].y;
    bool foundDifference = false;

    for(int j = 0; j < 5; j++)
    {
      for(int k = 0; k < 5; k++)
      {
        if (markerIds[j] == checking_markeids[k]) {foundDifference = false; break;}
        if ( k == 4 && markerIds[j] != checking_markeids[k]) 
        {
          foundDifference = true;
          checking_markeids[i] = markerIds[j];
          current_marker_num = j; 
        }        
      }
      if(foundDifference) break;
    }

    // if(i == 0 || i == 1) {aruco_2d[0] = markerCorners[0][3].x; aruco_2d[1] = markerCorners[0][3].y;} 
    // else if(i == 2) {aruco_2d[0] = markerCorners[1][3].x; aruco_2d[1] = markerCorners[1][3].y;} 
    // else {aruco_2d[0] = markerCorners[2][3].x; aruco_2d[1] = markerCorners[2][3].y;} 
    aruco_2d[0] = markerCorners[current_marker_num][3].x;
    aruco_2d[1] = markerCorners[current_marker_num][3].y;


   if(srvGetpose.call(srv))
   {
       for(int i=0; i<6; i++)
       {
         campus_pj::robot_current_pose[i] = srv.response.pos[i];
       }
   }



  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[l] = srv2.response.rot_matrix[i].data[j] ;
         l++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  l = 0;
  
 cv::Matx44f g2b1 = {
   data[0], data[1], data[2], robot_current_pose[0],
   data[3], data[4], data[5], robot_current_pose[1],
   data[6], data[7], data[8], robot_current_pose[2],
   0, 0, 0, 1};


  distance2 = depth_image.at<float>(markerCorners[current_marker_num][3].y, markerCorners[current_marker_num][3].x)*1000;

  rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance2 );
  //  rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, robot_current_pose[2]-4.47);
   /*cv::putText(image_marker_1st, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
   cv::imwrite("aruco_id1_camera.jpg", image_marker_1st);*/

   aruco_cal_in.val[0] = aruco_3d[0];
   aruco_cal_in.val[1] = aruco_3d[1];
   aruco_cal_in.val[2] = aruco_3d[2];
   aruco_cal_in.val[3] = 1;

   aruco_cal_out = g2b1 * c2g * aruco_cal_in;

   cv::Matx33d R_aruco2; //
   cv::Matx33d R_ee2; //
   cv::Matx44f T_aruco_out2;//
  //  cv::Rodrigues(rvecs[i],R_aruco2);// 대시야에서 구한 값이야 이거는

  cv::Rodrigues(rvec[current_marker_num],R_aruco2);//
  
  rvec[current_marker_num][1] = -rvec[current_marker_num][1];
  rvec[current_marker_num][2] = -rvec[current_marker_num][2];
  cv::Rodrigues(rvec[current_marker_num],R_ee2);//
  cv::putText(image_marker_1st, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[current_marker_num][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
  cv::circle(image_marker_1st, markerCorners[current_marker_num][3], 2, cv::Scalar(0, 255, 0), 2);
  // QString text_for_append53;

  //   text_for_append53.sprintf("  < tvec > %f, %f, %f ", tvec[0][0], tvec[0][1], tvec[0][2]);
  //   ui->textEdit_process_log->append(text_for_append53);
  QString text_for_append001;

  text_for_append001.sprintf("\n id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[current_marker_num], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
  ui->textEdit_process_log->append(text_for_append001);

  cv::drawFrameAxes(image_marker_1st, A, distCoeffs, rvec[current_marker_num], tvec[current_marker_num],100.0, 1);

   
   sprintf(buf, "marker_id%06d_world_1st.jpg",index);
   cv::imwrite(buf, image_marker_1st);


   /*cv::Point3f arucomarker_world_3d;
   arucomarker_world_3d.x = aruco_cal_out.val[0];
   arucomarker_world_3d.y = aruco_cal_out.val[1];
   arucomarker_world_3d.z = aruco_cal_out.val[2];*/

   float aruco_2d_center[2] = {0,0}, aruco_3d_center[3]= {0,0,0};
   for(int j = 0; j < markerCorners[current_marker_num].size(); j++)
   {
     aruco_2d_center[0] += markerCorners[current_marker_num][j].x;
     aruco_2d_center[1] += markerCorners[current_marker_num][j].y;
   }

   aruco_2d_center[0] = aruco_2d_center[0] / 4.;
   aruco_2d_center[1] = aruco_2d_center[1] / 4.;


   rs2_deproject_pixel_to_point(aruco_3d_center, &RS_camera_info_, aruco_2d_center, distance3);

   std::cout << aruco_3d_center[0] << std::endl;
   std::cout << aruco_3d_center[1] << std::endl;

  // QString text_for_append003;

  // text_for_append003.sprintf("id %d : center = (%.5lf, %.5lf, %.5lf) ",markerIds[current_marker_num], aruco_3d_center[0], aruco_3d_center[1], aruco_3d_center[2] );
  // ui->textEdit_process_log->append(text_for_append003);


      showimage = image_marker_1st.clone();
 cv::resize(showimage, showimage, cv::Size(640, 360));
 ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));


  cv::Point3f arucomarker_world_3d_2nd;
   arucomarker_world_3d_2nd.x = aruco_cal_out.val[0];
   arucomarker_world_3d_2nd.y = aruco_cal_out.val[1];
   arucomarker_world_3d_2nd.z = aruco_cal_out.val[2];


   float targetpos1[6] = {robot_current_pose[0] + aruco_3d_center[0], robot_current_pose[1] - aruco_3d_center[1], robot_current_pose[2], rr, pp, ww};

  // QString text_for_append4;

  //   text_for_append4.sprintf("  <aruco_3d_center x, y> %f %f ",aruco_3d_center[0],aruco_3d_center[1]);
  //   ui->textEdit_process_log->append(text_for_append4);
   movel(targetpos1,velx,accx,4.5,0,0,0,0,0);
   cv::waitKey(1);

   cv::Mat image_marker_2nd = color_image_raw.clone();
   cv::aruco::detectMarkers(image_marker_2nd, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

   


  bool foundDifference_2 = false;

  for(int j = 0; j < 5; j++)
  {
    for(int k = 0; k < 5; k++)
    {
      if (markerIds[j] == checking_markeids_2[k]) {foundDifference_2 = false; break;}
      if ( k == 4 && markerIds[j] != checking_markeids_2[k]) 
      {
        foundDifference_2 = true;
        checking_markeids_2[i] = markerIds[j];
        current_marker_num_2 = j; 
      }        
    }
    if(foundDifference_2) break;
  }
    distance3 = depth_image.at<float>(markerCorners[current_marker_num_2][3].y, markerCorners[current_marker_num_2][3].x)*1000;
   aruco_2d[0] = markerCorners[current_marker_num_2][3].x;
   aruco_2d[1] = markerCorners[current_marker_num_2][3].y;

   if(srvGetpose.call(srv))
   {
       for(int i=0; i<6; i++)
       {
         campus_pj::robot_current_pose[i] = srv.response.pos[i];
       }
   }



  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[l] = srv2.response.rot_matrix[i].data[j] ;
         l++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  l = 0;
  
 cv::Matx44f g2b2 = {
   data[0], data[1], data[2], robot_current_pose[0],
   data[3], data[4], data[5], robot_current_pose[1],
   data[6], data[7], data[8], robot_current_pose[2],
   0, 0, 0, 1};

   rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance3);
   /*cv::putText(image_marker_1st, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
   cv::imwrite("aruco_id1_camera.jpg", image_marker_1st);*/

   aruco_cal_in.val[0] = aruco_3d[0];
   aruco_cal_in.val[1] = aruco_3d[1];
   aruco_cal_in.val[2] = aruco_3d[2];
   aruco_cal_in.val[3] = 1;

   aruco_cal_out = g2b2 * c2g * aruco_cal_in;

   cv::putText(image_marker_2nd, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);

   sprintf(buf, "marker_id%06d_world_2nd.jpg",index);
   cv::imwrite(buf, image_marker_2nd);
    cv::waitKey(1);
  QString text_for_append002;

  text_for_append002.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[current_marker_num_2], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
  ui->textEdit_process_log->append(text_for_append002);
    /*float coordinate_x, coordinate_y, coordinate_z;

    coordinate_x = aruco_cal_out.val[0];
    coordinate_y = aruco_cal_out.val[1];
    coordinate_z = aruco_cal_out.val[2];

    float targetpos3[6] = {coordinate_x, coordinate_y, 150, 0, -180.0, 180.0};
    movel(targetpos3,velx,accx,2,0,0,0,0,0);

    float targetpos4[6] = {coordinate_x, coordinate_y, 111.5, 0, -180.0, 180.0};
    movel(targetpos4,velx,accx,1,0,0,0,0,0);

    cv::waitKey(1);

    float targetpos5[6] = {coordinate_x, coordinate_y, 150, 0, -180.0, 180.0};
    movel(targetpos5,velx,accx,1,0,0,0,0,0);*/

   index++;
 }
}
