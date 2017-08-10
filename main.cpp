#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;
int array_index = 0;

vector<vector<Point2f> >corners_l_array;
bool ChessboardStable(vector<Point2f>corners_l){
  if (corners_l_array.size() < 10){
    corners_l_array.push_back(corners_l);
    return false;
  }
  else{
    corners_l_array[array_index % 10] = corners_l;
    array_index++;
    double error = 0.0;
    for (uint i = 0; i < corners_l_array.size(); i++){
      for (uint j = 0; j < corners_l_array[i].size(); j++){
        error += abs(corners_l[j].x - corners_l_array[i][j].x) + abs(corners_l[j].y - corners_l_array[i][j].y);
      }
    }
    if (error < 500)
    {
      corners_l_array.clear();
      array_index = 0;
      return true;
    }
    else
      return false;
  }
}
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    int n_boards=20;
//    float image_sf=0.5f;
    const float squareSize = 21.5f;  // Set this to your actual square size
//    float delay=1.f;
    int board_w=9;
    int board_h=7;

    int board_n=board_w*board_h;
    Size board_sz=Size(board_w,board_h);

    VideoCapture capture(0);
    if(!capture.isOpened()){
        cout<<"\nCould not open the camera\n";
        return -1;
    }
    //分配存储空间
    vector<vector<Point2f> > image_points;
    vector<vector<Point3f> > object_points;
    //获取角点视图
    Size image_size;

    while(image_points.size()<(size_t)n_boards){
        Mat image0,image;
        capture>>image0;
        image0.copyTo(image);
        image_size=image0.size();
//        resize(image0,image,Size(),image_sf,image_sf);

        //寻找board
        vector<Point2f> corners;
        bool found=findChessboardCorners(image0,board_sz,corners,
                                         CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        //存储
        if(found&&ChessboardStable(corners)){
            //绘制
            drawChessboardCorners(image,board_sz,corners,found);

            image+=100;

//            Mat mcorners(corners);
//            mcorners*=(1./image_sf);
            image_points.push_back(corners);
            object_points.push_back(vector<Point3f>());
            vector<Point3f>& opts=object_points.back();
            opts.resize(board_n);
            for(int j=0;j<board_n;j++){
                opts[j]=Point3f((float)((j/board_w)*squareSize), (float)((j%board_w)*squareSize), 0.f);
            }
            cout<<"Collected our "<<(int)image_points.size()
               <<" of "<<n_boards<<" needed chessboard images\n"<<endl;
            imshow("Calibration",image);
            char c = (char)waitKey(500);
            if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
              exit(-1);
        }else{
            drawChessboardCorners(image,board_sz,corners,found);
        }
        imshow("Calibration",image);
        if((waitKey(30)&255)==27){
            return -1;
        }
    }
    destroyWindow("Calibration");
    cout<<"\n\n*** CALIBRATING THE CAMERA...\n"<<endl;
    //校准
    Mat intrinsic_matrix,distortion_coeffs;
    double err=calibrateCamera(object_points,image_points,image_size,intrinsic_matrix,distortion_coeffs,
                               noArray(),noArray(),CALIB_ZERO_TANGENT_DIST|CALIB_FIX_PRINCIPAL_POINT);
    //保存内参矩阵
    cout<<"*** DONE!\n\nReprojection error is " << err <<
          "\nStoring Intrinsics.xml and Distortions.xml files\n\n";
    cv::FileStorage fs( "intrinsics.xml", FileStorage::WRITE );
    fs << "image_width" << image_size.width << "image_height" << image_size.height
    <<"camera_matrix" << intrinsic_matrix << "distortion_coefficients"
    << distortion_coeffs;
    fs.release();
    // EXAMPLE OF LOADING THESE MATRICES BACK IN:
    fs.open( "intrinsics.xml", cv::FileStorage::READ );
    cout << "\nimage width: " << (int)fs["image_width"];
    cout << "\nimage height: " << (int)fs["image_height"];

    Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
    fs["camera_matrix"] >> intrinsic_matrix_loaded;
    fs["distortion_coefficients"] >> distortion_coeffs_loaded;
    cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
    cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << endl;
    //创建矫正映射
    Mat map1,map2;
    initUndistortRectifyMap(intrinsic_matrix_loaded,distortion_coeffs_loaded,Mat(),
                            intrinsic_matrix_loaded,image_size,CV_16SC2,map1,map2);
    //显示
    for(;;) {
        Mat image, image0;
        capture >> image0;
        if( image0.empty() ) break;
        remap(image0,image,map1,map2,INTER_LINEAR,BORDER_CONSTANT,Scalar());
        cv::imshow("Undistorted", image);
        if((cv::waitKey(30) & 255) == 27) break;
    }
    return a.exec();
}
