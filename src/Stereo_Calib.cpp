//
// Created by qiaochunxiao on 2020/7/27.
//

#include "Stereo_Calib.h"

Stereo_Calib::Stereo_Calib(std::string extrinsics,std::string intrinsics,
                           int img_width,int img_height,
                           std::string algorithm)
                          :S_extrinsics(extrinsics),S_intrinsics(intrinsics),
                           S_img_width(img_width),S_img_height(img_height),
                           S_algorithm(algorithm)
{
    Stereo_Calib::init();
}

Stereo_Calib::~Stereo_Calib()
{
    ;
}

void Stereo_Calib::print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|sgbm3way] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
           "[--no-display] [-o=<disparity_image>] [-p=<point_cloud_file>]\n");
}
void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;//fabs(x)返回x的绝对值
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

int Stereo_Calib::init()
{
    alg = STEREO_SGBM;//int alg = STEREO_SGBM;
    int SADWindowSize, numberOfDisparities;
    bool no_display;
    float scale;

    if(S_help)
    {
        print_help();
        return 0;
    }
    if (!S_algorithm.empty())
    {
        std::string _alg = S_algorithm;
        cout<<"alg"<<_alg<<endl;
        alg = _alg == "bm" ? STEREO_BM :
              _alg == "sgbm" ? STEREO_SGBM :
              _alg == "hh" ? STEREO_HH :
              _alg == "var" ? STEREO_VAR :
              _alg == "sgbm3way" ? STEREO_3WAY : -1;
    }
    numberOfDisparities = S_max_disparity;
    SADWindowSize =S_blocksize;
    scale = S_scale;
    no_display = S_no_display;
    intrinsic_filename =    S_intrinsics;;
    extrinsic_filename =    S_extrinsics;
    if( alg < 0 )
    {
        printf("Command-line parameter error: Unknown stereo algorithm\n\n");
        print_help();
        return -1;
    }
    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        print_help();
        return -1;
    }
    if (scale < 0)
    {
        printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
        return -1;
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }
//    if( img1_filename.empty() || img2_filename.empty() )
//    {
//        printf("Command-line parameter error: both left and right images must be specified\n");
//        return -1;
//    }
    if( (!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }
    if( extrinsic_filename.empty() && !point_cloud_filename.empty() )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }
    int color_mode = alg == STEREO_BM ? 0 : -1;

//    Mat img1 = imread(img1_filename, color_mode);
//    Mat img2 = imread(img2_filename, color_mode);
    Mat img1(S_img_height,S_img_width,CV_8U);
//    Mat img2 = right_img_input;
//
//    if (img1.empty())
//    {
//        printf("Command-line parameter error: could not load the first input image file\n");
//        return -1;
//    }
//    if (img2.empty())
//    {
//        printf("Command-line parameter error: could not load the second input image file\n");
//        return -1;
//    }
//    if (scale != 1.f)
//    {
//        Mat temp1, temp2;
//        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
//        resize(img1, temp1, Size(), scale, scale, method);
//        img1 = temp1;
//        resize(img2, temp2, Size(), scale, scale, method);
//        img2 = temp2;
//    }
    img_size = img1.size();
    Mat img1r, img2r;//畸变校正后的图
    Mat img1rec,img2rec,img1r2,img2r2;//畸变后的图像img1rec=img1r,img1r2是画方框重新定义大小的图片
    if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename.c_str());
            return -1;
        }
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;
        std::cout<<"m1="<<M1<<std::endl;
        M1 *= scale;
        M2 *= scale;
        fs.open(extrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename.c_str());
            return -1;
        }
        fs["R"] >> R;
        fs["T"] >> T;
        //立体校正函数其中cameraMatrixL, distCoeffL,  cameraMatrixR, distCoeffR, R, T皆是输入的相机参数，
        // 通过stereoRectify得出校正旋转矩阵R、投影矩阵P、重投影矩阵Q，其中参数-1会输出完整的去畸变图像，输入0会自动裁剪图像。
        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2 );
        cout<<"roi1"<<roi1<<" roi2"<<roi2<<endl;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);//畸变校准
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
//        Mat img1r, img2r;
//        remap(img1, img1r, map11, map12, INTER_LINEAR);//坐标重映射图片大时map11.data无值
//        remap(img2, img2r, map21, map22, INTER_LINEAR);
//        rectangle(img1r, roi1, Scalar(0,0,255), 3, 8);
//        rectangle(img2r, roi2, Scalar(0,0,255), 3, 8);
//        img1rec=img1r(roi1).clone();
//        img2rec=img2r(roi2).clone();
//        resize(img1rec, img1r2, img1.size());
//        resize(img1rec, img2r2, img1.size());
//        img1 = img1r;//img1r是自动切割后的图片
//        img2 = img2r;
    }
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
    //对视差生成效果影响较大的主要参数是 SADWindowSize、numberOfDisparities 和 uniquenessRatio 三个
    bm->setROI1(roi1);//左右视图的有效像素区域一般由双目校正阶段的 cvStereoRectify 函数传递
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);//预处理滤波器的截断值
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);//SAD窗口大小，容许范围是[5,255]，一般应该在 5x5 至 21x21 之间，参数必须是奇数，int 型
    cout<<"blocksize="<<bm->getBlockSize()<<endl;
    bm->setMinDisparity(100);//最小视差默认值为 0, 可以是负值，int 型
    bm->setNumDisparities(80);//numberOfDisparities 视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍，int 型
    cout<<"NumDisparities="<<bm->getNumDisparities()<<endl;
    bm->setTextureThreshold(10);//低纹理区域的判断阈值 该参数不能为负值，int 型
    bm->setUniquenessRatio(5);//15 视差唯一性百分比 一般5-15左右的值比较合适
    cout<<"UniquenessRatio="<<bm->getUniquenessRatio()<<endl;
    bm->setSpeckleWindowSize(100);//检查视差连通区域变化度的窗口大小值为 0 时取消 speckle 检查，int 型
    bm->setSpeckleRange(32);//视差变化阈值当窗口内视差变化大于阈值时，该窗口内的视差清零
    bm->setDisp12MaxDiff(10);//1 左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。超过该阈值的视差值将被清零。
    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(7);//sgbmWinSize SAD窗口大小，容许范围是[5,255]，一般应该在 5x5 至 21x21 之间，参数必须是奇数，int 型
    cout<<"sgbmblocksize="<<sgbm->getBlockSize()<<endl;
    int cn = img1.channels();
    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);//最小视差默认值为 0, 可以是负值，int 型
    sgbm->setNumDisparities(160);//numberOfDisparities视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍，int 型
    cout<<"sgbmNumDisparities="<<sgbm->getNumDisparities()<<endl;
    sgbm->setUniquenessRatio(15);//15 视差唯一性百分比 一般5-15左右的值比较合适
    cout<<"sgbmUniquenessRatio="<<sgbm->getUniquenessRatio()<<endl;
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(10);//32
    sgbm->setDisp12MaxDiff(1);//1
    if(alg==STEREO_HH)
        sgbm->setMode(StereoSGBM::MODE_HH);
    else if(alg==STEREO_SGBM)
        sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(alg==STEREO_3WAY)
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
    return 1;
}
//计算视差和深度
int Stereo_Calib::makedisp(const cv::Mat &left_img_input, const cv::Mat &right_img_input)
{
    remap(left_img_input, img_left_r, map11, map12, INTER_LINEAR);//坐标重映射图片大时map11.data无值
    remap(right_img_input, img_right_r, map21, map22, INTER_LINEAR);
    int numberOfDisparities = S_max_disparity;
    Mat disp, disp8,disp16;
    clock_t t_start,t_end;
    t_start= clock();
    if( alg == STEREO_BM ){
        cout<<"ooooo"<<alg<<endl;
        bm->compute(img_left_r, img_right_r, disp);//计算视差
        cout<<"!!!1"<<alg<<endl;
    }
    else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
        sgbm->compute(img_left_r, img_right_r, disp);
    t_end=clock();
//    printf("Time elapsed: %dms\n", t_end-t_start);

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
    {
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        disp.convertTo(disp16, CV_16U);
    }
    else
    {
        disp.convertTo(disp8, CV_8U);//16位有符号转为8位无符号
        disp.convertTo(disp16, CV_16U);//16位有符号转为16位无符号
    }
//    cout<<disp.size<<"   "<<disp8.size<<endl;
    int tt=0;
    Mat depth(disp.rows,disp.cols,CV_16U);
    for(int i=0;i<(disp8.cols);i++)//计算深度
    {
        for(int j=0;j<(disp8.rows);j++)
        {
            if(disp16.at<uint16_t>(j,i)&&(disp16.at<uint16_t>(j,i)>50))
            {
                depth.at<uint16_t >(j,i) = (437.1073*79.3291*16) / disp16.at<uint16_t>(j,i);//depth = ( f * baseline) / disp
                tt++;
            }
            else depth.at<uint16_t >(j,i) =0;
        }
    }
    S_depth=depth;
//    imwrite("../../../../data_img_depth/depth_sgbm_1.png", depth);
//    if(!(left_img_input.empty()||right_img_input.empty()))
//    {
//        imshow("depth",depth);
//        imshow("right",right_img_input);
//        waitKey(0);
//    }
    return 0;
}