#include<wai_oa_image_processing_strategy.h>



/////////////////////////////////////////////////
/// Implementation of advanced hand detection
/////////////////////////////////////////////////
HandsAndHeadDetector::HandsAndHeadDetector(float f_range_min,float f_range_max,float f_threshold_dist,float f_threshold_bounds,float f_res_x,float f_res_y,float f_fx,float f_fy,float f_cx,float f_cy)
{
    m_f_range_min=f_range_min;
    m_f_range_max=f_range_max;
    m_f_threshold_dist=f_threshold_dist;
    m_f_threshold_bounds=f_threshold_bounds;
    m_f_res_x=f_res_x;
    m_f_res_y=f_res_y;
    m_f_fx=f_fx;
    m_f_fy=f_fy;
    m_f_cx=f_cx;
    m_f_cy=f_cy;
    m_siz_gaussian_blur.width=13;
    m_siz_gaussian_blur.height=13;

    m_m33_hah.setIdentity();
    m_vc3_hand_scaled_origin=tf::Vector3(1.0,0.25,m_f_range_max);
    m_vc3_camera_rgbd_wrt_head_old=tf::Vector3(0.0,0.0,0.0);
}

tf::Matrix3x3 HandsAndHeadDetector::process(cv::Mat mat_src)
{
    m_m33_hah.setIdentity();
    //tf::Vector3 m_vc3_camera_rgbd_wrt_hand_left;
    //tf::Vector3 m_vc3_camera_rgbd_wrt_hand_right;
    //tf::Vector3 m_vc3_camera_rgbd_wrt_head;
    // Detect hand position from geometry
    //cv::Mat mat_depth_32fc1=mat_src;
    //cv::Mat mat_depth_32fc1_cleaned_up;
    cv::Mat mat_depth_32fc1_gaussian_blur;

    // Cleanup Image
    //mat_src.copyTo(mat_depth_32fc1_cleaned_up);
	/*
    mat_depth_32fc1_cleaned_up.setTo(m_f_range_max, mat_depth_32fc1_cleaned_up != mat_depth_32fc1_cleaned_up);
    mat_depth_32fc1_cleaned_up.setTo(m_f_range_max, mat_depth_32fc1_cleaned_up > m_f_range_max);
    mat_depth_32fc1_cleaned_up.setTo(m_f_range_max, mat_depth_32fc1_cleaned_up < m_f_range_min);
    mat_depth_32fc1_cleaned_up.setTo(m_f_range_max, mat_depth_32fc1_cleaned_up != mat_depth_32fc1_cleaned_up);
    mat_depth_32fc1_cleaned_up.setTo(m_f_range_max, mat_depth_32fc1_cleaned_up < m_f_range_min);*/

    // We need to again set the 0.0 distances to 3.0 for head and hand pose detection work properly
    mat_src.setTo(3.0, mat_src == 0);

    // Apply Gaussian Blurr
    cv::GaussianBlur(mat_src,mat_depth_32fc1_gaussian_blur,m_siz_gaussian_blur,0.0,0.0,cv::BORDER_DEFAULT);
    //cv::medianBlur(mat_src,mat_depth_32fc1_gaussian_blur,3); same resurces, weak perf.
    //mat_depth_32fc1_gaussian_blur=mat_src;

    double min_hand_right, max_hand_right;
    double min_hand_left, max_hand_left;
    cv::Point min_loc_hand_right, max_loc_hand_right;
    cv::Point min_loc_hand_left, max_loc_hand_left;
    cv::minMaxLoc(mat_depth_32fc1_gaussian_blur(cv::Rect(0,0,m_f_res_x/2,m_f_res_y)),
                  &min_hand_left,
                  &max_hand_left,
                  &min_loc_hand_left,
                  &max_loc_hand_left);
    cv::minMaxLoc(mat_depth_32fc1_gaussian_blur(cv::Rect(m_f_res_x/2,0,m_f_res_x/2,m_f_res_y)), // LEFT/RIGHT SEEN FROM SPECTATOR CAM!
                  &min_hand_right,
                  &max_hand_right,
                  &min_loc_hand_right,
                  &max_loc_hand_right);


    // Find most upper point of body via gradient
    int ddepth = CV_32F;
    cv::Mat grad_x;
    cv::Sobel(mat_depth_32fc1_gaussian_blur,grad_x,ddepth,1,0);
    for(int i=grad_x.rows*m_f_threshold_bounds;i<grad_x.rows-grad_x.rows*m_f_threshold_bounds;i+=4) // -96
    {
        for(int j=grad_x.cols*m_f_threshold_bounds;j<grad_x.cols-grad_x.cols*m_f_threshold_bounds;j+=4) // -64
        {
            if(grad_x.at<float>(i,j)>m_f_threshold_dist) // 0.1
            {
                tf::Vector3 vc3_found=
                        tf::Vector3( (m_vc3_camera_rgbd_wrt_head_old.getX()+ ((j-m_f_cx)*mat_src.at<float>(i+10,j)/m_f_fx) )/2.0,
                        (m_vc3_camera_rgbd_wrt_head_old.getY()+ ((i-m_f_cy)*mat_src.at<float>(i+10,j)/m_f_fy) )/2.0,
                        (m_vc3_camera_rgbd_wrt_head_old.getZ()+ (mat_src.at<float>(i+10,j)+0.025) )/2.0 );
                if( !std::isnan(vc3_found.getX()) && !std::isnan(vc3_found.getY()) && !std::isnan(vc3_found.getZ()))
                {
                    m_m33_hah.setValue( m_m33_hah.getRow(0).getX(),m_m33_hah.getRow(0).getY(),m_m33_hah.getRow(0).getZ(),
                                        m_m33_hah.getRow(1).getX(),m_m33_hah.getRow(1).getY(),m_m33_hah.getRow(1).getZ(),
                                        vc3_found.getX(),vc3_found.getY(),vc3_found.getZ());
                    m_vc3_camera_rgbd_wrt_head_old.setX(m_m33_hah.getRow(2).getX());
                    m_vc3_camera_rgbd_wrt_head_old.setY(m_m33_hah.getRow(2).getY());
                    m_vc3_camera_rgbd_wrt_head_old.setZ(m_m33_hah.getRow(2).getZ());

                    //cv::circle(grad_x,cv::Point(j,i),5,cv::Scalar(255,255,255),5);
                }
                //ROS_DEBUG("%d ; %d ; %.3f\n",i,j,mat_depth_32fc1_cleaned_up.at<float>(i+10,j));
                goto endof2loops;
            }
        }
    }
    endof2loops:
    //cv::imshow("Debug",grad_x);
    //cv::waitKey(1);


    // Fill tf vector of left hand
    m_m33_hah.setValue( (min_loc_hand_left.x-m_f_cx)*min_hand_left/m_f_fx,(min_loc_hand_left.y-m_f_cy)*min_hand_left/m_f_fy,min_hand_left,
                        m_m33_hah.getRow(1).getX(),m_m33_hah.getRow(1).getY(),m_m33_hah.getRow(1).getZ(),
                        m_m33_hah.getRow(2).getX(),m_m33_hah.getRow(2).getY(),m_m33_hah.getRow(2).getZ());
    if(m_m33_hah.getRow(0).getZ()>m_f_range_max) //2.25
    {
        m_m33_hah.setValue( -0.15,0.0,m_f_range_max,
                            m_m33_hah.getRow(1).getX(),m_m33_hah.getRow(1).getY(),m_m33_hah.getRow(1).getZ(),
                            m_m33_hah.getRow(2).getX(),m_m33_hah.getRow(2).getY(),m_m33_hah.getRow(2).getZ());
    }
    else
    {
        // Scale hand left pointer with non-linear scale
        tf::Vector3 vc3_non_scaled=m_m33_hah.getRow(0);
        tf::Vector3 vc3_scaled= m_vc3_hand_scaled_origin+
                                (vc3_non_scaled-m_vc3_hand_scaled_origin).normalized()*
                                pow((vc3_non_scaled-m_vc3_hand_scaled_origin).length(),2)*1.5;
        m_m33_hah.setValue( vc3_scaled.getX(),vc3_scaled.getY(),vc3_scaled.getZ(),
                            m_m33_hah.getRow(1).getX(),m_m33_hah.getRow(1).getY(),m_m33_hah.getRow(1).getZ(),
                            m_m33_hah.getRow(2).getX(),m_m33_hah.getRow(2).getY(),m_m33_hah.getRow(2).getZ());
    }

    // Fill tf vector of right hand
    m_m33_hah.setValue( m_m33_hah.getRow(0).getX(),m_m33_hah.getRow(0).getY(),m_m33_hah.getRow(0).getZ(),
                        (min_loc_hand_right.x+m_f_res_x/2-m_f_cx)*min_hand_right/m_f_fx,(min_loc_hand_right.y-m_f_cy)*min_hand_right/m_f_fy,min_hand_right,
                        m_m33_hah.getRow(2).getX(),m_m33_hah.getRow(2).getY(),m_m33_hah.getRow(2).getZ());
    if(m_m33_hah.getRow(1).getZ()>m_f_range_max) //2.25
    {
        m_m33_hah.setValue( m_m33_hah.getRow(0).getX(),m_m33_hah.getRow(0).getY(),m_m33_hah.getRow(0).getZ(),
                            0.15,0.0,m_f_range_max,
                            m_m33_hah.getRow(2).getX(),m_m33_hah.getRow(2).getY(),m_m33_hah.getRow(2).getZ());
    }

    return m_m33_hah;
}
