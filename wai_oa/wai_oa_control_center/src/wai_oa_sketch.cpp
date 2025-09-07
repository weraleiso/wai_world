#include<wai_oa_sketch.h>
#include<QFileDialog>


/////////////////////////////////////////////////
/// OpenCV Sketch Tool
/////////////////////////////////////////////////
void WAIOASketch::cb_tmr_sketch(const ros::TimerEvent& event)
{
    if(cv::getWindowProperty("WAI OA Sketch",cv::WND_PROP_AUTOSIZE)==-1)
    {
        tmr_sketch.stop();
        return;
    }
    // OpenCV loop needs enough waitKey time-delay to process data,
    // 1/20Hz=50ms-10ms=40ms is enough to meet deadline of timer!
    m_c_key_pressed=(char)cv::waitKey(int(1.0/m_f_node_sample_frequency*1000.0)/2.0);
    //printf("Pressed: %d\n",m_c_key_pressed);
    if(m_c_key_pressed==27) // ESC
    {

    }
    else if(m_c_key_pressed==-23) // LEFT Alt
    {

    }
    else if(m_c_key_pressed==-29) // LEFT Ctrl
    {

    }
    else if(m_c_key_pressed==-31) // LEFT Shift
    {
    }
    else if(m_c_key_pressed=='c')
    {
        m_b_circle_enabled=!m_b_circle_enabled;

        if(m_b_circle_enabled==true)
        {
            m_pnt_cursor_pos_1=m_pnt_cursor_pos+m_pnt_cursor_offset;
        }
        else
        {
            DrawGUI();
            m_pnt_cursor_pos_2=m_pnt_cursor_pos+m_pnt_cursor_offset;
            cv::circle(mat_img_sketch,
                       m_pnt_cursor_pos_1,
                       m_i_line_length,
                       m_scl_brush_color,
                       m_i_brush_thickness);

            // Update the rest
            m_pnt_cursor_pos_old=m_pnt_cursor_pos_2;
            AddCurrentSketchToHistoryAndDraw();
        }
    }
    else if(m_c_key_pressed=='t')
    {
        DrawGUI();
        cv::putText(mat_img_gui,"Test: Z_ges'",m_pnt_cursor_pos+m_pnt_cursor_offset,cv::FONT_HERSHEY_DUPLEX,1.0,m_scl_brush_color,m_i_brush_thickness);
        AddCurrentSketchToHistoryAndDraw();
    }
    else
    {

    }
}
void WAIOASketch::cb_trackbar_on_change(int newValue,void* object)
{
    WAIOASketch* sketch = (WAIOASketch*) object;
    sketch->RedrawSketchOnScroll();
}

WAIOASketch::WAIOASketch()
{
    m_i_scroll_start_x=0;
    m_i_scroll_start_y=0;
    m_i_scroll_stop_x=0;
    m_i_scroll_stop_y=0;
    m_pnt_cursor_offset=cv::Point(m_i_scroll_stop_x,m_i_scroll_stop_y);
    m_pnt_cursor_pos.x=0;
    m_pnt_cursor_pos.y=0;
    m_pnt_cursor_pos_old=m_pnt_cursor_pos+m_pnt_cursor_offset;
    m_pnt_cursor_pos_1=m_pnt_cursor_pos;
    m_pnt_cursor_pos_2=m_pnt_cursor_pos;
    m_pnt_gui_element_dims.x=40; // GUI Element width
    m_pnt_gui_element_dims.y=40; // GUI Element height
    m_scl_brush_color=cv::Scalar(255,255,255);
    m_f_node_sample_frequency=0.0;
    m_f_sketch_resize_factor=1.0;
    m_i_image_history_counter=-1;
    m_f_line_angle_deg=0;
    m_i_brush_thickness=2;
    m_i_brush_shape=0;
    m_i_brush_mode=0;
    m_b_scroll_enabled=false;
    m_b_brush_enabled=false;
    m_i_line_length=0;
}

WAIOASketch::~WAIOASketch()
{

}

void WAIOASketch::Initialize(ros::NodeHandle* nh_,
                             image_transport::ImageTransport* it_,
                             sound_play::SoundClient* hdl_snd_client,
                             float f_node_sample_frequency,float f_sketch_resize_factor)
{
    m_hdl_node=nh_;
    m_hdl_it=it_;
    m_hdl_snd_client=hdl_snd_client;
    m_s_package_path=ros::package::getPath("wai_oa_gazebo");
    m_f_node_sample_frequency=f_node_sample_frequency;
    m_f_sketch_resize_factor=f_sketch_resize_factor;
    m_s_projector_topic="";
    m_s_rep_id="";

    // Get current Presenter or Audience IDs
    /*
    ros::get_environment_variable(m_s_rep_id,"WAI_OA_PRESENTER_ID");
    if(m_s_rep_id.compare("")==0)
    {
        ros::get_environment_variable(m_s_rep_id,"WAI_OA_AUDIENCE_ID");
        if(m_s_rep_id.compare("")==0)
        {
            ROS_WARN_STREAM("Sketch - No ID found! Closing plugin...");
            exit(1);
        }
        else
        {
            ROS_WARN_STREAM("Sketch - Found AUDIENCE ID as \"" << m_s_rep_id << "\"");
            m_s_projector_topic="/wai_world/oa"+m_s_rep_id+"/projector/image_raw";
        }
    }
    else
    {
        ROS_WARN_STREAM("Sketch - Found PRESENTER ID as \"" << m_s_rep_id << "\"");
        m_s_projector_topic="/wai_world/oa/projector/image_raw";
    }
    m_pub_img_sketch=it_->advertise(m_s_projector_topic,1);
        */
    m_pub_img_sketch=it_->advertise("projector/image_raw",1);
}

void WAIOASketch::PlaySound(std::string s_sound)
{
    sound_play::Sound snd_play=m_hdl_snd_client->waveSoundFromPkg("wai_oa_gazebo","resources/sounds/sketch/"+s_sound+".wav");
    snd_play.play();
}

void WAIOASketch::ReopenSketch()
{
    if(cv::getWindowProperty("WAI OA Sketch",cv::WND_PROP_AUTOSIZE)!=-1)
    {
        cv::destroyWindow("WAI OA Sketch");
    }
    cv::namedWindow("WAI OA Sketch",cv::WINDOW_AUTOSIZE);
    cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Welcome!");
    cv::setMouseCallback("WAI OA Sketch",onWAIOASketchMouseMove,this);

    ResetSketch();

    tmr_sketch=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOASketch::cb_tmr_sketch,this,false,true);
}

void WAIOASketch::ResetSketch(cv::Size siz_sketch_size,
                cv::Scalar sca_col_bg,
                bool b_grid_enabled,
                cv::Size siz_grid,
                cv::Scalar scl_col_grid)
{
    // Reset image history
    vec_mat_img_sketches.clear();
    m_i_image_history_counter=-1;

    m_i_scroll_height=0;
    m_i_scroll_width=0;
    m_i_scroll_start_x=0;
    m_i_scroll_start_y=0;
    m_i_scroll_stop_x=0;
    m_i_scroll_stop_y=0;
    m_pnt_cursor_offset.x=0;
    m_pnt_cursor_offset.y=0;
    m_siz_sketch_size=siz_sketch_size;
    m_siz_sketch_size_scroll=m_siz_sketch_size;
    if(m_siz_sketch_size_scroll.width>1200)m_siz_sketch_size_scroll.width=1200; // 1240
    if(m_siz_sketch_size_scroll.height>720)m_siz_sketch_size_scroll.height=720;
    m_scl_color_grid=scl_col_grid;
    mat_img_sketch=cv::Mat::zeros(m_siz_sketch_size,CV_8UC3);
    mat_img_sketch_scroll=cv::Mat::zeros(m_siz_sketch_size_scroll,CV_8UC3);
    mat_img_grid=cv::Mat::zeros(m_siz_sketch_size,CV_8UC3);
    mat_img_grid_scroll=cv::Mat::zeros(m_siz_sketch_size_scroll,CV_8UC3);
    mat_img_gui=cv::Mat::zeros(m_siz_sketch_size_scroll,CV_8UC3);
    mat_img_grid.setTo(sca_col_bg);
    mat_img_sketch.setTo(sca_col_bg);
    mat_img_gui.setTo(sca_col_bg);
    m_alpha=0.75;
    m_beta=1.0-m_alpha;

    //cv::createTrackbar("Scroll Height:","WAI OA Sketch",&m_i_scroll_height,m_siz_sketch_size.height-m_siz_sketch_size_scroll.height,&WAIOASketch::cb_trackbar_on_change,this);
    //cv::createTrackbar("Scroll Width:","WAI OA Sketch",&m_i_scroll_width,m_siz_sketch_size.width-m_siz_sketch_size_scroll.width,&WAIOASketch::cb_trackbar_on_change,this);

    int i_grid_thickness=1;
    int i_grid_thickness_border=3;
    int i_grid_border_width=2*siz_grid.width;
    int i_grid_border_height=2*siz_grid.height;
    if(b_grid_enabled==true)
    {
        for(int i=i_grid_border_height;i<=(mat_img_grid.rows-i_grid_border_height);i=i+siz_grid.height)
        {
            if(i==i_grid_border_height || i==(mat_img_grid.rows-i_grid_border_height))
            {
                i_grid_thickness=i_grid_thickness_border;
            }
            else
            {
                i_grid_thickness=1;
            }
            cv::line(mat_img_grid,cv::Point(i_grid_border_width,i),cv::Point(mat_img_grid.cols-i_grid_border_width,i),m_scl_color_grid,i_grid_thickness);
        }
        cv::line(mat_img_grid,
                 cv::Point(i_grid_border_width,mat_img_grid.rows-i_grid_border_height),
                 cv::Point(mat_img_grid.cols-i_grid_border_width,mat_img_grid.rows-i_grid_border_height),
                 m_scl_color_grid,i_grid_thickness_border);

        for(int j=i_grid_border_width;j<=(mat_img_grid.cols-i_grid_border_width);j+=siz_grid.width)
        {
            if(j==i_grid_border_width || j==(mat_img_grid.cols-i_grid_border_width))
            {
                i_grid_thickness=i_grid_thickness_border;
            }
            else
            {
                i_grid_thickness=1;
            }
            cv::line(mat_img_grid,cv::Point(j,i_grid_border_height),cv::Point(j,mat_img_grid.rows-i_grid_border_height),m_scl_color_grid,i_grid_thickness);
        }
        cv::line(mat_img_grid,
                 cv::Point(mat_img_grid.cols-i_grid_border_width,i_grid_border_height),
                 cv::Point(mat_img_grid.cols-i_grid_border_width,mat_img_grid.rows-i_grid_border_height),
                 m_scl_color_grid,i_grid_thickness_border);

        // Add timestamp
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::stringstream sst_date_time;
        sst_date_time<<std::put_time(&tm, "OA Sketch %d-%m-%Y %H-%M-%S (A4@150DPI)");
        cv::putText(mat_img_grid,sst_date_time.str(),cv::Point(75,100),cv::FONT_HERSHEY_DUPLEX,1.0,m_scl_color_grid,1);
    }
    DrawGUI();
    AddCurrentSketchToHistoryAndDraw();

    //QMessageBox::information(NULL,"Reset Sketch","Reset to new sketch. Be creative now!");
    cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered RESET!");
}

void WAIOASketch::RedrawSketchOnScroll()
{
    mat_img_sketch_scroll=mat_img_sketch(cv::Rect(m_i_scroll_width,m_i_scroll_height,m_siz_sketch_size_scroll.width,m_siz_sketch_size_scroll.height));
    mat_img_grid_scroll=mat_img_grid(cv::Rect(m_i_scroll_width,m_i_scroll_height,m_siz_sketch_size_scroll.width,m_siz_sketch_size_scroll.height));
    DrawSketch();
}

void WAIOASketch::DrawGUIIcon(int i_icon_id)
{
    cv::Mat mat_img_icon=cv::imread(m_s_package_path+"/resources/icons/sketchtool/icon_"+std::to_string(i_icon_id)+".png",cv::IMREAD_ANYCOLOR);
    cv::Mat mat_img_sketch_icon_roi(mat_img_gui,cv::Rect(4,4+i_icon_id*m_pnt_gui_element_dims.y,mat_img_icon.cols,mat_img_icon.rows));
    cv::Mat mat_img_icon_inv;
    cv::bitwise_not(mat_img_icon,mat_img_icon_inv);
    mat_img_icon_inv.copyTo(mat_img_sketch_icon_roi);
    cv::rectangle(mat_img_gui,cv::Rect(0,i_icon_id*m_pnt_gui_element_dims.y,m_pnt_gui_element_dims.x,m_pnt_gui_element_dims.y),cv::Scalar(255-150,255-109,255-26),2);
}

void WAIOASketch::DrawGUI()
{
    // Reset GUI layer
    mat_img_gui.setTo(0);

    // SHOW ABOUT
    DrawGUIIcon(0);

    // RESET AND SEND
    DrawGUIIcon(1);
    DrawGUIIcon(2);

    // THICKNESS +/-
    DrawGUIIcon(3);
    DrawGUIIcon(4);

    // COLOR Bl,R,G,B
    DrawGUIIcon(5);
    DrawGUIIcon(6);
    DrawGUIIcon(7);
    DrawGUIIcon(8);

    // HISTORY Back
    DrawGUIIcon(9);

    // FILE Open/Save
    DrawGUIIcon(10);
    DrawGUIIcon(11);
}

void WAIOASketch::DrawSketch()
{
    //mat_img_overlay=mat_img_grid_scroll+mat_img_sketch_scroll+mat_img_gui;
    //cv::addWeighted(mat_img_grid,m_alpha,mat_img_sketch,m_beta,0.0,mat_img_overlay);
    cv::bitwise_not(mat_img_grid_scroll+mat_img_sketch_scroll+mat_img_gui,mat_img_overlay);
    cv::imshow("WAI OA Sketch",mat_img_overlay);
}

void WAIOASketch::AddCurrentSketchToHistoryAndDraw()
{
    vec_mat_img_sketches.push_back(mat_img_sketch.clone());
    m_i_image_history_counter++;
    RedrawSketchOnScroll();
}

void WAIOASketch::LoadLastSketchInHistoryAndDraw()
{
    if(m_i_image_history_counter>0)
    {
        vec_mat_img_sketches.pop_back();
        m_i_image_history_counter--;
        mat_img_sketch=vec_mat_img_sketches[m_i_image_history_counter].clone();
        RedrawSketchOnScroll();
    }
}

void WAIOASketch::ShowAboutSketch()
{
    QMessageBox::information(NULL,"ABOUT","<b>WAI OA Sketch</b><br>A sketch tool for Open Auditorium (OA).<br>(©2019, W. A. Isop)");
    cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered ABOUT!");
}

void WAIOASketch::SaveCurrentSketchToFile()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm,"%Y_%m_%d_%H_%M_%S_");
    cv::Mat mat_img_save;
    cv::bitwise_not(mat_img_grid+mat_img_sketch,mat_img_save);
    cv::imwrite("/home/ias/"+oss.str()+"oa_sketch.png",mat_img_save);
    //QMessageBox::information(NULL,"SAVED Sketch","Saved current sketch to *.png file in home folder!");
    cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered SAVE SKETCH!");
}

void WAIOASketch::SendSketchToProjection()
{
    cv::Mat mat_img_publish;
    cv::Mat mat_img_projection;
    cv::bitwise_not(mat_img_grid_scroll+mat_img_sketch_scroll,mat_img_projection);
    cv::resize(mat_img_projection,
               mat_img_publish,
               cv::Size(),
               m_f_sketch_resize_factor,
               m_f_sketch_resize_factor);

    m_msg_img_sketch=cv_bridge::CvImage(
                std_msgs::Header(),
                "bgr8",
                mat_img_publish).toImageMsg();
    m_pub_img_sketch.publish(m_msg_img_sketch);
    ros::spinOnce();
    //QMessageBox::information(NULL,"SENT Sketch","Published sketch to projector.");
    cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered PROJECT!");
}

int WAIOASketch::GetMMFromPixels(int i_pixels)
{
    return int(float(i_pixels)/150.0*25.4); // At 150DPI
}

void WAIOASketch::onWAIOASketchMove(int event,int x,int y,int flags)
{
    m_pnt_cursor_pos.x=x;
    m_pnt_cursor_pos.y=y;

    // Calculate angle of line
    if(m_b_line_enabled)
    {
        m_pnt_line_length=m_pnt_cursor_pos_2-(m_pnt_cursor_pos_1-m_pnt_cursor_offset);
        if(m_pnt_line_length.x!=0)
        {
            m_f_line_angle_deg=atan(-float(m_pnt_line_length.y)/float(fabs(m_pnt_line_length.x)))*180.0/M_PI;
        }
    }
    else m_f_line_angle_deg=0.0;

    cv::setWindowTitle("WAI OA Sketch",
                       "WAI OA Sketch - Triggered MOUSE (Px="+
                       std::to_string(m_pnt_cursor_pos.x)+"pxl,Py="+
                       std::to_string(m_pnt_cursor_pos.y)+"pxl,R="+
                       std::to_string(m_i_line_length)+"pxl|"+std::to_string(GetMMFromPixels(m_i_line_length))+"mm,Alpha="+
                       std::to_string(m_f_line_angle_deg)+"deg)!");

    if(event==cv::EVENT_MBUTTONDOWN)
    {
        m_b_scroll_enabled=!m_b_scroll_enabled;
        if(m_b_scroll_enabled)
        {
            m_i_scroll_start_x=x+m_i_scroll_stop_x;
            m_i_scroll_start_y=y+m_i_scroll_stop_y;
        }
        else
        {
            m_i_scroll_stop_x=m_i_scroll_width;
            m_i_scroll_stop_y=m_i_scroll_height;
            m_pnt_cursor_offset=cv::Point(m_i_scroll_stop_x,m_i_scroll_stop_y);
        }
    }
    if(m_b_scroll_enabled==true)
    {
        m_i_scroll_width=m_i_scroll_start_x-x;
        m_i_scroll_height=m_i_scroll_start_y-y;

        if(m_i_scroll_width>(m_siz_sketch_size.width-m_siz_sketch_size_scroll.width))
        {
            m_i_scroll_width=m_siz_sketch_size.width-m_siz_sketch_size_scroll.width;
        }
        if(m_i_scroll_width<0)
        {
            m_i_scroll_width=0;
        }
        if(m_i_scroll_height>(m_siz_sketch_size.height-m_siz_sketch_size_scroll.height))
        {
            m_i_scroll_height=m_siz_sketch_size.height-m_siz_sketch_size_scroll.height;
        }
        if(m_i_scroll_height<0)
        {
            m_i_scroll_height=0;
        }
        RedrawSketchOnScroll();
        return;
    }

    if((event==10||event==11) && !m_b_scroll_enabled)
    {
        if(flags>0)
        {
            m_i_scroll_height+=25;
            m_i_scroll_stop_y+=25;
            m_pnt_cursor_offset.y+=25;
            if(m_i_scroll_height>=(m_siz_sketch_size.height-m_siz_sketch_size_scroll.height))
            {
                m_i_scroll_height=m_siz_sketch_size.height-m_siz_sketch_size_scroll.height;
                m_i_scroll_stop_y=m_siz_sketch_size.height-m_siz_sketch_size_scroll.height;
                m_pnt_cursor_offset.y=m_siz_sketch_size.height-m_siz_sketch_size_scroll.height;
            }
            RedrawSketchOnScroll();
        }
        else
        {
            m_i_scroll_height-=25;
            m_i_scroll_stop_y-=25;
            m_pnt_cursor_offset.y-=25;
            if(m_i_scroll_height<=0)
            {
                m_i_scroll_height=0;
                m_i_scroll_stop_y=0;
                m_pnt_cursor_offset.y=0;
            }
            RedrawSketchOnScroll();
        }
    }

    if(event==cv::EVENT_LBUTTONUP)
    {
        if(m_pnt_cursor_pos.x<m_pnt_gui_element_dims.x)
        {
            PlaySound("click");
            if(m_pnt_cursor_pos.y<1*m_pnt_gui_element_dims.y)
            {
                ShowAboutSketch();
                return;
            }
            else if(m_pnt_cursor_pos.y>1*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<2*m_pnt_gui_element_dims.y)
            {
                ResetSketch();
                return;
            }
            else if(m_pnt_cursor_pos.y>2*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<3*m_pnt_gui_element_dims.y)
            {
                SendSketchToProjection();
                return;
            }
            else if(m_pnt_cursor_pos.y>3*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<4*m_pnt_gui_element_dims.y)
            {
                if(m_i_brush_thickness<=10)
                {
                    m_i_brush_thickness++;
                    cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered BRUSH THICKNESS ("+std::to_string(m_i_brush_thickness)+")!");
                }
                return;
            }
            else if(m_pnt_cursor_pos.y>4*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<5*m_pnt_gui_element_dims.y)
            {
                if(m_i_brush_thickness>=1)
                {
                    m_i_brush_thickness--;
                    cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered BRUSH THICKNESS ("+std::to_string(m_i_brush_thickness)+")!");
                }
                return;
            }
            else if(m_pnt_cursor_pos.y>5*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<6*m_pnt_gui_element_dims.y)
            {
                m_scl_brush_color=cv::Scalar(255,255,255);
                cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered BRUSH COLOR (White)!");
                return;
            }
            else if(m_pnt_cursor_pos.y>6*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<7*m_pnt_gui_element_dims.y)
            {
                //m_scl_brush_color=cv::Scalar(0,0,255);
                m_scl_brush_color=cv::Scalar(255,255,0);
                cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered BRUSH COLOR (Red)!");
                return;
            }
            else if(m_pnt_cursor_pos.y>7*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<8*m_pnt_gui_element_dims.y)
            {
                //m_scl_brush_color=cv::Scalar(0,255,0);
                m_scl_brush_color=cv::Scalar(255,0,255);
                cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered BRUSH COLOR (Green)!");
                return;
            }
            else if(m_pnt_cursor_pos.y>8*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<9*m_pnt_gui_element_dims.y)
            {
                //m_scl_brush_color=cv::Scalar(255,0,0);
                m_scl_brush_color=cv::Scalar(0,255,255);
                cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered BRUSH COLOR (Blue)!");
                return;
            }
            else if(m_pnt_cursor_pos.y>9*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<10*m_pnt_gui_element_dims.y)
            {
                LoadLastSketchInHistoryAndDraw();
                cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered UNDO!");
                return;
            }
            else if(m_pnt_cursor_pos.y>10*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<11*m_pnt_gui_element_dims.y)
            {
                QString qst_path=QFileDialog::getOpenFileName(NULL,"LOAD Sketch","/home/ias","PNG Files (*.png)");
                if(qst_path.isNull()==false)
                {
                    cv::Mat mat_img_sketch_loaded=cv::imread(qst_path.toStdString());
                    ResetSketch(mat_img_sketch_loaded.size(),cv::Scalar(0,0,0),false);
                    cv::Mat mat_img_sketch_loaded_inv;
                    cv::bitwise_not(mat_img_sketch_loaded,mat_img_sketch_loaded_inv);
                    mat_img_sketch=mat_img_sketch_loaded_inv;
                    //cv::resize(mat_img_sketch_loaded,mat_img_sketch,m_siz_sketch_size);
                    cv::setWindowTitle("WAI OA Sketch","WAI OA Sketch - Triggered LOAD SKETCH!");
                }
                return;
            }
            else if(m_pnt_cursor_pos.y>11*m_pnt_gui_element_dims.y
                    && m_pnt_cursor_pos.y<12*m_pnt_gui_element_dims.y)
            {
                SaveCurrentSketchToFile();
                return;
            }
            else
            {
                // Do nothing...
            }
        }
        else
        {
            // No GUI element was hit, check for drawing interaction
            m_b_brush_enabled=!m_b_brush_enabled;
            if(m_b_brush_enabled==true)
            {

            }
            else
            {
                AddCurrentSketchToHistoryAndDraw();
            }
            m_pnt_cursor_pos_old=m_pnt_cursor_pos+m_pnt_cursor_offset;
        }
        return;
    }
    else if(event==cv::EVENT_RBUTTONUP)
    {
        m_b_line_enabled=!m_b_line_enabled;

        if(m_b_line_enabled==true)
        {
            m_pnt_cursor_pos_1=m_pnt_cursor_pos+m_pnt_cursor_offset;
        }
        else
        {
            DrawGUI();
            m_pnt_cursor_pos_2=m_pnt_cursor_pos+m_pnt_cursor_offset;
            cv::line(mat_img_sketch,
                     m_pnt_cursor_pos_1,
                     m_pnt_cursor_pos_2,
                     m_scl_brush_color,
                     m_i_brush_thickness);

            // Update the rest
            m_pnt_cursor_pos_old=m_pnt_cursor_pos_2;
            AddCurrentSketchToHistoryAndDraw();
        }
        return;
    }
    else if(event==cv::EVENT_MOUSEMOVE)
    {
        // Check for mouse is moving on the sketch
        // Draw current shape as user feedback
        if(m_b_brush_enabled==true)
        {
            cv::line(mat_img_sketch,
                     m_pnt_cursor_pos_old,
                     m_pnt_cursor_pos+m_pnt_cursor_offset,
                     m_scl_brush_color,
                     m_i_brush_thickness);
            m_pnt_cursor_pos_old=m_pnt_cursor_pos+m_pnt_cursor_offset;
        }
        else if(m_b_line_enabled==true)
        {
            DrawGUI();
            m_pnt_cursor_pos_2=m_pnt_cursor_pos;
            m_pnt_line_length=m_pnt_cursor_pos_2-(m_pnt_cursor_pos_1-m_pnt_cursor_offset);
            m_i_line_length=int(sqrt(m_pnt_line_length.x*m_pnt_line_length.x+m_pnt_line_length.y*m_pnt_line_length.y));
            cv::line(mat_img_gui,
                     m_pnt_cursor_pos_1-m_pnt_cursor_offset,
                     m_pnt_cursor_pos_2,
                     m_scl_brush_color,
                     m_i_brush_thickness);
        }
        else if(m_b_circle_enabled==true)
        {
            DrawGUI();
            m_pnt_cursor_pos_2=m_pnt_cursor_pos;
            m_pnt_line_length=m_pnt_cursor_pos_2-(m_pnt_cursor_pos_1-m_pnt_cursor_offset);
            m_i_line_length=int(sqrt(m_pnt_line_length.x*m_pnt_line_length.x+m_pnt_line_length.y*m_pnt_line_length.y));
            cv::circle(mat_img_gui,
                       m_pnt_cursor_pos_1-m_pnt_cursor_offset,
                       m_i_line_length,
                       m_scl_brush_color,
                       m_i_brush_thickness);
        }
        else
        {
            // Do nothing...
        }
        RedrawSketchOnScroll();
    }
    else
    {
        // Do nothing...
    }

    return;
}
