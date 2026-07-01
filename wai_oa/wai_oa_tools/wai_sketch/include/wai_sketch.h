#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<stdio.h>
#include<string.h>
#include<iomanip>
#include<array>
#include<vector>
#include<math.h>

#include<QApplication>
#include<QMainWindow>
#include<QMessageBox>
#include<QFileDialog>



enum DrawingToolStates
{
    DrawNothing=0,
    DrawLoadImage=1,
    DrawSaveImage=2,
    DrawGridToggle=3,
    DrawZoomIn=4,
    DrawZoomOut=5,
    DrawZoomReset=6,
    DrawScrollSelect=7,
    DrawScrollPreview=8,
    DrawScrollDeselect=9,
    DrawCurveSelect=10,
    DrawCurveDeselect=11,
    DrawLineSelectP1=12,
    DrawLinePreview=13,
    DrawLineSelectP2=14,
    DrawRectangleSelectP1=15,
    DrawRectanglePreview=16,
    DrawRectangleSelectP2=17,
    DrawCircleSelectP1=18,
    DrawCirclePreview=19,
    DrawCircleSelectP2=20,
    DrawThicknessDecrease=21,
    DrawThicknessIncrease=22,
    DrawHistoryBack=23,
    DrawColorBlack=24,
    DrawColorWhite=25,
    DrawColorRed=26,
    DrawColorGreen=27,
    DrawColorBlue=28,
    DrawColorCyan=29,
    DrawColorOrange=30,
    PublishToProjector=31,
    DrawShowHelp=32
};



class WAISketch
{
    ros::NodeHandle m_hdl_node;
    image_transport::ImageTransport m_hdl_it;

    image_transport::Publisher m_pub_img_sketch;
    sensor_msgs::ImagePtr m_msg_img_sketch;

    QString qst_img_background_path;
    std::string m_s_path_nodename;
    std::string m_s_sketch_window_name;
    std::string m_s_sketch_window_quick_info;
    std::stringstream sst_quick_info;
    std::string m_s_sketch_window_title;
    unsigned char m_c_sketch_key_pressed;

    float F_NODE_SAMPLE_FREQUENCY;
    std::string S_SKETCH_FORMAT;
    std::string S_SKETCH_FILE_NAME;
    int I_SKETCH_DPI;

    int m_i_sketch_window_size_x;
    int m_i_sketch_window_size_y;
    int m_i_sketch_gui_element_size;
    int m_i_sketch_border_distance;
    int m_i_sketch_grid_distance;
    int m_i_sketch_resolution_x;
    int m_i_sketch_resolution_y;
    int m_i_sketch_roi_offset_x;
    int m_i_sketch_roi_offset_y;
    float f_sketch_zoom_scale;
    bool m_b_sketch_grid_enable;
    cv::Mat m_mat_sketch_window;
    cv::Mat m_mat_background;
    cv::Mat m_mat_sketch_grid;
    cv::Mat m_mat_sketch_current;
    cv::Mat m_mat_sketch_zoomed;
    cv::Mat m_mat_sketch_zoomed_scrolled;
    cv::Mat m_mat_sketch_inner_roi_base;
    cv::Mat m_mat_sketch_inner_roi;
    cv::Mat m_mat_sketch_inner_roi_zoomed;
    cv::Mat m_mat_sketch_loaded;

    std::vector<cv::Mat> m_vec_mat_sketch_gui;
    std::vector<cv::Mat> m_vec_sketch_layers;

    // Drawing tools and user inputs
    std::array<DrawingToolStates,33> drawing_tool_states
    {
        DrawingToolStates::DrawNothing,
        DrawingToolStates::DrawLoadImage,
        DrawingToolStates::DrawSaveImage,
        DrawingToolStates::DrawGridToggle,
        DrawingToolStates::DrawZoomIn,
        DrawingToolStates::DrawZoomOut,
        DrawingToolStates::DrawZoomReset,
        DrawingToolStates::DrawScrollSelect,
        DrawingToolStates::DrawScrollPreview,
        DrawingToolStates::DrawScrollDeselect,
        DrawingToolStates::DrawCurveSelect,
        DrawingToolStates::DrawCurveDeselect,
        DrawingToolStates::DrawLineSelectP1,
        DrawingToolStates::DrawLinePreview,
        DrawingToolStates::DrawLineSelectP2,
        DrawingToolStates::DrawRectangleSelectP1,
        DrawingToolStates::DrawRectanglePreview,
        DrawingToolStates::DrawRectangleSelectP2,
        DrawingToolStates::DrawCircleSelectP1,
        DrawingToolStates::DrawCirclePreview,
        DrawingToolStates::DrawCircleSelectP2,
        DrawingToolStates::DrawThicknessDecrease,
        DrawingToolStates::DrawThicknessIncrease,
        DrawingToolStates::DrawHistoryBack,
        DrawingToolStates::DrawColorBlack,
        DrawingToolStates::DrawColorWhite,
        DrawingToolStates::DrawColorRed,
        DrawingToolStates::DrawColorGreen,
        DrawingToolStates::DrawColorBlue,
        DrawingToolStates::DrawColorCyan,
        DrawingToolStates::DrawColorOrange,
        DrawingToolStates::PublishToProjector,
        DrawingToolStates::DrawShowHelp
    };
    DrawingToolStates m_stm_state;

    // MOUSE inputs
    int m_i_mouse_pos_x;
    int m_i_mouse_pos_y;
    int m_i_mouse_pos_x_old;
    int m_i_mouse_pos_y_old;
    bool m_b_dialog_lock;
    // BUTTON inputs
    bool m_b_key_pressed_backspace;
    bool m_b_key_pressed_esc;
    bool m_b_key_pressed_ctrl;
    bool m_b_key_pressed_alt;
    bool m_b_key_pressed_shift;
    bool m_b_key_pressed_z;
    bool m_b_key_pressed_g;
    bool m_b_key_pressed_l;
    bool m_b_key_pressed_s;
    bool m_b_key_pressed_r;
    bool m_b_key_pressed_h;
    bool m_b_key_pressed_c;
    bool m_b_key_pressed_plus;
    bool m_b_key_pressed_minus;
    bool m_b_key_pressed_0;
    bool m_b_key_pressed_1;
    bool m_b_key_pressed_2;
    bool m_b_key_pressed_3;
    bool m_b_key_pressed_4;
    bool m_b_key_pressed_5;
    bool m_b_key_pressed_6;

    // VIEW MEMBERS
    float m_f_sketch_zoom;

    // SHAPE MEMBERS
    // Line
    int m_i_line_thickness;
    cv::Scalar m_scl_line_color;
    float m_f_line_length;
    float m_f_line_angle;
    // Scroll
    int m_i_scroll_offset_x;
    int m_i_scroll_offset_y;
    int m_i_scroll_start_x;
    int m_i_scroll_start_y;
    int m_i_scroll_delta_x;
    int m_i_scroll_delta_y;
    // Line
    int m_i_line_pt1_x;
    int m_i_line_pt1_y;
    int m_i_line_pt2_x;
    int m_i_line_pt2_y;
    // Circle
    int m_i_circle_pt_x;
    int m_i_circle_pt_y;
    int m_i_circle_radius;
    float m_f_circle_radius;

public:
    WAISketch():m_hdl_it(m_hdl_node)
    {
        m_s_path_nodename=ros::this_node::getName();
        m_hdl_node.getParam(m_s_path_nodename+"/"+"F_NODE_SAMPLE_FREQUENCY",F_NODE_SAMPLE_FREQUENCY);
        m_hdl_node.getParam(m_s_path_nodename+"/"+"S_SKETCH_FORMAT",S_SKETCH_FORMAT);
        m_hdl_node.getParam(m_s_path_nodename+"/"+"I_SKETCH_DPI",I_SKETCH_DPI);
        m_hdl_node.getParam(m_s_path_nodename+"/"+"S_SKETCH_FILE_NAME",S_SKETCH_FILE_NAME);
        m_pub_img_sketch=m_hdl_it.advertise("projector/image_raw",1);

        // Init state machine
        m_stm_state=drawing_tool_states[0];

        // Init members
        m_i_sketch_resolution_x=0;
        m_i_sketch_resolution_y=0;
        m_i_sketch_roi_offset_x=0;
        m_i_sketch_roi_offset_y=0;
        m_i_sketch_border_distance=0;
        m_i_sketch_grid_distance=0;
        m_i_sketch_window_size_x=0;
        m_i_sketch_window_size_y=0;
        m_i_sketch_gui_element_size=0;
        f_sketch_zoom_scale=0;
        m_b_sketch_grid_enable=true;
        m_b_dialog_lock=false;

        m_i_line_thickness=2;
        m_scl_line_color=cv::Scalar(255,255,255);
        m_f_line_length=0.0f;
        m_f_line_angle=0.0f;
        m_f_circle_radius=0.0f;
        m_f_sketch_zoom=1.0f;

        m_b_key_pressed_backspace=false;
        m_b_key_pressed_esc=false;
        m_b_key_pressed_ctrl=false;
        m_b_key_pressed_alt=false;
        m_b_key_pressed_shift=false;
        m_b_key_pressed_z=false;
        m_b_key_pressed_g=false;
        m_b_key_pressed_l=false;
        m_b_key_pressed_s=false;
        m_b_key_pressed_r=false;
        m_b_key_pressed_h=false;
        m_b_key_pressed_c=false;
        m_b_key_pressed_plus=false;
        m_b_key_pressed_minus=false;
        m_b_key_pressed_1=false;
        m_b_key_pressed_2=false;
        m_b_key_pressed_3=false;
        m_b_key_pressed_4=false;
        m_b_key_pressed_5=false;
        m_b_key_pressed_6=false;

        // Init helper images (with default size!)
        m_mat_sketch_window=cv::Mat(m_i_sketch_window_size_y,m_i_sketch_window_size_x,CV_8UC3);
        m_mat_sketch_grid=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3);
        m_mat_background=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3);
        m_mat_sketch_zoomed=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3);

        // Init main CV window
        m_s_sketch_window_name="WAI Sketch Tool - ";
        m_s_sketch_window_quick_info="No quick info available!";
        m_s_sketch_window_title=m_s_sketch_window_name+m_s_sketch_window_quick_info;
        cv::namedWindow(m_s_sketch_window_name,cv::WINDOW_AUTOSIZE);
        cv::waitKey(1);
        cv::moveWindow(m_s_sketch_window_name,40,25);
        cv::waitKey(1);
        cv::setWindowTitle(m_s_sketch_window_name,m_s_sketch_window_title);
        cv::setMouseCallback(m_s_sketch_window_name,onWAISketchMouseEvents,this);

        //cv::namedWindow("Test Window",cv::WINDOW_AUTOSIZE);
    }
    ~WAISketch()
    {
        if(CheckIfSketchWindowIsOpen())
        {
            cv::destroyWindow(m_s_sketch_window_name);
        }
    }

    void InitializeSketch()
    {
        // Setup for 150DPI
        if(I_SKETCH_DPI==150)
        {
            if(S_SKETCH_FORMAT.compare("A3_PORTRAIT")==0)
            {
                m_i_sketch_resolution_x=1754;
                m_i_sketch_resolution_y=2480;
            }
            else if(S_SKETCH_FORMAT.compare("A3_LANDSCAPE")==0)
            {
                m_i_sketch_resolution_x=2480;
                m_i_sketch_resolution_y=1754;
            }
            else if(S_SKETCH_FORMAT.compare("A4_PORTRAIT")==0)
            {
                m_i_sketch_resolution_x=1240;
                m_i_sketch_resolution_y=1754;
            }
            else if(S_SKETCH_FORMAT.compare("A4_LANDSCAPE")==0)
            {
                m_i_sketch_resolution_x=1754;
                m_i_sketch_resolution_y=1240;
            }
            else if(S_SKETCH_FORMAT.compare("A5_PORTRAIT")==0)
            {
                m_i_sketch_resolution_x=874;
                m_i_sketch_resolution_y=1240;
            }
            else if(S_SKETCH_FORMAT.compare("A5_LANDSCAPE")==0)
            {
                m_i_sketch_resolution_x=1240;
                m_i_sketch_resolution_y=874;
            }
            else if(S_SKETCH_FORMAT.compare("CUSTOM_720P")==0)
            {
                m_i_sketch_resolution_x=1280;
                m_i_sketch_resolution_y=720;
            }
            else
            {
                // Default setting with A4 portrait @ 150DPI
                m_i_sketch_resolution_x=1280;
                m_i_sketch_resolution_y=720;
            }
        }
        else if(I_SKETCH_DPI==300)
        {
            // ...
        }
        else
        {
            // Default setting with A4 portrait @ 150DPI
            m_i_sketch_resolution_x=1240;
            m_i_sketch_resolution_y=1754;
        }

        m_i_sketch_border_distance=50;
        m_i_sketch_grid_distance=30;
        m_i_sketch_gui_element_size=20;
        f_sketch_zoom_scale=1.0;

        RedrawMainWindow();
        RedrawBackgroundImage();
        RedrawGrid();
        ClearLayers();
        RedrawZoomedImage();
        RedrawGUI();
        ResetScroll();

        // Load image if filename passed as argument
        if(S_SKETCH_FILE_NAME.compare("")!=0)
        {
            m_mat_sketch_loaded=cv::imread(S_SKETCH_FILE_NAME);
            m_b_sketch_grid_enable=false;
        }

        RedrawSketchFull(true);
    }
    void RedrawMainWindow()
    {
        m_i_sketch_window_size_x=m_i_sketch_resolution_x+2*m_i_sketch_border_distance;
        m_i_sketch_window_size_y=m_i_sketch_resolution_y+2*m_i_sketch_border_distance;
        m_mat_sketch_window=cv::Mat(m_i_sketch_window_size_y,m_i_sketch_window_size_x,CV_8UC3);
        m_mat_sketch_window.setTo(cv::Scalar(192,192,192));
        cv::rectangle(m_mat_sketch_window,
                      cv::Point(m_i_sketch_border_distance,m_i_sketch_border_distance),
                      cv::Point(m_mat_sketch_window.cols-m_i_sketch_border_distance,m_mat_sketch_window.rows-m_i_sketch_border_distance),
                      cv::Scalar(0,0,0),
                      3);
        cv::imshow(m_s_sketch_window_name,m_mat_sketch_window);
    }
    void RedrawBackgroundImage()
    {
        m_mat_background=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3);
        m_mat_background.setTo(cv::Scalar(0,0,0));
    }
    void RedrawGrid()
    {
        m_mat_sketch_grid=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3);
        m_mat_sketch_grid.setTo(cv::Scalar(0,0,0));

        // Draw border
        cv::rectangle(m_mat_sketch_grid,cv::Point(0,0),cv::Point(m_mat_sketch_grid.cols-1,m_mat_sketch_grid.rows-1),cv::Scalar(128,128,128),3);

        // Draw horizontal grid lines
        for(int i=m_i_sketch_grid_distance;i<=(m_mat_sketch_grid.rows-m_i_sketch_grid_distance);i=i+m_i_sketch_grid_distance)
        {
                cv::line(m_mat_sketch_grid,
                         cv::Point(m_i_sketch_grid_distance,i),
                         cv::Point(m_mat_sketch_grid.cols-m_i_sketch_grid_distance,i),
                         cv::Scalar(128,128,128),1);
        }
        // Draw vertical grid lines
        for(int j=m_i_sketch_grid_distance;j<=(m_mat_sketch_grid.cols-m_i_sketch_grid_distance);j=j+m_i_sketch_grid_distance)
        {
                cv::line(m_mat_sketch_grid,
                         cv::Point(j,m_i_sketch_grid_distance),
                         cv::Point(j,m_mat_sketch_grid.rows-m_i_sketch_grid_distance),
                         cv::Scalar(128,128,128),1);
        }

        // Add watermark
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::stringstream sst_date_time;
        sst_date_time<<std::put_time(&tm, "Sketch %d-%m-%Y %H-%M-%S ") << S_SKETCH_FORMAT << "@" << I_SKETCH_DPI<< "DPI)";
        cv::putText(m_mat_sketch_grid,sst_date_time.str(),cv::Point(60,75),cv::FONT_HERSHEY_DUPLEX,1.0,cv::Scalar(128,128,128),1);
    }
    void ClearLayers()
    {
        m_vec_sketch_layers.clear();
        m_mat_sketch_inner_roi_base=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3);
        m_mat_sketch_inner_roi=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3); // ROI as part of window with all images stacked in line
        m_mat_sketch_current=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3); // Current sketch, potentially added to layers
    }
    void RedrawZoomedImage()
    {
        m_mat_sketch_zoomed=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3);
        m_mat_sketch_zoomed.setTo(cv::Scalar(0,0,0));
        m_mat_sketch_inner_roi_zoomed=cv::Mat(m_i_sketch_resolution_y,m_i_sketch_resolution_x,CV_8UC3);
        m_mat_sketch_inner_roi_zoomed.setTo(cv::Scalar(0,0,0));
    }
    void RedrawGUI()
    {

    }
    void ResetScroll()
    {
        m_i_scroll_offset_x=0;
        m_i_scroll_offset_y=0;
        m_i_scroll_start_x=0;
        m_i_scroll_start_y=0;
        m_i_scroll_delta_x=0;
        m_i_scroll_delta_y=0;
    }

    void AddCurrentSketchToLayers()
    {
        m_vec_sketch_layers.push_back(m_mat_sketch_current.clone());
        m_mat_sketch_current.setTo(cv::Scalar(0,0,0));
        RedrawSketchFull(true);
    }

    void UpdateQuickInfo()
    {
        // Update QUICK INFO
        sst_quick_info.str("");
        sst_quick_info << std::setprecision(5) <<
                          "Zoom: x" << m_f_sketch_zoom <<
                          " , Px: " << m_i_mouse_pos_x << " (=" << m_i_mouse_pos_x/(float)I_SKETCH_DPI*2.54 << "cm)" <<
                          " , Py: " << m_i_mouse_pos_y << " (=" << m_i_mouse_pos_y/(float)I_SKETCH_DPI*2.54 << "cm)" <<
                          " , l: " << m_f_line_length << " (=" << m_f_line_length/(float)I_SKETCH_DPI*2.54 << "cm)" <<
                          " , r: " << m_f_circle_radius << " (=" << m_f_circle_radius/(float)I_SKETCH_DPI*2.54 << "cm)" <<
                          " , α: " << m_f_line_angle << "DEG";
        m_s_sketch_window_quick_info=sst_quick_info.str();
        m_s_sketch_window_title=m_s_sketch_window_name+m_s_sketch_window_quick_info;
        cv::setWindowTitle(m_s_sketch_window_name,m_s_sketch_window_title);
    }
    void UpdateROIBase()
    {
        // Update SKETCH ROI Base
        m_mat_sketch_inner_roi_base.setTo(cv::Scalar(0,0,0));
        m_mat_sketch_inner_roi_base=m_mat_sketch_inner_roi_base+m_mat_background; // Add background and grid
        if(m_b_sketch_grid_enable==true)
        {
            m_mat_sketch_inner_roi_base+=m_mat_sketch_grid;
        }
        else
        {
            // Do nothing...
        }

        // Add "old" layers OVER grid and...
        // Add current sketch to inner ROI of full sketch drawing window
        for(int i=0;i<m_vec_sketch_layers.size();i++)
        {
            m_mat_sketch_inner_roi_base=m_mat_sketch_inner_roi_base+m_vec_sketch_layers[i];
        }
    }
    void UpdateROI()
    {
        m_mat_sketch_inner_roi=m_mat_sketch_inner_roi_base+m_mat_sketch_current;
    }
    void UpdateLoadedImage()
    {
        // Update LOADED IMAGE
        if(m_mat_sketch_loaded.empty())
        {
        }
        else
        {
            cv::Mat m_mat_sketch_loaded_zoomed;
            cv::resize(m_mat_sketch_loaded,m_mat_sketch_loaded_zoomed,cv::Size(m_mat_sketch_inner_roi.cols,m_mat_sketch_inner_roi.rows));
            m_mat_sketch_inner_roi=m_mat_sketch_inner_roi+m_mat_sketch_loaded_zoomed;
        }
    }
    void UpdateZoom()
    {
        // Update ZOOM
        m_mat_sketch_zoomed.setTo(cv::Scalar(0,0,0)); // Reset zoomed sketch
        cv::resize(m_mat_sketch_inner_roi,m_mat_sketch_inner_roi_zoomed,cv::Size(),m_f_sketch_zoom,m_f_sketch_zoom);
        m_i_sketch_roi_offset_x=abs(m_mat_sketch_zoomed.cols-m_mat_sketch_inner_roi_zoomed.cols)/2;
        m_i_sketch_roi_offset_y=abs(m_mat_sketch_zoomed.rows-m_mat_sketch_inner_roi_zoomed.rows)/2;
        if(m_f_sketch_zoom<1.0)
        {
            // Fit zoomed-out sketch window into inner sketch drawing surface
            m_mat_sketch_inner_roi_zoomed.copyTo(m_mat_sketch_zoomed(cv::Rect(
                                        m_i_sketch_roi_offset_x,
                                        m_i_sketch_roi_offset_y,
                                        m_mat_sketch_inner_roi_zoomed.cols,
                                        m_mat_sketch_inner_roi_zoomed.rows)));
        }
        else
        {
            if(abs(m_i_scroll_delta_x) <= m_i_sketch_roi_offset_x &&
               abs(m_i_scroll_delta_y) <= m_i_sketch_roi_offset_y )
            {
                m_mat_sketch_zoomed=m_mat_sketch_inner_roi_zoomed(cv::Rect(
                                        m_i_sketch_roi_offset_x+m_i_scroll_delta_x,
                                        m_i_sketch_roi_offset_y+m_i_scroll_delta_y,
                                        m_mat_sketch_zoomed.cols,
                                        m_mat_sketch_zoomed.rows));
            }
            else
            {
                if(m_i_scroll_delta_x < -m_i_sketch_roi_offset_x) m_i_scroll_delta_x=-m_i_sketch_roi_offset_x;
                if(m_i_scroll_delta_x > m_i_sketch_roi_offset_x) m_i_scroll_delta_x=m_i_sketch_roi_offset_x;
                if(m_i_scroll_delta_y < -m_i_sketch_roi_offset_y) m_i_scroll_delta_y=-m_i_sketch_roi_offset_y;
                if(m_i_scroll_delta_y > m_i_sketch_roi_offset_y) m_i_scroll_delta_y=m_i_sketch_roi_offset_y;
            }
        }
        //cv::imshow("Test Window",m_mat_sketch_inner_roi);
    }
    void UpdateSketchWindowFull()
    {
        // Update full SKETCH WINDOW
        cv::Mat mat_window_roi(m_mat_sketch_window,cv::Rect(m_i_sketch_border_distance,m_i_sketch_border_distance,m_mat_sketch_grid.cols,m_mat_sketch_grid.rows));
        m_mat_sketch_zoomed.copyTo(mat_window_roi);
        cv::imshow(m_s_sketch_window_name,m_mat_sketch_window);
    }

    void RedrawSketchFull(bool b_redraw_roi_base=false)
    {
        UpdateQuickInfo();
        if(b_redraw_roi_base==true) UpdateROIBase();
        UpdateROI();
        UpdateLoadedImage();
        UpdateZoom();
        UpdateSketchWindowFull();
    }

    bool CheckIfSketchWindowIsOpen()
    {
        if(cv::getWindowProperty(m_s_sketch_window_name,cv::WND_PROP_AUTOSIZE)!=-1) return true;
        else return false;
    }

    static void onWAISketchMouseEvents(int event,int x,int y,int flags,void* userdata)
    {
        WAISketch* tolskt = reinterpret_cast<WAISketch*>(userdata);
        tolskt->onWAISketchMouseInputs(event,x,y,flags);
    }
    void onWAISketchMouseInputs(int event,int x,int y,int flags)
    {
        if(m_f_sketch_zoom>=1.0) // If zooming in, ADD ROI-Offset...
        {
            m_i_mouse_pos_x=float(x-m_i_sketch_border_distance+m_i_sketch_roi_offset_x+m_i_scroll_offset_x)/m_f_sketch_zoom;
            m_i_mouse_pos_y=float(y-m_i_sketch_border_distance+m_i_sketch_roi_offset_y+m_i_scroll_offset_y)/m_f_sketch_zoom;
        }
        else // Otherwise, if zoomed out, SUBSTRACT ROI-Offset...
        {
            m_i_mouse_pos_x=float(x-m_i_sketch_border_distance-m_i_sketch_roi_offset_x+m_i_scroll_offset_x)/m_f_sketch_zoom;
            m_i_mouse_pos_y=float(y-m_i_sketch_border_distance-m_i_sketch_roi_offset_y+m_i_scroll_offset_y)/m_f_sketch_zoom;
        }
        //ROS_WARN("RES x: %3d | x: %3d | Border x: %3d | ROI Offset x: %3d | Scroll Offset x: %3d | zoom: %3.3f",
        //         m_i_mouse_pos_x,x,m_i_sketch_border_distance,m_i_sketch_roi_offset_x,m_i_scroll_offset_x,m_f_sketch_zoom);

        //===================
        // STATE TRANSITIONS
        //===================
        if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_z==true)
        {
            m_stm_state=DrawingToolStates::DrawZoomReset;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_l==true)
        {
            m_stm_state=DrawingToolStates::DrawLoadImage;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_s==true)
        {
            m_stm_state=DrawingToolStates::DrawSaveImage;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_g==true)
        {
            m_stm_state=DrawingToolStates::DrawGridToggle;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_0==true)
        {
            m_stm_state=DrawingToolStates::DrawColorBlack;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_1==true)
        {
            m_stm_state=DrawingToolStates::DrawColorWhite;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_2==true)
        {
            m_stm_state=DrawingToolStates::DrawColorRed;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_3==true)
        {
            m_stm_state=DrawingToolStates::DrawColorGreen;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_4==true)
        {
            m_stm_state=DrawingToolStates::DrawColorBlue;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_5==true)
        {
            m_stm_state=DrawingToolStates::DrawColorCyan;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_6==true)
        {
            m_stm_state=DrawingToolStates::DrawColorOrange;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_backspace==true)
        {
            m_stm_state=DrawingToolStates::DrawHistoryBack;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_minus==true)
        {
            m_stm_state=DrawingToolStates::DrawThicknessDecrease;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_plus==true)
        {
            m_stm_state=DrawingToolStates::DrawThicknessIncrease;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_esc==true)
        {
            m_stm_state=DrawingToolStates::PublishToProjector;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_h==true)
        {
            m_stm_state=DrawingToolStates::DrawShowHelp;
        }
        // Mouse button LEFT with Key C (--> Draw CIRCLE)
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_c==true)
        {
            m_stm_state=DrawingToolStates::DrawCircleSelectP1;
        }
        else if(m_stm_state==DrawingToolStates::DrawCirclePreview &&
                event==cv::EVENT_LBUTTONUP)
        {
            m_stm_state=DrawingToolStates::DrawCircleSelectP2;
        }
        // Mouse button LEFT with Key R (--> Draw RECTANGLE)
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN && m_b_key_pressed_r==true)
        {
            m_stm_state=DrawingToolStates::DrawRectangleSelectP1;
        }
        else if(m_stm_state==DrawingToolStates::DrawRectanglePreview &&
                event==cv::EVENT_LBUTTONUP)
        {
            m_stm_state=DrawingToolStates::DrawRectangleSelectP2;
        }
        // Mouse button LEFT with Key Nothing
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_LBUTTONDOWN)
        {
            // Prevent wrong point for first iteration
            m_i_mouse_pos_x_old=m_i_mouse_pos_x;
            m_i_mouse_pos_y_old=m_i_mouse_pos_y;

            m_stm_state=DrawingToolStates::DrawCurveSelect;
        }
        else if(m_stm_state==DrawingToolStates::DrawCurveSelect &&
                event==cv::EVENT_LBUTTONUP)
        {
            m_stm_state=DrawingToolStates::DrawCurveDeselect;
        }
        // Mouse button MIDDLE with Key Nothing
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_MBUTTONDOWN &&
                m_f_sketch_zoom>1.0) // Pan with "smaller" ROI of the full sketch only!
        {
            m_stm_state=DrawingToolStates::DrawScrollSelect;
        }
        else if(m_stm_state==DrawingToolStates::DrawScrollPreview &&
                event==cv::EVENT_MBUTTONUP)
        {
            m_stm_state=DrawingToolStates::DrawScrollDeselect;
        }
        // RIGHT button with Key Nothing
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_RBUTTONDOWN && m_b_key_pressed_r==true)
        {
            // Left open for more interactions
        }
        else if(m_stm_state==DrawingToolStates::DrawRectanglePreview &&
                event==cv::EVENT_RBUTTONUP)
        {
            // Left open ...
        }
        // RIGHT Button with NO key
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                event==cv::EVENT_RBUTTONDOWN)
        {
            m_stm_state=DrawingToolStates::DrawLineSelectP1;
        }
        else if(m_stm_state==DrawingToolStates::DrawLinePreview &&
                event==cv::EVENT_RBUTTONUP)
        {
            m_stm_state=DrawingToolStates::DrawLineSelectP2;
        }
        // SCROLL WHEEL UP/DOWN
        else if(m_stm_state==DrawingToolStates::DrawNothing &&
                (event==10||event==11))
        {
            if(flags>0)
            {
                m_stm_state=DrawingToolStates::DrawZoomIn;
            }
            else
            {
                m_stm_state=DrawingToolStates::DrawZoomOut;
            }
        }
        else
        {
            // Do nothing...
        }


        //===================
        // HANDLE STATES
        //===================
        if(m_stm_state==DrawingToolStates::PublishToProjector)
        {
            PlaySoundClick();

            cv::Mat mat_to_projector=cv::Mat(m_mat_sketch_inner_roi.rows,1280,CV_8UC3);
            if(m_mat_sketch_inner_roi.rows<720)
            {
                cv::resize(m_mat_sketch_inner_roi,mat_to_projector,cv::Size(1280,m_mat_sketch_inner_roi.rows));
                cv::Mat mat_to_projector_roi(mat_to_projector,cv::Rect(0,0,1280,m_mat_sketch_inner_roi.rows));
                m_pub_img_sketch.publish( cv_bridge::CvImage(std_msgs::Header(),"bgr8",mat_to_projector_roi).toImageMsg() );
            }
            else
            {
                cv::resize(m_mat_sketch_inner_roi,mat_to_projector,cv::Size(1280,m_mat_sketch_inner_roi.rows));
                cv::Mat mat_to_projector_roi(mat_to_projector,cv::Rect(0,0,1280,720));
                m_pub_img_sketch.publish( cv_bridge::CvImage(std_msgs::Header(),"bgr8",mat_to_projector_roi).toImageMsg() );
            }

            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawGridToggle)
        {
            PlaySoundClick();

            m_b_sketch_grid_enable=!m_b_sketch_grid_enable;
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawShowHelp
                && m_b_dialog_lock==false)
        {
            PlaySoundClick();

            m_b_dialog_lock=true;

            QMessageBox::information(NULL,"Help",
                                     "<b>HELP</b><br>\
                                     Mouse LBUTTON + Key H ... Show this help panel.<br>\
                                     Mouse LBUTTON + Shift ... Publish to projector.<br>\
                                     Mouse LBUTTON + Key L ... Image Load.<br>\
                                     Mouse LBUTTON + Key S ... Image Save.<br>\
                                     Mouse LBUTTON + Back  ... Undo.<br>\
                                     Mouse LBUTTON + Key Z ... Zoom Reset.<br>\
                                     Mouse MBUTTON + Scroll... Zoom In/Out.<br>\
                                     Mouse MBUTTON + Move  ... View Pan.<br>\
                                     Mouse LBUTTON + Key G ... Grid Toggle.<br>\
                                     Mouse LBUTTON + Key + ... Line Thickness Increase.<br>\
                                     Mouse LBUTTON + Key - ... Line Thickness Decrease.<br>\
                                     Mouse LBUTTON + Key R ... Draw Rectangle.<br>\
                                     Mouse LBUTTON + Key C ... Draw Circle.<br>\
                                     Mouse LBUTTON + Key 1 ... Color Select White.<br>\
                                     Mouse LBUTTON + Key 2 ... Color Select Red.<br>\
                                     Mouse LBUTTON + Key 3 ... Color Select Green.<br>\
                                     Mouse LBUTTON + Key 4 ... Color Select Blue.<br>\
                                     Mouse LBUTTON + Key 5 ... Color Select Cyan.<br>\
                                     Mouse LBUTTON + Key 6 ... Color Select Orange.<br>\
                                     Mouse LBUTTON + No key... Draw curve.<br>\
                                     Mouse RBUTTON + No key... Draw line.<br>");

            m_stm_state=DrawingToolStates::DrawNothing;
            m_b_dialog_lock=false;
        }
        else if(m_stm_state==DrawingToolStates::DrawLoadImage
                && m_b_dialog_lock==false)
        {
            PlaySoundClick();

            // Load sketch...
            m_b_dialog_lock=true;

            qst_img_background_path=QFileDialog::getOpenFileName(NULL,"LOAD Sketch","/home/ias","PNG Files (*.png)",0,QFileDialog::DontUseNativeDialog);
            if(qst_img_background_path.isNull()==false)
            {
                m_mat_sketch_loaded=cv::imread(qst_img_background_path.toStdString());
                //cv::bitwise_not(mat_sketch_loaded,mat_sketch_loaded);
                m_b_sketch_grid_enable=false;
            }
            m_stm_state=DrawingToolStates::DrawNothing;
            m_b_dialog_lock=false;
        }
        else if(m_stm_state==DrawingToolStates::DrawSaveImage
                && m_b_dialog_lock==false)
        {
            PlaySoundClick();

            m_b_dialog_lock=true;

            for(int i=0;i<10;i++)cv::waitKey(100);

            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << "/home/ias/" << std::put_time(&tm,"%Y_%m_%d_%H_%M") << "_oa_sketch.png";
            QString qst_filename=QFileDialog::getSaveFileName(NULL,"Save File",
                                    QString::fromStdString(oss.str()),
                                    "Images (*.png)",0,QFileDialog::DontUseNativeDialog);
            if(qst_filename.compare("")!=0)
            {
                cv::imwrite(qst_filename.toStdString(),m_mat_sketch_zoomed);
            }

            m_stm_state=DrawingToolStates::DrawNothing;
            m_b_dialog_lock=false;
        }
        else if(m_stm_state==DrawingToolStates::DrawColorBlack)
        {
            PlaySoundClick();

            m_scl_line_color=cv::Scalar(0,0,0);
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawColorWhite)
        {
            PlaySoundClick();

            m_scl_line_color=cv::Scalar(255,255,255);
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawColorRed)
        {
            PlaySoundClick();

            m_scl_line_color=cv::Scalar(0,0,255);
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawColorGreen)
        {
            PlaySoundClick();

            m_scl_line_color=cv::Scalar(0,255,0);
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawColorBlue)
        {
            PlaySoundClick();

            m_scl_line_color=cv::Scalar(255,0,0);
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawColorCyan)
        {
            PlaySoundClick();

            m_scl_line_color=cv::Scalar(255,255,0);
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawColorOrange)
        {
            PlaySoundClick();

            m_scl_line_color=cv::Scalar(0,165,255);
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawHistoryBack)
        {
            PlaySoundClick();

            if(m_vec_sketch_layers.size()>0)
            {
                m_vec_sketch_layers.resize(m_vec_sketch_layers.size()-1);
                RedrawSketchFull(true);
            }
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawThicknessDecrease)
        {
            PlaySoundClick();

            if(m_i_line_thickness>=3) m_i_line_thickness-=2;
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawThicknessIncrease)
        {
            PlaySoundClick();

            if(m_i_line_thickness<=7) m_i_line_thickness+=2;
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        // SCROLL WINDOW
        else if(m_stm_state==DrawingToolStates::DrawScrollSelect)
        {
            m_i_scroll_start_x=m_i_mouse_pos_x;
            m_i_scroll_start_y=m_i_mouse_pos_y;
            m_stm_state=DrawingToolStates::DrawScrollPreview;
        }
        else if(m_stm_state==DrawingToolStates::DrawScrollPreview)
        {
            m_i_scroll_delta_x=m_i_scroll_offset_x+m_i_mouse_pos_x-m_i_scroll_start_x;
            m_i_scroll_delta_y=m_i_scroll_offset_y+m_i_mouse_pos_y-m_i_scroll_start_y;
        }
        else if(m_stm_state==DrawingToolStates::DrawScrollDeselect)
        {
            m_i_scroll_offset_x=m_i_scroll_delta_x;
            m_i_scroll_offset_y=m_i_scroll_delta_y;
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        // DRAW CURVE
        else if(m_stm_state==DrawingToolStates::DrawCurveSelect)
        {
            cv::line(m_mat_sketch_current,
                     cv::Point(m_i_mouse_pos_x_old,m_i_mouse_pos_y_old),
                     cv::Point(m_i_mouse_pos_x,m_i_mouse_pos_y),
                     m_scl_line_color,
                     m_i_line_thickness);
        }
        else if(m_stm_state==DrawingToolStates::DrawCurveDeselect)
        {
            AddCurrentSketchToLayers();
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        // DRAW LINE
        else if(m_stm_state==DrawingToolStates::DrawLineSelectP1)
        {
            m_i_line_pt1_x=m_i_mouse_pos_x;
            m_i_line_pt1_y=m_i_mouse_pos_y;
            m_stm_state=DrawingToolStates::DrawLinePreview;
        }
        else if(m_stm_state==DrawingToolStates::DrawLinePreview)
        {
            m_mat_sketch_current.setTo(cv::Scalar(0,0,0)); // Remove old preview lines

            // Calculate line angle
            if((m_i_mouse_pos_x-m_i_line_pt1_x)!=0)
            {
                m_f_line_length=pow( pow(float(m_i_mouse_pos_y-m_i_line_pt1_y),2.0) + pow(float(m_i_mouse_pos_x-m_i_line_pt1_x),2.0) ,0.5);
                m_f_line_angle=atan(-float(m_i_mouse_pos_y-m_i_line_pt1_y)/float(m_i_mouse_pos_x-m_i_line_pt1_x))*180.0/M_PI;
            }
            // Redraw preview line
            cv::line(m_mat_sketch_current,
                     cv::Point(m_i_line_pt1_x,m_i_line_pt1_y),
                     cv::Point(m_i_mouse_pos_x,m_i_mouse_pos_y),
                     m_scl_line_color,
                     m_i_line_thickness);
        }
        else if(m_stm_state==DrawingToolStates::DrawLineSelectP2)
        {
            cv::line(m_mat_sketch_current,
                     cv::Point(m_i_line_pt1_x,m_i_line_pt1_y),
                     cv::Point(m_i_mouse_pos_x,m_i_mouse_pos_y),
                     m_scl_line_color,
                     m_i_line_thickness);

            AddCurrentSketchToLayers();
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        // DRAW RECT
        else if(m_stm_state==DrawingToolStates::DrawRectangleSelectP1)
        {
            m_i_line_pt1_x=m_i_mouse_pos_x;
            m_i_line_pt1_y=m_i_mouse_pos_y;
            m_stm_state=DrawingToolStates::DrawRectanglePreview;
        }
        else if(m_stm_state==DrawingToolStates::DrawRectanglePreview)
        {
            m_mat_sketch_current.setTo(cv::Scalar(0,0,0)); // Remove old preview rectangles
            cv::rectangle(m_mat_sketch_current,
                     cv::Point(m_i_line_pt1_x,m_i_line_pt1_y),
                     cv::Point(m_i_mouse_pos_x,m_i_mouse_pos_y),
                     m_scl_line_color,
                     m_i_line_thickness);
        }
        else if(m_stm_state==DrawingToolStates::DrawRectangleSelectP2)
        {
            cv::rectangle(m_mat_sketch_current,
                     cv::Point(m_i_line_pt1_x,m_i_line_pt1_y),
                     cv::Point(m_i_mouse_pos_x,m_i_mouse_pos_y),
                     m_scl_line_color,
                     m_i_line_thickness);

            AddCurrentSketchToLayers();
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        // Draw CIRCLE
        else if(m_stm_state==DrawingToolStates::DrawCircleSelectP1)
        {
            m_i_circle_pt_x=m_i_mouse_pos_x;
            m_i_circle_pt_y=m_i_mouse_pos_y;
            m_stm_state=DrawingToolStates::DrawCirclePreview;
        }
        else if(m_stm_state==DrawingToolStates::DrawCirclePreview)
        {
            m_mat_sketch_current.setTo(cv::Scalar(0,0,0)); // Remove old preview circles

            // Calculate circle radius
            m_f_circle_radius=pow( pow(float(m_i_mouse_pos_x-m_i_circle_pt_x),2.0)+pow(float(m_i_mouse_pos_y-m_i_circle_pt_y),2.0) ,0.5);
            m_i_circle_radius=int(m_f_circle_radius);
            cv::circle(m_mat_sketch_current,
                     cv::Point(m_i_circle_pt_x,m_i_circle_pt_y),
                     m_i_circle_radius,
                     m_scl_line_color,
                     m_i_line_thickness);
        }
        else if(m_stm_state==DrawingToolStates::DrawCircleSelectP2)
        {
            cv::circle(m_mat_sketch_current,
                     cv::Point(m_i_circle_pt_x,m_i_circle_pt_y),
                     m_i_circle_radius,
                     m_scl_line_color,
                     m_i_line_thickness);

            AddCurrentSketchToLayers();
            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawZoomOut)
        {
            if(m_f_sketch_zoom>=0.55) m_f_sketch_zoom-=0.05;
            ResetScroll();

            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawZoomIn)
        {
            if(m_f_sketch_zoom<=1.95) m_f_sketch_zoom+=0.05;
            ResetScroll();

            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawZoomReset)
        {
            m_f_sketch_zoom=1.0;
            ResetScroll();

            m_stm_state=DrawingToolStates::DrawNothing;
        }
        else if(m_stm_state==DrawingToolStates::DrawNothing)
        {
            // Do nothing...
        }

        // Store latest mouse pos xy
        m_i_mouse_pos_x_old=m_i_mouse_pos_x;
        m_i_mouse_pos_y_old=m_i_mouse_pos_y;
    }

    void CheckForInputsKeyboard()
    {
        m_c_sketch_key_pressed=(unsigned char)cv::waitKey(int(1.0/(F_NODE_SAMPLE_FREQUENCY)*1000.0));
        //ROS_WARN("Key code: %d",m_c_sketch_key_pressed);

        if(m_c_sketch_key_pressed==8) m_b_key_pressed_backspace=true;
        else m_b_key_pressed_backspace=false;

        if(m_c_sketch_key_pressed==27) m_b_key_pressed_esc=true;
        else m_b_key_pressed_esc=false;

        if(m_c_sketch_key_pressed==233) m_b_key_pressed_alt=true;
        else m_b_key_pressed_alt=false;

        if(m_c_sketch_key_pressed==227) m_b_key_pressed_ctrl=true;
        else m_b_key_pressed_ctrl=false;

        if(m_c_sketch_key_pressed==225) m_b_key_pressed_shift=true;
        else m_b_key_pressed_shift=false;

        if(m_c_sketch_key_pressed==122) m_b_key_pressed_z=true;
        else m_b_key_pressed_z=false;

        if(m_c_sketch_key_pressed==103) m_b_key_pressed_g=true;
        else m_b_key_pressed_g=false;

        if(m_c_sketch_key_pressed==108) m_b_key_pressed_l=true;
        else m_b_key_pressed_l=false;

        if(m_c_sketch_key_pressed==115) m_b_key_pressed_s=true;
        else m_b_key_pressed_s=false;

        if(m_c_sketch_key_pressed==114) m_b_key_pressed_r=true;
        else m_b_key_pressed_r=false;

        if(m_c_sketch_key_pressed==104) m_b_key_pressed_h=true;
        else m_b_key_pressed_h=false;

        if(m_c_sketch_key_pressed==99) m_b_key_pressed_c=true;
        else  m_b_key_pressed_c=false;

        if(m_c_sketch_key_pressed==45) m_b_key_pressed_minus=true;
        else  m_b_key_pressed_minus=false;

        if(m_c_sketch_key_pressed==43) m_b_key_pressed_plus=true;
        else  m_b_key_pressed_plus=false;

        if(m_c_sketch_key_pressed==48) m_b_key_pressed_0=true;
        else  m_b_key_pressed_0=false;

        if(m_c_sketch_key_pressed==49) m_b_key_pressed_1=true;
        else  m_b_key_pressed_1=false;

        if(m_c_sketch_key_pressed==50) m_b_key_pressed_2=true;
        else  m_b_key_pressed_2=false;

        if(m_c_sketch_key_pressed==51) m_b_key_pressed_3=true;
        else  m_b_key_pressed_3=false;

        if(m_c_sketch_key_pressed==52) m_b_key_pressed_4=true;
        else  m_b_key_pressed_4=false;

        if(m_c_sketch_key_pressed==53) m_b_key_pressed_5=true;
        else  m_b_key_pressed_5=false;

        if(m_c_sketch_key_pressed==54) m_b_key_pressed_6=true;
        else  m_b_key_pressed_6=false;
    }

    void PlaySoundClick()
    {
        // Prompt response is always non-blocking
        std::string s_command="canberra-gtk-play -V 0.0 -f /home/ias/catkin_ws/src/wai_world/wai_oa/wai_oa_tools/wai_sketch/resources/click.wav &";
        int i_retval=system(s_command.c_str());
    }

    void run()
    {
        InitializeSketch();

        while(ros::ok())
        {
            // Check if OpenCV window is valid and opened!
            if(CheckIfSketchWindowIsOpen())
            {
                // Redraw full sketch
                RedrawSketchFull();

                // Check for keyboard inputs
                CheckForInputsKeyboard();
            }
            else
            {
                return;
            }

            ros::spinOnce();
        }
    }
};
