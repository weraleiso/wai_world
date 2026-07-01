#ifndef WAI_OA_AUDIENCE_H
#define WAI_OA_AUDIENCE_H

/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<iostream>
#include<math.h>
#include<vector>
#include<queue>
#include<string>
#include<string.h>
#include<sstream>
#include<fstream>

#include<ros/ros.h>
#include<std_msgs/Empty.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>

#include<tf/transform_datatypes.h>

#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<view_controller_msgs/CameraPlacement.h>

#include<wai_oa_listener.h>



/////////////////////////////////////////////////
/// Class definition of WAIOAAudience
/////////////////////////////////////////////////
class WAIOAAudience
{
    ros::NodeHandle* m_hdl_node;
    int m_i_audience_count_max;
    std::string m_s_audience_alias;
    int m_i_audience_eval_model;
    int m_i_audience_listeners_per_row;
    tf::Vector3 m_vc3_audience_offset;
    tf::Vector3 m_vc3_audience_spacing;

    std::vector<WAIOAListener*> vec_audience;

public:
    WAIOAAudience();
    ~WAIOAAudience();

    WAIOAListener GetAudienceID(int i_id);
    std::string GetAliasID(int i_id);
    void SetAliasID(int i_id,std::string s_alias,std::string s_alias_group);
    float GetScoresOverallFromAudienceID(int i_id);
    float GetFractionsOverallFromAudienceID(int i_id);
    void Initialize(ros::NodeHandle* hdl_node,
                    int i_audience_count_max,
                    int i_audience_listeners_per_row,
                    tf::Vector3 vc3_audience_offset,
                    tf::Vector3 vc3_audience_spacing,
                    std::string s_audience_alias,
                    int i_audience_eval_model,
                    float f_eval_part_score_max,
                    float f_eval_exam_score_max);
    /* Old interface for eval:
    std::string ImportEvalsIDSubperiodFromTextfile(std::string s_path_group,int i_eval_id,int i_eval_sub);
    */
    // With new interface for eval:
    void ImportEvalsGroupFromFileText(std::string s_path_group);
    std::string GetEvalsAudienceIDSubperiod(int i_eval_audience_id,int i_eval_audience_sub);
    float GetStatsAudienceID(int i_stats_audience_id);
    void ResetStatsAudienceID(int i_stats_audience_id);
    int GetAudienceCountMax();
    int GetAudienceListenersPerRow();
    tf::Vector3 GetAudiencePosFromID(int i_audience_id);
};

#endif //WAI_OA_AUDIENCE_H
