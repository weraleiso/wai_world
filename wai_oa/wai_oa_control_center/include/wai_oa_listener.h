#ifndef WAI_OA_LISTENER_H
#define WAI_OA_LISTENER_H

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



/////////////////////////////////////////////////
/// Helper class WAIOAListener
/////////////////////////////////////////////////

#define EVAL_COUNT_MAX_SUBPERIOD 16 // Maximum of 16 evaluations per subperiod
#define EVAL_SUBPERIODS 12 // Month-based subperiods
#define EVAL_PART_FRACTION_SIZE 8.0
#define EVAL_EXAM_FRACTION_SIZE 24.0
/*
#define EVAL_PART_WEIGHT 0.5 // Currently used for calculator model only!
#define EVAL_EXAM_WEIGHT 0.5 // Currently used for calculator model only!
*/

enum EvalModel
{
    EVAL_MODEL_CALCULATOR = 0,
    EVAL_MODEL_FRACTIONS_MOSAIC = 1
};

struct Eval
{
    int i_eval_id;
    std::string s_date;
        int i_year;
        int i_month;
        int i_day;
    int i_calendar_week;
    std::string s_weekday;
    std::string s_time;
        int i_hour;
        int i_min;
        int i_sec;
    int i_audience_id;
    std::string s_audience_alias;
    std::string s_group;
    std::string s_topic;
    std::string s_expertise;
    std::string s_expertise_level;
    std::string s_session;
    std::string s_type;
    std::string s_label;
    std::string s_score;
        float f_score;
    std::string s_score_max;
        float f_score_max;
    std::string s_relation;
        float f_relation;
    std::string s_comment;
};

class WAIOAListener
{
    ros::NodeHandle* m_hdl_node;
    ros::Publisher pub_hea_info_to_audience_id;
    ros::Publisher pub_hea_ping_to_audience_id;
    ros::Subscriber sub_hea_ping_from_audience_id;

    std_msgs::Header msg_hea_info_to_audience_id;
    std_msgs::Header msg_hea_ping_to_audience_id;
    geometry_msgs::PoseStamped m_pst_audience_pose;
    std::string m_s_alias_audience_id;
    std::string m_s_alias_group;

    float m_f_stats_ping;

    int m_i_audience_id;
    int m_i_eval_model;

    // New interface
    std::vector<Eval> m_vec_evals;

    // Old interface
    float m_f_eval_part_scores_max;
    float m_f_eval_exam_scores_max;
    float m_f_eval_part_scores_sub[EVAL_SUBPERIODS][EVAL_COUNT_MAX_SUBPERIOD];
    float m_f_eval_exam_scores_sub[EVAL_SUBPERIODS][EVAL_COUNT_MAX_SUBPERIOD];
    float m_f_eval_part_scores_overall_sub[EVAL_SUBPERIODS];
    float m_f_eval_exam_scores_overall_sub[EVAL_SUBPERIODS];
    float m_f_eval_part_fractions_overall_sub[EVAL_SUBPERIODS];
    float m_f_eval_exam_fractions_overall_sub[EVAL_SUBPERIODS];
    int m_i_eval_part_scores_count_sub[EVAL_SUBPERIODS];
    int m_i_eval_exam_scores_count_sub[EVAL_SUBPERIODS];
    float m_f_eval_part_stats_sub[EVAL_SUBPERIODS];
    float m_f_eval_exam_stats_sub[EVAL_SUBPERIODS];

    float *m_f_overall;
    float m_f_eval_scores_overall;
    float m_f_eval_fractions_overall;
    float m_f_eval_part_scores_overall;
    float m_f_eval_exam_scores_overall;
    float m_f_eval_part_fractions_overall;
    float m_f_eval_exam_fractions_overall;

    float m_f_eval_part_stats;
    float m_f_eval_exam_stats;
    float m_f_eval_overall_stats;
    int m_i_eval_overall_count;

public:

    void cb_sub_hea_ping_from_audience_id(const std_msgs::HeaderPtr&);

    WAIOAListener();
    ~WAIOAListener();
    void Initialize(ros::NodeHandle* hdl_node,int i_id);

    // New interface for evaluations
    std::string GetAlias();
    void SetAlias(std::string s_alias,std::string s_group);
    void SetEvalModel(int i_eval_model);
    int GetEvalModel();
    void AddEval(Eval evl_new);
    void ClearEvals();
    Eval GetEval(int i_eval_id);
    int GetEvalsCount();
    float GetEvalPartScoreMax();
    void SetEvalPartScoreMax(float f_eval_part_scores_max);
    float GetEvalExamScoreMax();
    void SetEvalExamScoreMax(float f_eval_exam_scores_max);
    std::string GetEvalsSubperiodSummary(int i_subperiod);
    float GetEvalPartExamScoresOverall();
    float GetEvalPartExamFractionsOverall();
    void SetEvalPartExamScoresOverall(float f_partexam);
    void SetEvalPartExamFractionsOverall(float f_partexam);

    void AddEvalPart(float f_eval_part_score,int i_sub_period);
    void AddEvalExam(float f_eval_exam_score,int i_sub_period);
    int GetEvalPartCountSubperiod(int i_sub_period);
    int GetEvalExamCountSubperiod(int i_sub_period);
    int GetEvalOverallCount();
    void ReInitEvalScoreSubperiods();
    float GetEvalPartScoreSubperiod(int i_sub_period,int i_which_part_score);
    float GetEvalExamScoreSubperiod(int i_sub_period,int i_which_exam_score);
    void UpdateEvalPartStatsSubperiod(int i_sub_period);
    void UpdateEvalExamStatsSubperiod(int i_sub_period);
    float GetEvalOverallStatsSubperiod(int i_sub);
    void UpdateEvalStatsOverall(float* f_eval_part_scores_overall,
                                float* f_eval_part_fractions_overall,
                                float* f_eval_exam_scores_overall,
                                float* f_eval_exam_fractions_overall,
                                float* f_eval_part_stats,
                                float* f_eval_exam_stats,
                                float* f_eval_overall_stats,
                                std::string* s_eval_overall_stats);

    // Other helper methods
    void SetPose(geometry_msgs::PoseStamped pst_pose);
    geometry_msgs::PoseStamped GetPose();
    void SendPing();
    void SendInfo(std::string s_info_to_audience_id);
    void SetStatsPing(float f_ping);
    float GetStatsPing();
};

#endif //WAI_OA_LISTENER_H
