#include<wai_oa_listener.h>



/////////////////////////////////////////////////
/// Implementation of WAIOAListener
/////////////////////////////////////////////////
WAIOAListener::WAIOAListener()
{
}
WAIOAListener::~WAIOAListener()
{
}

void WAIOAListener::Initialize(ros::NodeHandle* hdl_node,int i_id)
{
    m_hdl_node=hdl_node;
    m_i_audience_id=i_id;
    std::stringstream sst_s_name;
    sst_s_name << "ID" << i_id;
    m_s_alias_audience_id=sst_s_name.str();
    m_s_alias_group="default";

    m_f_stats_ping=0.0;

    m_pst_audience_pose.header.stamp=ros::Time::now();
    m_pst_audience_pose.header.frame_id="";
    m_pst_audience_pose.pose.position.x=0.0;
    m_pst_audience_pose.pose.position.y=0.0;
    m_pst_audience_pose.pose.position.z=0.0;
    m_pst_audience_pose.pose.orientation.w=1.0;
    m_pst_audience_pose.pose.orientation.x=0.0;
    m_pst_audience_pose.pose.orientation.y=0.0;
    m_pst_audience_pose.pose.orientation.z=0.0;

    m_i_eval_model=EVAL_MODEL_FRACTIONS_MOSAIC;
    m_f_eval_part_scores_max=1.0;
    m_f_eval_exam_scores_max=1.0;
    m_i_eval_overall_count=0;
    for(int i=0;i<EVAL_SUBPERIODS;i++)
    {
        m_f_eval_part_scores_overall_sub[i]=0.0;
        m_f_eval_exam_scores_overall_sub[i]=0.0;
        m_f_eval_part_fractions_overall_sub[i]=0.0;
        m_f_eval_exam_fractions_overall_sub[i]=0.0;
        m_i_eval_part_scores_count_sub[i]=0;
        m_i_eval_exam_scores_count_sub[i]=0;

        for(int j=0;j<EVAL_COUNT_MAX_SUBPERIOD;j++)
        {
            m_f_eval_part_scores_sub[i][j]=-1024.0;
            m_f_eval_exam_scores_sub[i][j]=-1024.0;
        }
    }

    m_f_overall=new float[2];
    m_f_eval_scores_overall=0.0;
    m_f_eval_fractions_overall=0.0;
    m_f_eval_part_scores_overall=0.0;
    m_f_eval_exam_scores_overall=0.0;
    m_f_eval_part_fractions_overall=0.0;
    m_f_eval_exam_fractions_overall=0.0;

    m_f_eval_overall_stats=0.0;

    // Init publishers for communication with audience
    pub_hea_info_to_audience_id=m_hdl_node->advertise<std_msgs::Header>("/wai_world/oa"+std::to_string(m_i_audience_id)+"/info_to_audience",10);
    sub_hea_ping_from_audience_id=m_hdl_node->subscribe("ping_from_audience_"+std::to_string(m_i_audience_id),1,&WAIOAListener::cb_sub_hea_ping_from_audience_id,this);
    pub_hea_ping_to_audience_id=m_hdl_node->advertise<std_msgs::Header>("/wai_world/oa"+std::to_string(m_i_audience_id)+"/ping_to_audience",1);
}

/////////////////////////////////////////////////
/// Callback to check pings to/from audience
/////////////////////////////////////////////////
void WAIOAListener::cb_sub_hea_ping_from_audience_id(const std_msgs::HeaderPtr& msg)
{
    std::string s_audience_id=msg->frame_id;
    size_t last_index = s_audience_id.find_last_not_of("0123456789");
    std::string s_cid = s_audience_id.substr(last_index + 1);
    int i=atoi(s_cid.c_str()); // Array and IDs start with index zero!
    m_f_stats_ping=(ros::Time::now()-msg->stamp).toSec();
    ROS_DEBUG_STREAM("Ping: Ping to ID" << s_cid  << " (Index " << i << ") is " << m_f_stats_ping << "s");
}

void WAIOAListener::SendPing()
{
    m_f_stats_ping=0.0;
    msg_hea_ping_to_audience_id.stamp=ros::Time::now();
    msg_hea_ping_to_audience_id.frame_id=("ping:to_audience_"+std::to_string(m_i_audience_id)).c_str();
    pub_hea_ping_to_audience_id.publish(msg_hea_ping_to_audience_id);
    ros::spinOnce();
}

void WAIOAListener::SendInfo(std::string s_info_to_audience_id)
{
    msg_hea_info_to_audience_id.stamp=ros::Time::now();
    msg_hea_info_to_audience_id.frame_id=s_info_to_audience_id.c_str();
    pub_hea_info_to_audience_id.publish(msg_hea_info_to_audience_id);
    ros::spinOnce();
}

std::string WAIOAListener::GetAlias()
{
    return m_s_alias_audience_id;
}

void WAIOAListener::SetAlias(std::string s_alias,std::string s_group)
{
    m_s_alias_audience_id=s_alias;
    m_s_alias_group=s_group;
}

void WAIOAListener::SetEvalModel(int i_eval_model)
{
    m_i_eval_model=i_eval_model;
}

int WAIOAListener::GetEvalModel()
{
    return m_i_eval_model;
}

void WAIOAListener::AddEval(Eval evl_new)
{
    m_vec_evals.push_back(evl_new);
}

void WAIOAListener::ClearEvals()
{
    m_vec_evals.clear();
}

Eval WAIOAListener::GetEval(int i_eval_id)
{
    return m_vec_evals[i_eval_id];
}

int WAIOAListener::GetEvalsCount()
{
    return m_vec_evals.size();
}

float WAIOAListener::GetEvalPartScoreMax()
{
    return m_f_eval_part_scores_max;
}

void WAIOAListener::SetEvalPartScoreMax(float f_eval_part_scores_max)
{
    m_f_eval_part_scores_max=f_eval_part_scores_max;
    if(m_i_eval_model==EVAL_MODEL_CALCULATOR)
    {
        m_f_eval_part_scores_max=1.0;
    }
}

float WAIOAListener::GetEvalExamScoreMax()
{
    return m_f_eval_exam_scores_max;
}

void WAIOAListener::SetEvalExamScoreMax(float f_eval_exam_scores_max)
{
    m_f_eval_exam_scores_max=f_eval_exam_scores_max;
    if(m_i_eval_model==EVAL_MODEL_CALCULATOR)
    {
        m_f_eval_exam_scores_max=1.0;
    }
}

std::string WAIOAListener::GetEvalsSubperiodSummary(int i_subperiod)
{
    std::stringstream sst_return;
    sst_return.str("");
    for(int i=0;i<m_vec_evals.size();i++)
    {
        if(m_vec_evals[i].i_month==i_subperiod) // Increment to get month 1-12
        {
            sst_return
                << m_vec_evals[i].s_date << "|"
                << m_vec_evals[i].s_audience_alias << "|"
                << m_vec_evals[i].s_type << "|"
                << m_vec_evals[i].s_label << "|"
                << m_vec_evals[i].s_score << "|"
                << m_vec_evals[i].s_comment << std::endl;
        }
    }
    return sst_return.str();
}

void WAIOAListener::AddEvalPart(float f_eval_part_score,int i_sub_period)
{
    m_i_eval_overall_count++;
    (m_i_eval_part_scores_count_sub[i_sub_period])++;
    m_f_eval_part_scores_sub[i_sub_period][m_i_eval_part_scores_count_sub[i_sub_period]]=f_eval_part_score;
}

void WAIOAListener::AddEvalExam(float f_eval_exam_score,int i_sub_period)
{
    m_i_eval_overall_count++;
    (m_i_eval_exam_scores_count_sub[i_sub_period])++;
    m_f_eval_exam_scores_sub[i_sub_period][m_i_eval_exam_scores_count_sub[i_sub_period]]=f_eval_exam_score;
}

int WAIOAListener::GetEvalPartCountSubperiod(int i_sub_period)
{
    return m_i_eval_part_scores_count_sub[i_sub_period];
}

int WAIOAListener::GetEvalExamCountSubperiod(int i_sub_period)
{
    return m_i_eval_exam_scores_count_sub[i_sub_period];
}

int WAIOAListener::GetEvalOverallCount()
{
    return m_i_eval_overall_count;
}

void WAIOAListener::ReInitEvalScoreSubperiods()
{
    for(int i=0;i<EVAL_SUBPERIODS;i++)
    {
        m_i_eval_part_scores_count_sub[i]=0;
        m_i_eval_exam_scores_count_sub[i]=0;

        for(int j=0;j<EVAL_COUNT_MAX_SUBPERIOD;j++)
        {
            m_f_eval_part_scores_sub[i][j]=-1024.0;
            m_f_eval_exam_scores_sub[i][j]=-1024.0;
        }
    }
}

float WAIOAListener::GetEvalPartScoreSubperiod(int i_sub_period,int i_which_part_score)
{
    //UpdateEvalPartStatsSubperiod(i_sub_period);
    return m_f_eval_part_scores_sub[i_sub_period][i_which_part_score];
}

float WAIOAListener::GetEvalExamScoreSubperiod(int i_sub_period,int i_which_exam_score)
{
    //UpdateEvalExamStatsSubperiod(i_sub_period);
    return m_f_eval_exam_scores_sub[i_sub_period][i_which_exam_score];
}

void WAIOAListener::UpdateEvalPartStatsSubperiod(int i_sub_period)
{
    if(m_i_eval_part_scores_count_sub[i_sub_period]>0)
    {
        if(m_i_eval_model==EVAL_MODEL_CALCULATOR)
        {
            // At this point the same as FM model
            m_f_eval_part_scores_overall_sub[i_sub_period]=0.0;
            m_f_eval_part_fractions_overall_sub[i_sub_period]=0.0;

            for(int i=1;i<=m_i_eval_part_scores_count_sub[i_sub_period];i++)
            {
                m_f_eval_part_scores_overall_sub[i_sub_period]=m_f_eval_part_scores_overall_sub[i_sub_period]+m_f_eval_part_scores_sub[i_sub_period][i];
                m_f_eval_part_fractions_overall_sub[i_sub_period]=m_f_eval_part_fractions_overall_sub[i_sub_period]+m_f_eval_part_scores_max;
            }
        }
        else if(m_i_eval_model==EVAL_MODEL_FRACTIONS_MOSAIC)
        {
            // Participation
            m_f_eval_part_scores_overall_sub[i_sub_period]=0.0;
            m_f_eval_part_fractions_overall_sub[i_sub_period]=0.0;

            for(int i=1;i<=m_i_eval_part_scores_count_sub[i_sub_period];i++)
            {
                m_f_eval_part_scores_overall_sub[i_sub_period]=m_f_eval_part_scores_overall_sub[i_sub_period]+m_f_eval_part_scores_sub[i_sub_period][i];
                m_f_eval_part_fractions_overall_sub[i_sub_period]=m_f_eval_part_fractions_overall_sub[i_sub_period]+m_f_eval_part_scores_max;
            }
            //ROS_ERROR("eval_part %3.3f eval_exam %3.3f, count %d",m_f_eval_part_scores_overall_sub[i_sub_period],m_f_eval_part_fractions_overall_sub[i_sub_period],m_i_eval_part_scores_count_sub[i_sub_period]);
            //m_f_eval_part_stats_sub[i_sub_period]=m_f_eval_part_scores_overall_sub[i_sub_period]/m_f_eval_part_fractions_overall_sub[i_sub_period];
        }
        else
        {
            ROS_DEBUG_STREAM("Eval: No proper model selected to update PART!");
        }
    }
    else
    {
        ROS_DEBUG_STREAM("Eval: Too less counts to update PART!");
    }
}

void WAIOAListener::UpdateEvalExamStatsSubperiod(int i_sub_period)
{
    if(m_i_eval_exam_scores_count_sub[i_sub_period]>0)
    {
        if(m_i_eval_model==EVAL_MODEL_CALCULATOR)
        {
            // At this point the same as FM model
            m_f_eval_exam_scores_overall_sub[i_sub_period]=0.0;
            m_f_eval_exam_fractions_overall_sub[i_sub_period]=0.0;

            for(int i=1;i<=m_i_eval_exam_scores_count_sub[i_sub_period];i++)
            {
                m_f_eval_exam_scores_overall_sub[i_sub_period]=m_f_eval_exam_scores_overall_sub[i_sub_period]+m_f_eval_exam_scores_sub[i_sub_period][i];
                m_f_eval_exam_fractions_overall_sub[i_sub_period]=m_f_eval_exam_fractions_overall_sub[i_sub_period]+m_f_eval_exam_scores_max;
            }
        }
        else if(m_i_eval_model==EVAL_MODEL_FRACTIONS_MOSAIC)
        {
            // Examination
            m_f_eval_exam_scores_overall_sub[i_sub_period]=0.0;
            m_f_eval_exam_fractions_overall_sub[i_sub_period]=0.0;

            for(int i=1;i<=m_i_eval_exam_scores_count_sub[i_sub_period];i++)
            {
                m_f_eval_exam_scores_overall_sub[i_sub_period]=m_f_eval_exam_scores_overall_sub[i_sub_period]+m_f_eval_exam_scores_sub[i_sub_period][i];
                m_f_eval_exam_fractions_overall_sub[i_sub_period]=m_f_eval_exam_fractions_overall_sub[i_sub_period]+m_f_eval_exam_scores_max;
            }
            //m_f_eval_exam_stats_sub[i_sub_period]=m_f_eval_exam_scores_overall_sub[i_sub_period]/m_f_eval_exam_fractions_overall_sub[i_sub_period];
        }
        else
        {
            ROS_DEBUG_STREAM("Eval: No proper model selected to update EXAM!");
        }
    }
    else
    {
        ROS_DEBUG_STREAM("Eval: Too less counts to update EXAM!");
    }
}

float WAIOAListener::GetEvalOverallStatsSubperiod(int i_sub)
{
    float f_overall_subper=0.0;
    UpdateEvalPartStatsSubperiod(i_sub);
    UpdateEvalExamStatsSubperiod(i_sub);

    if(m_i_eval_part_scores_count_sub[i_sub]>0 &&
            m_i_eval_exam_scores_count_sub[i_sub]>0)
    {
        // Be carefule with mixing per subperiod stats and overall stats with fractions!
        // With FRACTIONS_MOSAIC:
        // Mean of individual subperiods to overall mean gets wronged (of course!)
        // E.g. in one subperiod PART 4/8=0.5 and 16/24=0.666 mean is 0.58333 ...
        // ... unequals (4+16)/(8+24)=0.625 (remember that throughout fraction size we achieve weighting here!)
        // To switch to simple mean calculation, disable b_use_eval_status_overall parameter!
        f_overall_subper=
                (m_f_eval_part_scores_overall_sub[i_sub]+m_f_eval_exam_scores_overall_sub[i_sub])/
                (m_f_eval_part_fractions_overall_sub[i_sub]+m_f_eval_exam_fractions_overall_sub[i_sub]);

        /* f_overall_subper=EVAL_PART_WEIGHT*
                m_f_eval_part_scores_overall_sub[i_sub]/
                m_f_eval_part_fractions_overall_sub[i_sub]
                +
                EVAL_EXAM_WEIGHT*
                m_f_eval_exam_scores_overall_sub[i_sub]/
                m_f_eval_exam_fractions_overall_sub[i_sub];*/
    }
    else if( (m_i_eval_part_scores_count_sub[i_sub]>0) &&
            !(m_i_eval_exam_scores_count_sub[i_sub]>0) )
    {
        f_overall_subper=//EVAL_PART_WEIGHT*
                m_f_eval_part_scores_overall_sub[i_sub]/
                m_f_eval_part_fractions_overall_sub[i_sub];
    }
    else if( !(m_i_eval_part_scores_count_sub[i_sub]>0) &&
            (m_i_eval_exam_scores_count_sub[i_sub]>0) )
    {
        f_overall_subper=//EVAL_EXAM_WEIGHT*
                m_f_eval_exam_scores_overall_sub[i_sub]/
                m_f_eval_exam_fractions_overall_sub[i_sub];
    }
    else
    {
        f_overall_subper=-1024.0;
    }

    return f_overall_subper;
}

float WAIOAListener::GetEvalPartExamScoresOverall()
{
    return m_f_overall[0];
}
float WAIOAListener::GetEvalPartExamFractionsOverall()
{
    return m_f_overall[1];
}
void WAIOAListener::SetEvalPartExamScoresOverall(float f_partexam)
{
    m_f_overall[0]=f_partexam;
}
void WAIOAListener::SetEvalPartExamFractionsOverall(float f_partexam)
{
    m_f_overall[1]=f_partexam;
}


void WAIOAListener::UpdateEvalStatsOverall(
                            float* f_eval_part_scores_overall,
                            float* f_eval_part_fractions_overall,
                            float* f_eval_exam_scores_overall,
                            float* f_eval_exam_fractions_overall,
                            float* f_eval_part_stats,
                            float* f_eval_exam_stats,
                            float* f_eval_overall_stats,
                            std::string* s_eval_overall_stats)
{
    m_f_eval_part_scores_overall=0.0;
    m_f_eval_exam_scores_overall=0.0;
    m_f_eval_part_fractions_overall=0.0;
    m_f_eval_exam_fractions_overall=0.0;
    m_f_eval_overall_stats=0.0;

    if(m_i_eval_model==EVAL_MODEL_CALCULATOR)
    {
        for(int i=0;i<EVAL_SUBPERIODS;i++)
        {
            UpdateEvalPartStatsSubperiod(i);
            UpdateEvalExamStatsSubperiod(i);

            if(m_i_eval_part_scores_count_sub[i]>0)
            {
                m_f_eval_part_scores_overall=m_f_eval_part_scores_overall+m_f_eval_part_scores_overall_sub[i];
                m_f_eval_part_fractions_overall=m_f_eval_part_fractions_overall+m_f_eval_part_fractions_overall_sub[i];
                // Upate every period here
                m_f_eval_part_stats=m_f_eval_part_scores_overall/m_f_eval_part_fractions_overall;
                //m_f_eval_overall_stats=EVAL_PART_WEIGHT*m_f_eval_part_stats+EVAL_EXAM_WEIGHT*m_f_eval_exam_stats;
            }
            if(m_i_eval_exam_scores_count_sub[i]>0)
            {
                m_f_eval_exam_scores_overall=m_f_eval_exam_scores_overall+m_f_eval_exam_scores_overall_sub[i];
                m_f_eval_exam_fractions_overall=m_f_eval_exam_fractions_overall+m_f_eval_exam_fractions_overall_sub[i];
                // Upate every period here
                m_f_eval_exam_stats=m_f_eval_exam_scores_overall/m_f_eval_exam_fractions_overall;
                //m_f_eval_overall_stats=EVAL_PART_WEIGHT*m_f_eval_part_stats+EVAL_EXAM_WEIGHT*m_f_eval_exam_stats;
            }
        }

        *f_eval_part_scores_overall=m_f_eval_part_scores_overall;// Scores (e.g. multiple 16.5 scores=tilde) over all subperiods per ID
        *f_eval_part_fractions_overall=m_f_eval_part_fractions_overall;// E.g. (e.g. multiple 24-fractions) over all subperiods per ID
        *f_eval_exam_scores_overall=m_f_eval_exam_scores_overall;
        *f_eval_exam_fractions_overall=m_f_eval_exam_fractions_overall;
        *f_eval_part_stats=m_f_eval_part_stats;
        *f_eval_exam_stats=m_f_eval_exam_stats;
        //*f_eval_overall_stats=m_f_eval_overall_stats; Unused for CALC model!

        // With calculator model, simply calc mean of exam scores/fractions and let
        // part scores "decide between half grades"
        // (Bäh..., please find a novel proper way to well define thresholds here...)
        int i_part_decisive=0;
        float f_part_decisive_threshold=0.125/20.0;
        if(m_f_eval_part_stats==0.5) i_part_decisive=0;
        if(m_f_eval_part_stats<0.5) i_part_decisive=-1;
        if(m_f_eval_part_stats>0.5) i_part_decisive=1;

        // CALC model - part is supporting "half-grades", use fractions as simple eval counter here
        if(m_f_eval_part_fractions_overall>0 && m_f_eval_exam_fractions_overall>0)
        {
            if(fabs(0.5-m_f_eval_exam_stats)<f_part_decisive_threshold)
            {
                if(i_part_decisive==-1) *s_eval_overall_stats="5";
                if(i_part_decisive==0) *s_eval_overall_stats="4-5";
                if(i_part_decisive==1) *s_eval_overall_stats="4";
            }
            else if(fabs(0.625-m_f_eval_exam_stats)<f_part_decisive_threshold)
            {
                if(i_part_decisive==-1) *s_eval_overall_stats="4";
                if(i_part_decisive==0) *s_eval_overall_stats="3-4";
                if(i_part_decisive==1) *s_eval_overall_stats="3";
            }
            else if(fabs(0.75-m_f_eval_exam_stats)<f_part_decisive_threshold)
            {
                if(i_part_decisive==-1) *s_eval_overall_stats="3";
                if(i_part_decisive==0) *s_eval_overall_stats="2-3";
                if(i_part_decisive==1) *s_eval_overall_stats="2";
            }
            else if(fabs(0.875-m_f_eval_exam_stats)<f_part_decisive_threshold)
            {
                if(i_part_decisive==-1) *s_eval_overall_stats="2";
                if(i_part_decisive==0) *s_eval_overall_stats="1-2";
                if(i_part_decisive==1) *s_eval_overall_stats="1";
            }
            else if(m_f_eval_overall_stats>=0.5 && m_f_eval_overall_stats<0.625)
            {
                *s_eval_overall_stats="4";
            }
            else if(m_f_eval_overall_stats>=0.625 && m_f_eval_overall_stats<0.75)
            {
                *s_eval_overall_stats="3";
            }
            else if(m_f_eval_overall_stats>=0.75 && m_f_eval_overall_stats<0.875)
            {
                *s_eval_overall_stats="2";
            }
            else if(m_f_eval_overall_stats>=0.875)
            {
                *s_eval_overall_stats="1";
            }
            else
            {
                *s_eval_overall_stats="!";
            }
        }
        else
        {
            *s_eval_overall_stats="N/A";
        }

    }
    else if(m_i_eval_model==EVAL_MODEL_FRACTIONS_MOSAIC)
    {
        for(int i=0;i<EVAL_SUBPERIODS;i++)
        {
            UpdateEvalPartStatsSubperiod(i);
            UpdateEvalExamStatsSubperiod(i);

            m_f_eval_part_scores_overall=m_f_eval_part_scores_overall+m_f_eval_part_scores_overall_sub[i];
            m_f_eval_part_fractions_overall=m_f_eval_part_fractions_overall+m_f_eval_part_fractions_overall_sub[i];
            m_f_eval_exam_scores_overall=m_f_eval_exam_scores_overall+m_f_eval_exam_scores_overall_sub[i];
            m_f_eval_exam_fractions_overall=m_f_eval_exam_fractions_overall+m_f_eval_exam_fractions_overall_sub[i];

            /*
            if(m_i_eval_part_scores_count_sub[i]>0)
            {
                m_f_eval_part_scores_overall=m_f_eval_part_scores_overall+m_f_eval_part_scores_overall_sub[i];
                m_f_eval_part_fractions_overall=m_f_eval_part_fractions_overall+m_f_eval_part_fractions_overall_sub[i];
                // Upate every period here
                // Weighting is achieved throughout FRACTIONS SIZE!
                m_f_eval_part_stats=m_f_eval_part_scores_overall/m_f_eval_part_fractions_overall;
                m_f_eval_overall_stats=(m_f_eval_part_scores_overall+m_f_eval_exam_scores_overall)/(m_f_eval_part_fractions_overall+m_f_eval_exam_fractions_overall);
            }
            if(m_i_eval_exam_scores_count_sub[i]>0)
            {
                m_f_eval_exam_scores_overall=m_f_eval_exam_scores_overall+m_f_eval_exam_scores_overall_sub[i];
                m_f_eval_exam_fractions_overall=m_f_eval_exam_fractions_overall+m_f_eval_exam_fractions_overall_sub[i];
                // Upate every period here
                // Weighting is achieved throughout FRACTIONS SIZE!
                m_f_eval_exam_stats=m_f_eval_exam_scores_overall/m_f_eval_exam_fractions_overall;
                m_f_eval_overall_stats=(m_f_eval_part_scores_overall+m_f_eval_exam_scores_overall)/(m_f_eval_part_fractions_overall+m_f_eval_exam_fractions_overall);
            }*/
        }

        // Todo: Check for correct instantiation of objects in vector!
        m_f_overall[0]=m_f_eval_part_scores_overall+m_f_eval_exam_scores_overall;
        m_f_overall[1]=m_f_eval_part_fractions_overall+m_f_eval_exam_fractions_overall;
        m_f_eval_scores_overall=m_f_eval_part_scores_overall+m_f_eval_exam_scores_overall;
        m_f_eval_fractions_overall=m_f_eval_part_fractions_overall+m_f_eval_exam_fractions_overall;
        *f_eval_part_scores_overall=m_f_eval_part_scores_overall;// Scores (e.g. multiple 16.5 scores=tilde) over all subperiods per ID
        *f_eval_part_fractions_overall=m_f_eval_part_fractions_overall;// Fractions (e.g. multiple 24-fractions) over all subperiods per ID
        *f_eval_exam_scores_overall=m_f_eval_exam_scores_overall;
        *f_eval_exam_fractions_overall=m_f_eval_exam_fractions_overall;
        *f_eval_part_stats=m_f_eval_part_stats;
        *f_eval_exam_stats=m_f_eval_exam_stats;
        *f_eval_overall_stats=m_f_overall[0]/m_f_overall[1];
        m_f_eval_overall_stats=m_f_overall[0]/m_f_overall[1];
        //ROS_WARN("Overall: %3.3f ; %3.3f",*f_eval_overall_stats,m_f_overall[0]/m_f_overall[1]);

        // Assignment to grade - equal interval widths
        if(m_f_eval_part_fractions_overall>0 && m_f_eval_exam_fractions_overall>0)
        {
            if(m_f_eval_overall_stats<0.5 && m_f_eval_overall_stats>0.0)
            {
                *s_eval_overall_stats="5";
            }
            else if(m_f_eval_overall_stats>=0.5 && m_f_eval_overall_stats<0.625)
            {
                *s_eval_overall_stats="4";
            }
            else if(m_f_eval_overall_stats>=0.625 && m_f_eval_overall_stats<0.75)
            {
                *s_eval_overall_stats="3";
            }
            else if(m_f_eval_overall_stats>=0.75 && m_f_eval_overall_stats<0.875)
            {
                *s_eval_overall_stats="2";
            }
            else if(m_f_eval_overall_stats>=0.875)
            {
                *s_eval_overall_stats="1";
            }
            else
            {
                *s_eval_overall_stats="!";
            }
        }
        else
        {
            *s_eval_overall_stats="N/A";
        }
    }
}

// Methods not related to eval
void WAIOAListener::SetPose(geometry_msgs::PoseStamped pst_pose)
{
    m_pst_audience_pose=pst_pose;
}

geometry_msgs::PoseStamped WAIOAListener::GetPose()
{
    return m_pst_audience_pose;
}

float WAIOAListener::GetStatsPing()
{
    //double d_stats_ping=;
    return m_f_stats_ping;//d_stats_ping;
}

void WAIOAListener::SetStatsPing(float f_ping)
{
    m_f_stats_ping=f_ping;
}
