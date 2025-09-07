#include<wai_oa_audience.h>



/////////////////////////////////////////////////
/// Implementation of WAIOAAudience
/////////////////////////////////////////////////

WAIOAAudience::WAIOAAudience()
{
}

WAIOAAudience::~WAIOAAudience()
{
}

WAIOAListener WAIOAAudience::GetAudienceID(int i_id)
{
    return *(vec_audience[i_id]);
}

std::string WAIOAAudience::GetAliasID(int i_id)
{
    return vec_audience[i_id]->GetAlias();
}
void WAIOAAudience::SetAliasID(int i_id,std::string s_alias,std::string s_alias_group)
{
    vec_audience[i_id]->SetAlias(s_alias,s_alias_group);
}

float WAIOAAudience::GetScoresOverallFromAudienceID(int i_id)
{
    return vec_audience[i_id]->GetEvalPartExamScoresOverall();
}
float WAIOAAudience::GetFractionsOverallFromAudienceID(int i_id)
{
    return vec_audience[i_id]->GetEvalPartExamFractionsOverall();
}

void WAIOAAudience::Initialize(ros::NodeHandle* hdl_node,
                                       int i_audience_count_max,
                                       int i_audience_listeners_per_row,
                                       tf::Vector3 vc3_audience_offset,
                                       tf::Vector3 vc3_audience_spacing,
                                       std::string s_audience_alias,
                                       int i_audience_eval_model,
                                       float f_eval_part_score_max,
                                       float f_eval_exam_score_max)
{
    m_hdl_node=hdl_node;
    m_i_audience_count_max=i_audience_count_max;
    m_i_audience_listeners_per_row=i_audience_listeners_per_row;
    m_vc3_audience_offset=vc3_audience_offset;
    m_vc3_audience_spacing=vc3_audience_spacing;
    m_s_audience_alias=s_audience_alias;
    m_i_audience_eval_model=i_audience_eval_model;

    for(int i=0;i<m_i_audience_count_max;i++)
    {
        WAIOAListener* oa_listener=new WAIOAListener();
        oa_listener->Initialize(hdl_node,i);
        std::stringstream sst_audience_alias;
        sst_audience_alias << "ID" << i;
        oa_listener->SetAlias(sst_audience_alias.str(),m_s_audience_alias);
        oa_listener->SetEvalModel(m_i_audience_eval_model);
        oa_listener->SetEvalPartScoreMax(f_eval_part_score_max);
        oa_listener->SetEvalExamScoreMax(f_eval_exam_score_max);
        oa_listener->SetEvalPartExamScoresOverall(0.0);
        oa_listener->SetEvalPartExamFractionsOverall(0.0);
        vec_audience.push_back(oa_listener);
    }
}

/* Old interface for eval:
std::string WAIOAAudience::ImportEvalsIDSubperiodFromTextfile(std::string s_path_group,int i_eval_id,int i_eval_sub)
{
    std::stringstream sst_return;
    sst_return.str("");

    for(int i=0;i<m_i_audience_count_max;i++)
    {
        vec_audience[i].ReInitEvalScoreSubperiods();
    }
    std::ifstream ifs_file(s_path_group.c_str());
    std::string line;

    while(std::getline(ifs_file, line))
    {
        std::string s_date;
        std::string s_weekday;
        std::string s_time;
        std::string s_id; int i_id;
        std::string s_group;
        std::string s_topic;
        std::string s_expertise;
        std::string s_expertise_level;
        std::string s_session;
        std::string s_eval_type;
        std::string s_eval_score;
        std::string s_eval_comment;
        std::replace(line.begin(), line.end(), ',', ' ');

        std::stringstream sst_line(line);
        sst_line >> s_date;
            std::stringstream sst_data(s_date);
            std::string s_year,s_month,s_day;
            std::getline(sst_data,s_year,'.');
            std::getline(sst_data,s_month,'.');
            std::getline(sst_data,s_day,'.');
            int i_year,i_month,i_day;
            i_year=atoi(s_year.c_str());
            i_month=atoi(s_month.c_str())-1; // Starts at IDX 0
            i_day=atoi(s_day.c_str());
        sst_line >> s_weekday;
        sst_line >> s_time;
        sst_line >> s_id;
            size_t last_index = s_id.find_last_not_of("0123456789");
            std::string s_cid = s_id.substr(last_index + 1);
            i_id=atoi(s_cid.c_str())-1; // watch array bounds! // Starts at IDX 0
        sst_line >> s_group;
        sst_line >> s_topic;
        sst_line >> s_expertise;
        sst_line >> s_expertise_level;
        sst_line >> s_session;
        sst_line >> s_eval_type;
        sst_line >> s_eval_score;
            float f_eval_score=std::atof(s_eval_score.c_str());
        sst_line >> s_eval_comment;

        if(i_id==i_eval_id && i_month==i_eval_sub)
        {
            sst_return << s_date << " " << "ID" << i_id+1 << " " << s_eval_type << ":" << s_eval_score << "-" << s_eval_comment << std::endl;
        }
    }
    ifs_file.close();

    return sst_return.str();
}*/

// With new interface for eval:
void WAIOAAudience::ImportEvalsGroupFromFileText(std::string s_path_group)
{
    for(int i=0;i<m_i_audience_count_max;i++)
    {
        vec_audience[i]->ClearEvals();
        vec_audience[i]->ReInitEvalScoreSubperiods();
    }
    std::ifstream ifs_file(s_path_group.c_str());
    std::string line;

    while(std::getline(ifs_file, line))
    {
        std::string s_date; int i_year,i_month,i_day;
        std::string s_calendar_week; int i_calendar_week;
        std::string s_weekday;
        std::string s_time; int i_hour,i_min,i_sec;
        std::string s_id; int i_id;
        std::string s_alias;
        std::string s_group;
        std::string s_expertise;
        std::string s_expertise_level;
        std::string s_topic;
        std::string s_session;
        std::string s_label;
        std::string s_eval_type;
        std::string s_eval_score; float f_eval_score;
        std::string s_eval_score_max; float f_eval_score_max;
        std::string s_eval_relation; float f_eval_relation;
        std::string s_eval_comment;

        std::replace(line.begin(), line.end(), ' ', '_'); // Make sure that there are no whitespaces.
        //std::replace(line.begin(), line.end(), ';', ' '); // Substitue separator , with whitespace for parsing.

        std::istringstream sst_oneline(line);
        std::string s_line_fragment="";

        // DATE
        s_date="";
        std::getline(sst_oneline,s_line_fragment,';'); // YEAR
            s_date=s_date+s_line_fragment+"-";
            i_year=atoi(s_line_fragment.c_str());
        std::getline(sst_oneline,s_line_fragment,';'); // MONTH
            s_date=s_date+s_line_fragment+"-";
            i_month=atoi(s_line_fragment.c_str())-1; // Month is STORED in file readable M1-M12, PROCESSED with M0-M11!
        std::getline(sst_oneline,s_line_fragment,';'); // DAY
            s_date=s_date+s_line_fragment;
            i_day=atoi(s_line_fragment.c_str());
        std::getline(sst_oneline,s_line_fragment,';'); // CALENDAR WEEK
            s_calendar_week=s_line_fragment;
            i_calendar_week=atoi(s_line_fragment.c_str());
        std::getline(sst_oneline,s_line_fragment,';'); // WEEK DAY
            s_weekday=s_line_fragment;
        // TIME
        s_time="";
        std::getline(sst_oneline,s_line_fragment,';'); // HOUR
            s_time=s_time+s_line_fragment+":";
            i_hour=atoi(s_line_fragment.c_str());
        std::getline(sst_oneline,s_line_fragment,';'); // MINUTE
            s_time=s_time+s_line_fragment+":";
            i_min=atoi(s_line_fragment.c_str());
        std::getline(sst_oneline,s_line_fragment,';'); // SECOND
            s_time=s_time+s_line_fragment;
            i_sec=atoi(s_line_fragment.c_str());
        std::getline(sst_oneline,s_line_fragment,';'); // ID
            s_id=s_line_fragment;
            i_id=atoi(s_line_fragment.c_str());
        std::getline(sst_oneline,s_line_fragment,';'); s_alias=s_line_fragment; // ALIAS
        std::getline(sst_oneline,s_line_fragment,';'); s_group=s_line_fragment; // GROUP
        std::getline(sst_oneline,s_line_fragment,';'); s_topic=s_line_fragment; // TOPIC
        std::getline(sst_oneline,s_line_fragment,';'); s_expertise=s_line_fragment; // EXPERTISE
        std::getline(sst_oneline,s_line_fragment,';'); s_expertise_level=s_line_fragment; // EXPERTISE LEVEL
        std::getline(sst_oneline,s_line_fragment,';'); s_session=s_line_fragment; // SESSION NAME
        std::getline(sst_oneline,s_line_fragment,';'); s_eval_type=s_line_fragment; // EVAL TYPE
        std::getline(sst_oneline,s_line_fragment,';'); s_label=s_line_fragment; // EVAL LABEL

        std::getline(sst_oneline,s_line_fragment,';'); s_eval_score=s_line_fragment; // EVAL SCORE
        std::replace(s_eval_score.begin(),s_eval_score.end(),',','.');
        std::istringstream istr_eval_score(s_eval_score); istr_eval_score >> f_eval_score; // Make sure to use one language format (US)

        std::getline(sst_oneline,s_line_fragment,';'); s_eval_score_max=s_line_fragment; // EVAL SCORE MAX
        std::replace(s_eval_score_max.begin(),s_eval_score_max.end(),',','.');
        std::istringstream istr_eval_score_max(s_eval_score_max); istr_eval_score_max >> f_eval_score_max; // Make sure to use one language format (US)

        std::getline(sst_oneline,s_line_fragment,';'); s_eval_relation=s_line_fragment; // EVAL RELATION
        std::replace(s_eval_relation.begin(),s_eval_relation.end(),',','.');
        std::istringstream istr_eval_relation(s_eval_relation); istr_eval_relation >> f_eval_relation; // Make sure to use one language format (US)

        std::getline(sst_oneline,s_line_fragment,';'); s_eval_comment=s_line_fragment; // COMMENT

        // Debug output
        ROS_DEBUG_STREAM("--- PARSING EVAL --- " << std::endl
            << "DAT:" << s_date << " ; "
            << "Y:" << i_year << " ; "
            << "M:" << i_month+1 << " ; " // Month is STORED in file readable M1-M12, PROCESSED with M0-M11!
            << "D:" << i_day << " ; "
            << "CAL.WEEK:" << i_calendar_week << " ; "
            << "W:" << s_weekday << " ; "
            << "TIM:" << s_time << " ; "
            << "HOU:" << i_hour << " ; "
            << "MIN:" << i_min << " ; "
            << "SEC:" << i_sec << " ; "
            << "ID:" << i_id << " ; "
            << "ALIAS:" << s_alias << " ; "
            << "GRP:" << s_group << " ; "
            << "TOP:" << s_topic << " ; "
            << "EXP:" << s_expertise << " ; "
            << "EXP-LVL:" << s_expertise_level << " ; "
            << "SES:" << s_session << " ; "
            << "EVAL-TYP:" << s_eval_type << " ; "
            << "LBL:" << s_label << " ; "
            << "EVAL-SCR:" << f_eval_score << " ; "
            << "EVAL-SCRMAX:" << f_eval_score_max << " ; "
            << "EVAL-REL:" << f_eval_relation << " ; "
            << "EVAL-COMM:" << s_eval_comment);

        // NEW interface
        Eval evl_eval;
        evl_eval.i_eval_id=vec_audience[i_id]->GetEvalsCount();
        evl_eval.s_date=s_date;
            evl_eval.i_year=i_year;
            evl_eval.i_month=i_month;
            evl_eval.i_day=i_day;
        evl_eval.i_calendar_week=i_calendar_week;
        evl_eval.s_weekday=s_weekday;
        evl_eval.s_time=s_time;
        evl_eval.i_hour=i_hour;
        evl_eval.i_min=i_min;
        evl_eval.i_sec=i_sec;
        evl_eval.i_audience_id=i_id; // e.g., 1
        evl_eval.s_audience_alias=s_alias; // e.g. "ID1"
        evl_eval.s_group=s_group; // e.g., "5bhel"
        evl_eval.s_topic=s_topic; // e.g., "FSST"
        evl_eval.s_expertise=s_expertise; // e.g., "Loops"
        evl_eval.s_expertise_level=s_expertise_level; // e.g., 3 = "intermediate"
        evl_eval.s_session=s_session;
        evl_eval.s_type=s_eval_type; // e.g., "exam","part"
        evl_eval.s_label=s_label;// e.g., "part check for-loop"
        evl_eval.s_score=s_eval_score;
        evl_eval.f_score=f_eval_score;
        evl_eval.s_score_max=s_eval_score_max;
        evl_eval.f_score_max=f_eval_score_max;
        evl_eval.s_relation=s_eval_relation;
        evl_eval.f_relation=f_eval_relation;
        evl_eval.s_comment=s_eval_comment;
        vec_audience[i_id]->AddEval(evl_eval);


        // OLD Interface - Fill audience/listener
        //vec_audience[i_id]->SetAlias(s_id,s_group);  // Done
        if(s_eval_type.compare("part")==0)
        {
            vec_audience[i_id]->AddEvalPart(f_eval_score,i_month);
        }
        if(s_eval_type.compare("exam")==0)
        {
            vec_audience[i_id]->AddEvalExam(f_eval_score,i_month);
        }
    }

    ifs_file.close();
}

std::string WAIOAAudience::GetEvalsAudienceIDSubperiod(int i_eval_audience_id,int i_eval_audience_sub)
{
    return vec_audience[i_eval_audience_id]->GetEvalsSubperiodSummary(i_eval_audience_sub);
}

float WAIOAAudience::GetStatsAudienceID(int i_stats_audience_id)
{
    return vec_audience[i_stats_audience_id]->GetStatsPing();
}

void WAIOAAudience::ResetStatsAudienceID(int i_stats_audience_id)
{
    vec_audience[i_stats_audience_id]->SetStatsPing(0.0);
}

int WAIOAAudience::GetAudienceCountMax()
{
    return m_i_audience_count_max;
}

int WAIOAAudience::GetAudienceListenersPerRow()
{
    return m_i_audience_listeners_per_row;
}

tf::Vector3 WAIOAAudience::GetAudiencePosFromID(int i_audience_id)
{
    tf::Vector3 vc3_pos=tf::Vector3(0.0,0.0,0.0);
    vc3_pos.setX(m_vc3_audience_offset.getX()+m_vc3_audience_spacing.getX()*int(i_audience_id/m_i_audience_listeners_per_row));
    vc3_pos.setY(m_vc3_audience_offset.getY()+m_vc3_audience_spacing.getY()*int(i_audience_id%m_i_audience_listeners_per_row));
    vc3_pos.setZ(m_vc3_audience_offset.getZ()); // Alle audience workspaces are set at constant height for now!
    return vc3_pos;
}
