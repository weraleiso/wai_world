#ifndef WAI_ETHICAL_MODEL_H
#define WAI_ETHICAL_MODEL_H

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
#include<cstdlib>
#include<ctime>

#include<ros/ros.h>
#include<ros/package.h>
#include<std_msgs/Empty.h>
#include<std_msgs/Bool.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include<gazebo_msgs/LinkStates.h>
#include<gazebo_msgs/SetModelState.h>
#include<std_srvs/Empty.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<view_controller_msgs/CameraPlacement.h>



// -=[INTEL]=-
// Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
// preserve natural human presence in the use of AIS in education"
// In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
// Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
// --> Define Ethical Reference Model (WORK IN PROGRESS!):
#define ETHICAL_DECISION_ACCEPT "ACCEPT"
#define ETHICAL_DECISION_ADAPT "ADAPT"
#define ETHICAL_DECISION_TOLERATE "TOLERATE"
#define ETHICAL_DECISION_REJECT "REJECT"
#define ETHICAL_DECISION_OVERRIDE "OVERRIDE"

#define RISK_LEVEL_HIGH 3
#define RISK_LEVEL_MODERATE 2
#define RISK_LEVEL_LOW 1

struct ActorEthicalProperties
{
    float f_position_x;
    float f_position_y;
    float f_position_z;
    float f_visual_representation;
    float f_mood;
    float f_time_coherence;
};

class WAIEthicalUseCase
{
    // II) Well-define USE-CASE
    std::string m_s_principle;
    std::string m_s_method;
    std::string m_s_activity;
    std::string m_s_sub_goal;
    std::string m_s_use_case;

    // III) Well-define ETHICAL LOW-LEVEL PROPERTIES
    int m_i_risk_level; // E.g., 1=LOW, 2=MODERATE, 3=HIGH
    double m_d_iol_reference; // Maximum recommended IOL (WORK IN PROGRESS!)
    double m_d_iol_actual; // Current IOL for a specific use case
    int m_i_iol_occurence_prompts;
    //int m_i_actor_types_roles; // Actor Types/Roles: Natural Human (Leading/Participating), Artificial (Supporting)
    // E.g., Maximum of 25% of presentation time (one session is 100min) is allowed
    //m_d_iol_actual=0.0; // E.g., At the moment, AIS interactions demanded 22% of presentation time
    //m_d_iol_delta=0.0; // E.g., The newly requested AIS interaction would demand additional 5% of presentation time

public:
    WAIEthicalUseCase()
    {
    }
    ~WAIEthicalUseCase()
    {
    }
    void Initialize()
    {
        m_s_principle="";
        m_s_method="";
        m_s_activity="";
        m_s_sub_goal="";
        m_s_use_case="";

        m_i_risk_level=0;
        m_d_iol_reference=0.0;
        m_d_iol_actual=0.0;
        m_i_iol_occurence_prompts=0;
    }
    std::string GetUseCase()
    {
        return m_s_use_case;
    }
    int GetRiskLevel()
    {
        return m_i_risk_level;
    }
    double GetIOLReference()
    {
        return m_d_iol_reference;
    }
    double GetIOLActual()
    {
        return m_d_iol_actual;
    }
    void SetEthicalProperties(  std::string s_principle,
                                std::string s_method,
                                std::string s_activity,
                                std::string s_sub_goal,
                                std::string s_use_case,
                                int i_risk_level,
                                double d_iol_reference)
    {
        m_s_principle=s_principle;
        m_s_method=s_method;
        m_s_activity=s_activity;
        m_s_sub_goal=s_sub_goal;
        m_s_use_case=s_use_case;

        m_i_risk_level=i_risk_level;
        m_d_iol_reference=d_iol_reference;
    }
    void UpdateIOLActual(double d_delta)
    {
        m_d_iol_actual=m_d_iol_actual+d_delta;
    }
};

class WAIEthicalModel
{
    // -=[INTEL]=-
    // Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
    // preserve natural human presence in the use of AIS in education"
    // In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
    // Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
    // --> Decision-Making Algorithm - I) IDENTIFY CONTEXT
    // The context is well-defined by the overarching workspace, implemented in the base class "WAIWorkspace"
    std::string m_s_field_discipline;
    std::string m_s_specialization;
    std::string m_s_process;
    std::string m_s_system;
    std::string m_s_overall_goal;
    std::string m_s_delimitation;

public:
    WAIEthicalModel()
    {
    }
    ~WAIEthicalModel()
    {
    }

    std::vector<WAIEthicalUseCase> vec_wai_ethical_use_cases;
    std::vector<ActorEthicalProperties> m_vec_ethi_prop_pre;
    std::vector<ActorEthicalProperties> m_vec_ethi_prop_aud;
    std::vector<ActorEthicalProperties> m_vec_ethi_prop_ais;

    // Accords to N educators/presenters, n learners/audience, and m AIS
    void Initialize(int i_pre_cnt_max,int i_aud_cnt_max,int i_ais_cnt_max)
    {
        m_s_field_discipline="";
        m_s_specialization="";
        m_s_process="";
        m_s_system="";
        m_s_overall_goal="";
        m_s_delimitation="";

        vec_wai_ethical_use_cases.clear();

        // Init 3 vectors accordingly
        ActorEthicalProperties aep_temp;
        aep_temp.f_position_x=0.0;
        aep_temp.f_position_y=0.0;
        aep_temp.f_position_z=0.0;
        aep_temp.f_visual_representation=0.0;
        aep_temp.f_mood=0.0;
        aep_temp.f_time_coherence=0.0;
        m_vec_ethi_prop_pre.clear();
        m_vec_ethi_prop_aud.clear();
        m_vec_ethi_prop_ais.clear();
        for(int i=0;i<i_pre_cnt_max;i++) m_vec_ethi_prop_pre.push_back(aep_temp); // N=1 presenters
        for(int i=0;i<i_aud_cnt_max;i++) m_vec_ethi_prop_aud.push_back(aep_temp); // n=cnt_max audience
        for(int i=0;i<i_ais_cnt_max;i++) m_vec_ethi_prop_ais.push_back(aep_temp); // m=1 AIS
    }

    void SetContext(std::string s_field_discipline,
                    std::string s_specialization,
                    std::string s_process,
                    std::string s_system,
                    std::string s_overall_goal,
                    std::string s_delimitation)
    {
        m_s_field_discipline=s_field_discipline;
        m_s_specialization=s_specialization;
        m_s_process=s_process;
        m_s_system=s_system;
        m_s_overall_goal=s_overall_goal;
        m_s_delimitation=s_delimitation;
    }

    std::string GetContextFieldDiscipline()
    {
        return m_s_field_discipline;
    }
    std::string GetContextSpecialization()
    {
        return m_s_specialization;
    }
    std::string GetContextProcess()
    {
        return m_s_process;
    }
    std::string GetContextSystem()
    {
        return m_s_system;
    }
    std::string GetContextOverallGoal()
    {
        return m_s_overall_goal;
    }
    std::string GetContextDelimitation()
    {
        return m_s_delimitation;
    }

    void AddUseCase(std::string s_principle,
                    std::string s_method,
                    std::string s_activity,
                    std::string s_sub_goal,
                    std::string s_use_case,
                    int i_risk_level,
                    double d_iol_reference)
    {
        WAIEthicalUseCase wai_ethical_use_case;
        wai_ethical_use_case.Initialize();
        wai_ethical_use_case.SetEthicalProperties(s_principle,s_method,s_activity,s_sub_goal,s_use_case,i_risk_level,d_iol_reference);
        vec_wai_ethical_use_cases.push_back(wai_ethical_use_case);
    }

    int GetUseCaseRiskLevel(std::string s_use_case)
    {
        for(int i=0;i<vec_wai_ethical_use_cases.size();i++)
        {
            if(vec_wai_ethical_use_cases[i].GetUseCase().compare(s_use_case)==0)
            {
                return vec_wai_ethical_use_cases[i].GetRiskLevel();
            }
        }
    }
    double GetUseCaseIOLReference(std::string s_use_case)
    {
        for(int i=0;i<vec_wai_ethical_use_cases.size();i++)
        {
            if(vec_wai_ethical_use_cases[i].GetUseCase().compare(s_use_case)==0)
            {
                return vec_wai_ethical_use_cases[i].GetIOLReference();
            }
        }
    }
    double GetUseCaseIOLActual(std::string s_use_case)
    {
        for(int i=0;i<vec_wai_ethical_use_cases.size();i++)
        {
            if(vec_wai_ethical_use_cases[i].GetUseCase().compare(s_use_case)==0)
            {
                return vec_wai_ethical_use_cases[i].GetIOLActual();
            }
        }
    }
};

#endif //WAI_ETHICAL_MODEL_H
