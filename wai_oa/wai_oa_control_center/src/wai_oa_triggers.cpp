#include<wai_oa.h>
#include<wai_oa_state.h>
#include<wai_oa_triggers.h>



/////////////////////////////////////////////////
/// Implementation of trigger base class
/////////////////////////////////////////////////
WAIAuditoriumTrigger::WAIAuditoriumTrigger()
{
}
WAIAuditoriumTrigger::~WAIAuditoriumTrigger()
{
}



/////////////////////////////////////////////////
/// Implementation of all individual triggers
/////////////////////////////////////////////////

// 1.SCENE Interactions
WAIAuditoriumTriggerSceneSelectPrev::WAIAuditoriumTriggerSceneSelectPrev(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerSceneSelectPrev::~WAIAuditoriumTriggerSceneSelectPrev()
{

}
void WAIAuditoriumTriggerSceneSelectPrev::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0
            && m_wai_auditorium->GetSceneCountCurrent()>1)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpScenePrev,true);
    }
}

WAIAuditoriumTriggerSceneSelectNext::WAIAuditoriumTriggerSceneSelectNext(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerSceneSelectNext::~WAIAuditoriumTriggerSceneSelectNext()
{

}
void WAIAuditoriumTriggerSceneSelectNext::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0
            && m_wai_auditorium->GetSceneCountCurrent()<m_wai_auditorium->GetSceneCountMax())
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpSceneNext,true);
    }
}

WAIAuditoriumTriggerSceneSelectNumber::WAIAuditoriumTriggerSceneSelectNumber(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerSceneSelectNumber::~WAIAuditoriumTriggerSceneSelectNumber()
{

}
void WAIAuditoriumTriggerSceneSelectNumber::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpSceneSelect,true);
    }
}



// 2.CAMERA Interactions
WAIAuditoriumTriggerCameraRvizDefault::WAIAuditoriumTriggerCameraRvizDefault(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerCameraRvizDefault::~WAIAuditoriumTriggerCameraRvizDefault()
{

}
void WAIAuditoriumTriggerCameraRvizDefault::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpCameraRvizDefault,true);
    }
}

WAIAuditoriumTriggerCameraRvizCycle::WAIAuditoriumTriggerCameraRvizCycle(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerCameraRvizCycle::~WAIAuditoriumTriggerCameraRvizCycle()
{

}
void WAIAuditoriumTriggerCameraRvizCycle::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpCameraRvizCycle,true);
    }
}

WAIAuditoriumTriggerCameraRvizSelect::WAIAuditoriumTriggerCameraRvizSelect(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerCameraRvizSelect::~WAIAuditoriumTriggerCameraRvizSelect()
{

}
void WAIAuditoriumTriggerCameraRvizSelect::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpCameraRvizSelect,true);
    }
}

WAIAuditoriumTriggerCameraRvizEnforce::WAIAuditoriumTriggerCameraRvizEnforce(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerCameraRvizEnforce::~WAIAuditoriumTriggerCameraRvizEnforce()
{

}
void WAIAuditoriumTriggerCameraRvizEnforce::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpCameraRvizEnforce,true);
    }
}

WAIAuditoriumTriggerCameraRvizIdle::WAIAuditoriumTriggerCameraRvizIdle(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerCameraRvizIdle::~WAIAuditoriumTriggerCameraRvizIdle()
{

}
void WAIAuditoriumTriggerCameraRvizIdle::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0
            && m_wai_auditorium->GetCameraRvizEnabled())
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpCameraRvizIdle);
        m_wai_auditorium->RequestSetupModel();
        m_wai_auditorium->RequestSetupView();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpCameraRvizIdle")==0
            && m_wai_auditorium->GetCameraRvizEnabled())
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}

WAIAuditoriumTriggerCameraRvizFollow::WAIAuditoriumTriggerCameraRvizFollow(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerCameraRvizFollow::~WAIAuditoriumTriggerCameraRvizFollow()
{

}
void WAIAuditoriumTriggerCameraRvizFollow::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpCameraRvizFollow,true);
    }
}



// 3.PRESENTER HELPER Interactions
WAIAuditoriumTriggerIntroduction::WAIAuditoriumTriggerIntroduction(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerIntroduction::~WAIAuditoriumTriggerIntroduction()
{

}
void WAIAuditoriumTriggerIntroduction::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpIntroduction);
        m_wai_auditorium->RequestSetupModel();
        m_wai_auditorium->RequestSetupView();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpIntroduction")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}

WAIAuditoriumTriggerPresenceMode::WAIAuditoriumTriggerPresenceMode(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerPresenceMode::~WAIAuditoriumTriggerPresenceMode()
{

}
void WAIAuditoriumTriggerPresenceMode::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpPresenceMode,true);
    }
}

WAIAuditoriumTriggerBodyInteraction::WAIAuditoriumTriggerBodyInteraction(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerBodyInteraction::~WAIAuditoriumTriggerBodyInteraction()
{

}
void WAIAuditoriumTriggerBodyInteraction::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpBodyInteraction,true);
    }
}

WAIAuditoriumTriggerAvatar::WAIAuditoriumTriggerAvatar(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAvatar::~WAIAuditoriumTriggerAvatar()
{

}
void WAIAuditoriumTriggerAvatar::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAvatar,true);
    }
}

WAIAuditoriumTriggerLectern::WAIAuditoriumTriggerLectern(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerLectern::~WAIAuditoriumTriggerLectern()
{

}
void WAIAuditoriumTriggerLectern::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpLectern,true);
    }
}

WAIAuditoriumTriggerTable::WAIAuditoriumTriggerTable(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerTable::~WAIAuditoriumTriggerTable()
{

}
void WAIAuditoriumTriggerTable::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpTable,true);
    }
}

WAIAuditoriumTriggerAudio::WAIAuditoriumTriggerAudio(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudio::~WAIAuditoriumTriggerAudio()
{

}
void WAIAuditoriumTriggerAudio::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudio,true);
    }
}

WAIAuditoriumTriggerSketch::WAIAuditoriumTriggerSketch(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerSketch::~WAIAuditoriumTriggerSketch()
{

}
void WAIAuditoriumTriggerSketch::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpSketch,true);
    }
}

WAIAuditoriumTriggerRespawn2D3D::WAIAuditoriumTriggerRespawn2D3D(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerRespawn2D3D::~WAIAuditoriumTriggerRespawn2D3D()
{

}
void WAIAuditoriumTriggerRespawn2D3D::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0
            && m_wai_auditorium->CheckCollisionTriggerRespawn2D3D())
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUp2D3DRespawn,true);
    }
}

WAIAuditoriumTriggerForce::WAIAuditoriumTriggerForce(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerForce::~WAIAuditoriumTriggerForce()
{

}
void WAIAuditoriumTriggerForce::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpForceInit);
        m_wai_auditorium->RequestSetupModel();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpForceInit")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}

WAIAuditoriumTriggerVoiceCommand::WAIAuditoriumTriggerVoiceCommand(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerVoiceCommand::~WAIAuditoriumTriggerVoiceCommand()
{

}
void WAIAuditoriumTriggerVoiceCommand::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpVoiceListen,true);
    }
}

WAIAuditoriumTriggerVoicePrompt::WAIAuditoriumTriggerVoicePrompt(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerVoicePrompt::~WAIAuditoriumTriggerVoicePrompt()
{

}
void WAIAuditoriumTriggerVoicePrompt::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpVoicePrompt,true);
    }
}



// 4.AUDIENCE Interactions (Evaluation, WIM navigation, aso.)
WAIAuditoriumTriggerAudienceRequestIncoming::WAIAuditoriumTriggerAudienceRequestIncoming(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceRequestIncoming::~WAIAuditoriumTriggerAudienceRequestIncoming()
{

}
void WAIAuditoriumTriggerAudienceRequestIncoming::Activate()
{
    // If in startup phase accept audience (connection) requests in general
    if(m_wai_auditorium->GetSessionStartupFinished()==false)
    {
        // Todo:
        /* Add blocking of IDs here!
        if(GetAudienceIDisBlocked())
        {

        }*/
        m_wai_auditorium->SetupReceivingRequestAudienceModel();
        m_wai_auditorium->SetupReceivingRequestAudienceView();
    }
    else
    {
        // Consider requests from audience only if presenting in plenum
        if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
        {
            m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
            m_wai_auditorium->StateTransitionTo(new StatePlenumReceivingRequestAudience,true);
        }
        else
        {
            // Notify audience that presenter is currently busy/occupied and don't add to queue
            int i_request_audience_id=m_wai_auditorium->GetAudienceIDFromRequestMsg(m_wai_auditorium->GetAudienceRequestIncoming().frame_id);
            m_wai_auditorium->SendInfoToAudienceID(i_request_audience_id,m_wai_auditorium->GetAudienceRequestIncoming().frame_id+" (OCC)");
        }
    }
}

WAIAuditoriumTriggerAudienceRequestReject::WAIAuditoriumTriggerAudienceRequestReject(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceRequestReject::~WAIAuditoriumTriggerAudienceRequestReject()
{

}
void WAIAuditoriumTriggerAudienceRequestReject::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumHandlingRequestAudienceReject,true);
    }
}

WAIAuditoriumTriggerAudienceRequestAccept::WAIAuditoriumTriggerAudienceRequestAccept(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceRequestAccept::~WAIAuditoriumTriggerAudienceRequestAccept()
{

}
void WAIAuditoriumTriggerAudienceRequestAccept::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumHandlingRequestAudienceAccept);
        m_wai_auditorium->RequestSetupModel();
        m_wai_auditorium->RequestSetupView();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumHandlingRequestAudienceAccept")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}

WAIAuditoriumTriggerAudienceFocusDown::WAIAuditoriumTriggerAudienceFocusDown(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceFocusDown::~WAIAuditoriumTriggerAudienceFocusDown()
{

}
void WAIAuditoriumTriggerAudienceFocusDown::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceDown,true);
    }
}

WAIAuditoriumTriggerAudienceFocusUp::WAIAuditoriumTriggerAudienceFocusUp(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceFocusUp::~WAIAuditoriumTriggerAudienceFocusUp()
{

}
void WAIAuditoriumTriggerAudienceFocusUp::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceUp,true);
    }
}

WAIAuditoriumTriggerAudienceFocusLeft::WAIAuditoriumTriggerAudienceFocusLeft(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceFocusLeft::~WAIAuditoriumTriggerAudienceFocusLeft()
{

}
void WAIAuditoriumTriggerAudienceFocusLeft::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceLeft,true);
    }
}

WAIAuditoriumTriggerAudienceFocusRight::WAIAuditoriumTriggerAudienceFocusRight(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceFocusRight::~WAIAuditoriumTriggerAudienceFocusRight()
{

}
void WAIAuditoriumTriggerAudienceFocusRight::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceRight,true);
    }
}

WAIAuditoriumTriggerAudienceFocusNumber::WAIAuditoriumTriggerAudienceFocusNumber(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceFocusNumber::~WAIAuditoriumTriggerAudienceFocusNumber()
{

}
void WAIAuditoriumTriggerAudienceFocusNumber::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceNumber,true);
    }
}

WAIAuditoriumTriggerAudienceSelect::WAIAuditoriumTriggerAudienceSelect(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceSelect::~WAIAuditoriumTriggerAudienceSelect()
{

}
void WAIAuditoriumTriggerAudienceSelect::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceSelect);
        m_wai_auditorium->RequestSetupModel();
        m_wai_auditorium->RequestSetupView();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpAudienceSelect")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}

WAIAuditoriumTriggerAudienceMessage::WAIAuditoriumTriggerAudienceMessage(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceMessage::~WAIAuditoriumTriggerAudienceMessage()
{

}
void WAIAuditoriumTriggerAudienceMessage::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceMessage,true);
    }
}

WAIAuditoriumTriggerAudienceListenerMessage::WAIAuditoriumTriggerAudienceListenerMessage(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceListenerMessage::~WAIAuditoriumTriggerAudienceListenerMessage()
{

}
void WAIAuditoriumTriggerAudienceListenerMessage::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceListenerMessage,true);
    }
}

WAIAuditoriumTriggerAudienceKick::WAIAuditoriumTriggerAudienceKick(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceKick::~WAIAuditoriumTriggerAudienceKick()
{

}
void WAIAuditoriumTriggerAudienceKick::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpAudienceKick,true);
    }
}



// 5.EVALUATION (Inspection) Interactions
WAIAuditoriumTriggerAudienceEvalPartMinus::WAIAuditoriumTriggerAudienceEvalPartMinus(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalPartMinus::~WAIAuditoriumTriggerAudienceEvalPartMinus()
{

}
void WAIAuditoriumTriggerAudienceEvalPartMinus::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDParticipationMinus,true);
    }
}

WAIAuditoriumTriggerAudienceEvalPartTilde::WAIAuditoriumTriggerAudienceEvalPartTilde(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalPartTilde::~WAIAuditoriumTriggerAudienceEvalPartTilde()
{

}
void WAIAuditoriumTriggerAudienceEvalPartTilde::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDParticipationTilde,true);
    }
}

WAIAuditoriumTriggerAudienceEvalPartPlus::WAIAuditoriumTriggerAudienceEvalPartPlus(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalPartPlus::~WAIAuditoriumTriggerAudienceEvalPartPlus()
{

}
void WAIAuditoriumTriggerAudienceEvalPartPlus::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDParticipationPlus,true);
    }
}

WAIAuditoriumTriggerAudienceEvalPartScore::WAIAuditoriumTriggerAudienceEvalPartScore(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalPartScore::~WAIAuditoriumTriggerAudienceEvalPartScore()
{

}
void WAIAuditoriumTriggerAudienceEvalPartScore::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDParticipationScore,true);
    }
}

WAIAuditoriumTriggerAudienceEvalExamInsufficient::WAIAuditoriumTriggerAudienceEvalExamInsufficient(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalExamInsufficient::~WAIAuditoriumTriggerAudienceEvalExamInsufficient()
{

}
void WAIAuditoriumTriggerAudienceEvalExamInsufficient::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDExaminationInsufficient,true);
    }
}

WAIAuditoriumTriggerAudienceEvalExamSufficient::WAIAuditoriumTriggerAudienceEvalExamSufficient(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalExamSufficient::~WAIAuditoriumTriggerAudienceEvalExamSufficient()
{

}
void WAIAuditoriumTriggerAudienceEvalExamSufficient::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDExaminationSufficient,true);
    }
}

WAIAuditoriumTriggerAudienceEvalExamSatisfactory::WAIAuditoriumTriggerAudienceEvalExamSatisfactory(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalExamSatisfactory::~WAIAuditoriumTriggerAudienceEvalExamSatisfactory()
{

}
void WAIAuditoriumTriggerAudienceEvalExamSatisfactory::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDExaminationSatisfactory,true);
    }
}

WAIAuditoriumTriggerAudienceEvalExamGood::WAIAuditoriumTriggerAudienceEvalExamGood(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalExamGood::~WAIAuditoriumTriggerAudienceEvalExamGood()
{

}
void WAIAuditoriumTriggerAudienceEvalExamGood::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDExaminationGood,true);
    }
}

WAIAuditoriumTriggerAudienceEvalExamVeryGood::WAIAuditoriumTriggerAudienceEvalExamVeryGood(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalExamVeryGood::~WAIAuditoriumTriggerAudienceEvalExamVeryGood()
{

}
void WAIAuditoriumTriggerAudienceEvalExamVeryGood::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDExaminationVeryGood,true);
    }
}

WAIAuditoriumTriggerAudienceEvalExamScore::WAIAuditoriumTriggerAudienceEvalExamScore(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalExamScore::~WAIAuditoriumTriggerAudienceEvalExamScore()
{

}
void WAIAuditoriumTriggerAudienceEvalExamScore::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumLogAudienceIDExaminationScore,true);
    }
}

WAIAuditoriumTriggerAudienceEvalGraph::WAIAuditoriumTriggerAudienceEvalGraph(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalGraph::~WAIAuditoriumTriggerAudienceEvalGraph()
{

}
void WAIAuditoriumTriggerAudienceEvalGraph::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    }
}

WAIAuditoriumTriggerAudienceEvalGraphInspect::WAIAuditoriumTriggerAudienceEvalGraphInspect(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalGraphInspect::~WAIAuditoriumTriggerAudienceEvalGraphInspect()
{

}
void WAIAuditoriumTriggerAudienceEvalGraphInspect::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpEvalGraphInspect,true);
    }
}

WAIAuditoriumTriggerAudienceEvalMetaphoreBowl::WAIAuditoriumTriggerAudienceEvalMetaphoreBowl(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalMetaphoreBowl::~WAIAuditoriumTriggerAudienceEvalMetaphoreBowl()
{

}
void WAIAuditoriumTriggerAudienceEvalMetaphoreBowl::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpEvalBowl);
        m_wai_auditorium->RequestSetupModel();
        m_wai_auditorium->RequestSetupView();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpEvalBowl")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}

WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin::WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin::~WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin()
{

}
void WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpEvalMarvin);
        m_wai_auditorium->RequestSetupModel();
        m_wai_auditorium->RequestSetupView();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpEvalMarvin")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}

WAIAuditoriumTriggerAudienceEvalMetaphoreBalance::WAIAuditoriumTriggerAudienceEvalMetaphoreBalance(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerAudienceEvalMetaphoreBalance::~WAIAuditoriumTriggerAudienceEvalMetaphoreBalance()
{

}
void WAIAuditoriumTriggerAudienceEvalMetaphoreBalance::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpEvalBalance);
        m_wai_auditorium->RequestSetupModel();
        m_wai_auditorium->RequestSetupView();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpEvalBalance")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}



// 6.REP Interactions
WAIAuditoriumTriggerRepDetailFocus::WAIAuditoriumTriggerRepDetailFocus(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerRepDetailFocus::~WAIAuditoriumTriggerRepDetailFocus()
{

}
void WAIAuditoriumTriggerRepDetailFocus::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpRepDetailFocus,true);
    }
}

WAIAuditoriumTriggerRepSelect::WAIAuditoriumTriggerRepSelect(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerRepSelect::~WAIAuditoriumTriggerRepSelect()
{

}
void WAIAuditoriumTriggerRepSelect::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StatePlenumSettingUpRepSelect);
        m_wai_auditorium->RequestSetupModel();
        m_wai_auditorium->RequestSetupView();
    }
    else if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpRepSelect")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->RequestLeaveState();
    }
}

WAIAuditoriumTriggerRepDetailRot::WAIAuditoriumTriggerRepDetailRot(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerRepDetailRot::~WAIAuditoriumTriggerRepDetailRot()
{

}
void WAIAuditoriumTriggerRepDetailRot::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumSettingUpRepSelect")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->ToggleRepDetailRotation();
    }
}



// 7.LEARNING MODE Interactions
WAIAuditoriumTriggerLearningModePlen::WAIAuditoriumTriggerLearningModePlen(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerLearningModePlen::~WAIAuditoriumTriggerLearningModePlen()
{

}
void WAIAuditoriumTriggerLearningModePlen::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StateLearningModePlenumPrepare,true);
    }
}

WAIAuditoriumTriggerLearningModeCoop::WAIAuditoriumTriggerLearningModeCoop(WAIOpenAuditorium* wai_auditorium)
{
    m_wai_auditorium=wai_auditorium;
}
WAIAuditoriumTriggerLearningModeCoop::~WAIAuditoriumTriggerLearningModeCoop()
{

}
void WAIAuditoriumTriggerLearningModeCoop::Activate()
{
    if(m_wai_auditorium->GetStateNameCurrent().compare("PlenumPresenting")==0)
    {
        m_wai_auditorium->SetTimeTriggerActivated(ros::Time::now());
        m_wai_auditorium->StateTransitionTo(new StateLearningModeCooperativePrepare,true);
    }
}





/////////////////////////////////////////////////
/// Implementation of WAIOATriggers
/////////////////////////////////////////////////
WAIOATriggers::WAIOATriggers()
{

}

WAIOATriggers::~WAIOATriggers()
{
    XCloseDisplay(dsp_x11_display);
}

void WAIOATriggers::Initialize(ros::NodeHandle* hdl_node,
                               WAIOpenAuditorium* wai_open_auditorium,
                               float f_node_sample_frequency,
                               float f_trigger_timeout)
{
    m_hdl_node=hdl_node;
    m_f_node_sample_frequency=f_node_sample_frequency;

    m_wai_open_auditorium=wai_open_auditorium;

    dsp_x11_display=XOpenDisplay(NULL); // XOpenDisplay(":0");

    m_msg_str_trigger_status.data="X";

    m_f_trigger_timeout=f_trigger_timeout;
    m_b_triggers_available=false;
    m_s_oa_event_inpdev_last="N/A";


    /* INTERACTIONS SUBDIVIDED INTO:
     * 1.SCENE Interactions
     * 2.CAMERA Interactions
     * 3.PRESENTER HELPER Interactions
     * 4.AUDIENCE Interactions (Evaluation, WIM navigation, aso.)
     * 5.EVALUATION (Inspection) Interactions
     * 6.REP Interactions (Focus, selection, details, aso.)
     * 7.LEARNING MODE Interactions
     *
     * Available input devices:
     * KEY...Keyboard
     * PAC...Point And Click
     * JOY...Joypad
     * VIV...HTC Vive
     * MOB...Mobile Device (App)
     * VOI...Voice Command Interface
     * BOI...Body Interaction (3D-Sensor/RGBD-Camera)
    */


    //========================
    // Initialize INPUTS from
    // KEYBOARD
    //========================
    // 1.SCENE Interactions
    m_trg_scene_select_prev=new WAIAuditoriumTriggerSceneSelectPrev(m_wai_open_auditorium); // KEY JOY MOB PAC VOI BOI
    m_trg_scene_select_next=new WAIAuditoriumTriggerSceneSelectNext(m_wai_open_auditorium); // KEY JOY MOB PAC VOI BOI
    m_trg_scene_select_number=new WAIAuditoriumTriggerSceneSelectNumber(m_wai_open_auditorium); // KEY
    // 2.CAMERA Interactions
    m_trg_camera_rviz_default=new WAIAuditoriumTriggerCameraRvizDefault(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_camera_rviz_cycle=new WAIAuditoriumTriggerCameraRvizCycle(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_camera_rviz_select=new WAIAuditoriumTriggerCameraRvizSelect(m_wai_open_auditorium); // KEY
    m_trg_camera_rviz_enforce=new WAIAuditoriumTriggerCameraRvizEnforce(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_camera_rviz_idle=new WAIAuditoriumTriggerCameraRvizIdle(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_camera_rviz_follow=new WAIAuditoriumTriggerCameraRvizFollow(m_wai_open_auditorium); // KEY JOY MOB VOI
    // 3.PRESENTER HELPER Interactions
    m_trg_introduction=new WAIAuditoriumTriggerIntroduction(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_presence_mode=new WAIAuditoriumTriggerPresenceMode(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_body_interaction=new WAIAuditoriumTriggerBodyInteraction(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_avatar=new WAIAuditoriumTriggerAvatar(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_lectern=new WAIAuditoriumTriggerLectern(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_table=new WAIAuditoriumTriggerTable(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audio=new WAIAuditoriumTriggerAudio(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_sketch=new WAIAuditoriumTriggerSketch(m_wai_open_auditorium); // KEY
    m_trg_respawn_2d3d=new WAIAuditoriumTriggerRespawn2D3D(m_wai_open_auditorium); // JOY
    m_trg_force=new WAIAuditoriumTriggerForce(m_wai_open_auditorium); // JOY
    m_trg_voice_command=new WAIAuditoriumTriggerVoiceCommand(m_wai_open_auditorium); // KEY JOY MOB
    m_trg_voice_prompt=new WAIAuditoriumTriggerVoicePrompt(m_wai_open_auditorium); // KEY JOY MOB
    // 4.AUDIENCE Interactions
    m_trg_audience_request_incoming=new WAIAuditoriumTriggerAudienceRequestIncoming(m_wai_open_auditorium); // EVT/MSG!
    m_trg_audience_request_reject=new WAIAuditoriumTriggerAudienceRequestReject(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_request_accept=new WAIAuditoriumTriggerAudienceRequestAccept(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_focus_down=new WAIAuditoriumTriggerAudienceFocusDown(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_focus_up=new WAIAuditoriumTriggerAudienceFocusUp(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_focus_left=new WAIAuditoriumTriggerAudienceFocusLeft(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_focus_right=new WAIAuditoriumTriggerAudienceFocusRight(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_focus_number=new WAIAuditoriumTriggerAudienceFocusNumber(m_wai_open_auditorium); // KEY
    m_trg_audience_select=new WAIAuditoriumTriggerAudienceSelect(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_message=new WAIAuditoriumTriggerAudienceMessage(m_wai_open_auditorium); // KEY
    m_trg_audience_listener_message=new WAIAuditoriumTriggerAudienceListenerMessage(m_wai_open_auditorium); // KEY
    m_trg_audience_kick=new WAIAuditoriumTriggerAudienceKick(m_wai_open_auditorium); // KEY
    // 5.EVALUATION Interactions
    m_trg_audience_eval_part_minus=new WAIAuditoriumTriggerAudienceEvalPartMinus(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_eval_part_tilde=new WAIAuditoriumTriggerAudienceEvalPartTilde(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_eval_part_plus=new WAIAuditoriumTriggerAudienceEvalPartPlus(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_eval_part_score=new WAIAuditoriumTriggerAudienceEvalPartScore(m_wai_open_auditorium); // KEY JOY
    m_trg_audience_eval_exam_insufficient=new WAIAuditoriumTriggerAudienceEvalExamInsufficient(m_wai_open_auditorium); // KEY MOB VOI
    m_trg_audience_eval_exam_sufficient=new WAIAuditoriumTriggerAudienceEvalExamSufficient(m_wai_open_auditorium); // KEY MOB VOI
    m_trg_audience_eval_exam_satisfactory=new WAIAuditoriumTriggerAudienceEvalExamSatisfactory(m_wai_open_auditorium); // KEY MOB VOI
    m_trg_audience_eval_exam_good=new WAIAuditoriumTriggerAudienceEvalExamGood(m_wai_open_auditorium); // KEY MOB VOI
    m_trg_audience_eval_exam_very_good=new WAIAuditoriumTriggerAudienceEvalExamVeryGood(m_wai_open_auditorium); // KEY MOB VOI
    m_trg_audience_eval_exam_score=new WAIAuditoriumTriggerAudienceEvalExamScore(m_wai_open_auditorium); // KEY JOY
    m_trg_audience_eval_graph=new WAIAuditoriumTriggerAudienceEvalGraph(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_audience_eval_graph_inspect=new WAIAuditoriumTriggerAudienceEvalGraphInspect(m_wai_open_auditorium); // PAC
    m_trg_audience_eval_metaphore_bowl=new WAIAuditoriumTriggerAudienceEvalMetaphoreBowl(m_wai_open_auditorium); // KEY MOB VOI
    m_trg_audience_eval_metaphore_marvin=new WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin(m_wai_open_auditorium); // KEY MOB VOI
    m_trg_audience_eval_metaphore_balance=new WAIAuditoriumTriggerAudienceEvalMetaphoreBalance(m_wai_open_auditorium); // KEY MOB VOI
    // 6.REP Interactions
    m_trg_rep_detail_focus=new WAIAuditoriumTriggerRepDetailFocus(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_rep_detail_rot=new WAIAuditoriumTriggerRepDetailRot(m_wai_open_auditorium); // KEY JOY MOB VOI
    m_trg_rep_detail_select=new WAIAuditoriumTriggerRepSelect(m_wai_open_auditorium); // KEY JOY MOB PAC VOI
    // 7.LEARNING MODE Interactions
    m_trg_learning_mode_plen=new WAIAuditoriumTriggerLearningModePlen(m_wai_open_auditorium); // KEY MOB VOI
    m_trg_learning_mode_coop=new WAIAuditoriumTriggerLearningModeCoop(m_wai_open_auditorium); // KEY MOB VOI


    //========================
    // Initialize INPUTS from
    // POINT AND CLICK
    //========================
    sub_pts_rviz_clicked=m_hdl_node->subscribe("/wai_world/world/clicked_point",1,&WAIOATriggers::cb_sub_pts_rviz_clicked,this);
    m_msg_pts_rviz_clicked.header.frame_id="world";
    m_msg_pts_rviz_clicked.header.stamp=ros::Time::now();
    m_msg_pts_rviz_clicked.point.x=0.0;
    m_msg_pts_rviz_clicked.point.y=0.0;
    m_msg_pts_rviz_clicked.point.z=0.0;
    m_b_pts_rviz_clicked_received=false;


    //========================
    // Initialize INPUTS from
    // JOYPAD
    //========================
    m_sub_joy_controller=m_hdl_node->subscribe("/wai_world/world/joy",1, &WAIOATriggers::cb_m_sub_joy_controller,this);
    m_pub_jfa_joy_input_dev_feedback=m_hdl_node->advertise<sensor_msgs::JoyFeedbackArray>("/wai_world/world/joy/set_feedback",1);
    m_tmr_trigger_joypad_feedback=m_hdl_node->createTimer(ros::Duration(0.5),&WAIOATriggers::cb_tmr_trigger_joypad_feedback,this,true,false);


    //========================
    // Initialize INPUTS from
    // HTC VIVE
    //========================
    m_sub_htc_vive_controller_left=m_hdl_node->subscribe("/wai_world/world/htc_vive/left_controller/state",1,&WAIOATriggers::cb_m_sub_htc_vive_controller_left,this);
    m_pub_vib_htc_vive_input_dev_feedback_left=m_hdl_node->advertise<rviz_vive_plugin_msgs::ControllerVibration>("/wai_world/world/htc_vive/left_controller_vibration",1);
    m_sub_htc_vive_controller_right=m_hdl_node->subscribe("/wai_world/world/htc_vive/right_controller/state",1,&WAIOATriggers::cb_m_sub_htc_vive_controller_right,this);
    m_pub_vib_htc_vive_input_dev_feedback_right=m_hdl_node->advertise<rviz_vive_plugin_msgs::ControllerVibration>("/wai_world/world/htc_vive/right_controller_vibration",1);

    //========================
    // Initialize INPUTS from
    // MOBILE DEVICE
    //========================
    // 1.SCENE Interactions
    sub_bol_trg_mob_scene_select_prev=m_hdl_node->subscribe("trg_mob_scene_select_prev", 1, &WAIOATriggers::cb_sub_bol_trg_mob_scene_select_prev,this);
    sub_bol_trg_mob_scene_select_next=m_hdl_node->subscribe("trg_mob_scene_select_next", 1, &WAIOATriggers::cb_sub_bol_trg_mob_scene_select_next,this);
    sub_bol_trg_mob_scene_select_number=m_hdl_node->subscribe("trg_mob_scene_select_number", 1, &WAIOATriggers::cb_sub_bol_trg_mob_scene_select_number,this);
    // 2.CAMERA Interactions
    sub_bol_trg_mob_camera_rviz_default=m_hdl_node->subscribe("trg_mob_camera_rviz_default", 1, &WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_default,this);
    sub_bol_trg_mob_camera_rviz_cycle=m_hdl_node->subscribe("trg_mob_camera_rviz_cycle", 1, &WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_cycle,this);
    sub_bol_trg_mob_camera_rviz_select=m_hdl_node->subscribe("trg_mob_camera_rviz_select", 1, &WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_select,this);
    sub_bol_trg_mob_camera_rviz_enforce=m_hdl_node->subscribe("trg_mob_camera_rviz_enforce", 1, &WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_enforce,this);
    sub_bol_trg_mob_camera_rviz_idle=m_hdl_node->subscribe("trg_mob_camera_rviz_idle", 1, &WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_idle,this);
    sub_bol_trg_mob_camera_rviz_follow=m_hdl_node->subscribe("trg_mob_camera_rviz_follow", 1, &WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_follow,this);
    // 3.PRESENTER HELPER Interactions
    sub_bol_trg_mob_introduction=m_hdl_node->subscribe("trg_mob_introduction", 1, &WAIOATriggers::cb_sub_bol_trg_mob_introduction,this);
    sub_bol_trg_mob_presence_mode=m_hdl_node->subscribe("trg_mob_presence_mode", 1, &WAIOATriggers::cb_sub_bol_trg_mob_presence_mode,this);
    sub_bol_trg_mob_body_interaction=m_hdl_node->subscribe("trg_mob_body_interaction", 1, &WAIOATriggers::cb_sub_bol_trg_mob_body_interaction,this);
    sub_bol_trg_mob_avatar=m_hdl_node->subscribe("trg_mob_avatar", 1, &WAIOATriggers::cb_sub_bol_trg_mob_avatar,this);
    sub_bol_trg_mob_lectern=m_hdl_node->subscribe("trg_mob_lectern", 1, &WAIOATriggers::cb_sub_bol_trg_mob_lectern,this);
    sub_bol_trg_mob_table=m_hdl_node->subscribe("trg_mob_table", 1, &WAIOATriggers::cb_sub_bol_trg_mob_table,this);
    sub_bol_trg_mob_audio=m_hdl_node->subscribe("trg_mob_audio", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audio,this);
    sub_bol_trg_mob_voice_command=m_hdl_node->subscribe("trg_mob_voice_command", 1, &WAIOATriggers::cb_sub_bol_trg_mob_voice_command,this);
    sub_bol_trg_mob_voice_prompt=m_hdl_node->subscribe("trg_mob_voice_prompt", 1, &WAIOATriggers::cb_sub_bol_trg_mob_voice_prompt,this);
    // 4.AUDIENCE Interactions
    // Unused: m_trg_audience_request_incoming
    sub_bol_trg_mob_audience_request_reject=m_hdl_node->subscribe("trg_mob_audience_request_reject", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_request_reject,this);
    sub_bol_trg_mob_audience_request_accept=m_hdl_node->subscribe("trg_mob_audience_request_accept", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_request_accept,this);
    sub_bol_trg_mob_audience_focus_down=m_hdl_node->subscribe("trg_mob_audience_focus_down", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_focus_down,this);
    sub_bol_trg_mob_audience_focus_up=m_hdl_node->subscribe("trg_mob_audience_focus_up", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_focus_up,this);
    sub_bol_trg_mob_audience_focus_left=m_hdl_node->subscribe("trg_mob_audience_focus_left", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_focus_left,this);
    sub_bol_trg_mob_audience_focus_right=m_hdl_node->subscribe("trg_mob_audience_focus_right", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_focus_right,this);
    // Unused: m_trg_audience_focus_number
    sub_bol_trg_mob_audience_select=m_hdl_node->subscribe("trg_mob_audience_select", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_select,this);
    // Unused: m_trg_audience_message
    // Unused: m_trg_audience_kick
    // 5.EVALUATION Interactions
    sub_bol_trg_mob_audience_eval_part_minus=m_hdl_node->subscribe("trg_mob_audience_eval_part_minus", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_part_minus,this);
    sub_bol_trg_mob_audience_eval_part_tilde=m_hdl_node->subscribe("trg_mob_audience_eval_part_tilde", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_part_tilde,this);
    sub_bol_trg_mob_audience_eval_part_plus=m_hdl_node->subscribe("trg_mob_audience_eval_part_plus", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_part_plus,this);
    // Unused: m_trg_audience_eval_part_score
    sub_bol_trg_mob_audience_eval_exam_insufficient=m_hdl_node->subscribe("trg_mob_audience_eval_exam_insufficient", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_insufficient,this);
    sub_bol_trg_mob_audience_eval_exam_sufficient=m_hdl_node->subscribe("trg_mob_audience_eval_exam_sufficient", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_sufficient,this);
    sub_bol_trg_mob_audience_eval_exam_satisfactory=m_hdl_node->subscribe("trg_mob_audience_eval_exam_satisfactory", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_satisfactory,this);
    sub_bol_trg_mob_audience_eval_exam_good=m_hdl_node->subscribe("trg_mob_audience_eval_exam_good", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_good,this);
    sub_bol_trg_mob_audience_eval_exam_very_good=m_hdl_node->subscribe("trg_mob_audience_eval_exam_very_good", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_very_good,this);
    // Unused: m_trg_audience_eval_exam_score
    sub_bol_trg_mob_audience_eval_graph=m_hdl_node->subscribe("trg_mob_audience_eval_graph", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_graph,this);
    // Unused: m_trg_audience_eval_graph_inspect
    sub_bol_trg_mob_audience_eval_metaphore_bowl=m_hdl_node->subscribe("trg_mob_audience_eval_metaphore_bowl", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_metaphore_bowl,this);
    sub_bol_trg_mob_audience_eval_metaphore_marvin=m_hdl_node->subscribe("trg_mob_audience_eval_metaphore_marvin", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_metaphore_marvin,this);
    sub_bol_trg_mob_audience_eval_metaphore_balance=m_hdl_node->subscribe("trg_mob_audience_eval_metaphore_balance", 1, &WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_metaphore_balance,this);
    // 6.REP Interactions
    sub_bol_trg_mob_rep_detail_focus=m_hdl_node->subscribe("trg_mob_rep_detail_focus", 1, &WAIOATriggers::cb_sub_bol_trg_mob_rep_detail_focus,this);
    sub_bol_trg_mob_rep_detail_rot=m_hdl_node->subscribe("trg_mob_rep_detail_rot", 1, &WAIOATriggers::cb_sub_bol_trg_mob_rep_detail_rot,this);
    sub_bol_trg_mob_rep_detail_select=m_hdl_node->subscribe("trg_mob_rep_select", 1, &WAIOATriggers::cb_sub_bol_trg_mob_rep_detail_select,this);
    // 7.LEARNING MODE Interactions
    sub_bol_trg_mob_learning_mode_plen=m_hdl_node->subscribe("trg_mob_learning_mode_plen", 1, &WAIOATriggers::cb_sub_bol_trg_mob_learning_mode_plen,this);
    sub_bol_trg_mob_learning_mode_coop=m_hdl_node->subscribe("trg_mob_learning_mode_coop", 1, &WAIOATriggers::cb_sub_bol_trg_mob_learning_mode_coop,this);


    //========================
    // Initialize INPUTS from
    // VOICE COMMANDS
    //========================
    sub_pic_intent_action_result=m_hdl_node->subscribe("/wai_world/world/get_intent/result", 1, &WAIOATriggers::cb_sub_pic_intent_action_result,this);


    //=====================================
    // Initialize INPUTS from
    // EXTERNAL EVENTS (Audience Requests)
    //=====================================
    sub_hea_audience_request=m_hdl_node->subscribe("audience_request",10,&WAIOATriggers::cb_sub_hea_audience_request,this);
    m_b_hea_audience_request_received=false;

    // Initialize other helper PUBLISHERS and SUBSCRIBERS
    m_pub_trigger_status=m_hdl_node->advertise<std_msgs::String>("/wai_world/world/oa_trigger_status",1);

    ResetBoolEventTriggersMobile();
    ResetBoolEventTriggersVoice();
}

void WAIOATriggers::UpdateModel()
{

}

void WAIOATriggers::UpdateView()
{

}



///////////////////////////////////////////
/// CHECK FOR ALL TRIGGERS
///////////////////////////////////////////
void WAIOATriggers::CheckTriggersAll()
{
    if(m_wai_open_auditorium->GetSessionStartupFinished()
        && GetTriggersAvailable())
    {
        //CheckTriggersEvents(); handled directly by callback to avoid "loosing" request events from audience!
        CheckTriggersKeyboard();
        CheckTriggersMobile();
        CheckTriggersVoice();
        CheckTriggersPointAndClick();
        CheckTriggersJoypad();
        CheckTriggersHTCVive();
        CheckTriggersBodyInteraction();

        m_msg_str_trigger_status.data="TRG: Tick";
        m_pub_trigger_status.publish(m_msg_str_trigger_status);
    }
}
std::string WAIOATriggers::GetTriggersLast()
{
    return m_s_oa_event_inpdev_last;
}
bool WAIOATriggers::GetTriggersAvailable()
{
    m_b_triggers_available=(ros::Time::now()-m_wai_open_auditorium->GetTimeTriggerActivated()).toSec()>m_f_trigger_timeout;
    return m_b_triggers_available;
}



/////////////////////////////////////////////////////
/// Methods to receive inputs from KEYBOARD
/////////////////////////////////////////////////////
void WAIOATriggers::CheckTriggersKeyboard()
{
    // MODIFIER
    bool b_keyboard_modifier=KeyIsPressed(XK_Tab);
    bool b_keyboard_modifier_alt=KeyIsPressed(XK_Caps_Lock);
    //Old modifiers: KeyIsPressed(XK_Control_L) && KeyIsPressed(XK_Alt_L);

    // BASIC Interactions
    /*
    if(b_keyboard_modifier && KeyIsPressed(XK_F4))
    {
        int i_return_val=system("rosnode kill -a");
    }
    */
    if(b_keyboard_modifier && KeyIsPressed(XK_Left))
    {
        m_s_oa_event_inpdev_last="[KEY] SCENE Previous";
        m_trg_scene_select_prev->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Right))
    {
        m_s_oa_event_inpdev_last="[KEY] SCENE Next";
        m_trg_scene_select_next->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Shift_L))
    {
        m_s_oa_event_inpdev_last="[KEY] SCENE Number";
        m_trg_scene_select_number->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Down))
    {
        m_s_oa_event_inpdev_last="[KEY] CAMERA Default";
        m_trg_camera_rviz_default->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Up))
    {
        m_s_oa_event_inpdev_last="[KEY] CAMERA Cylce";
        m_trg_camera_rviz_cycle->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_y))
    {
        m_s_oa_event_inpdev_last="[KEY] CAMERA Select";
        m_trg_camera_rviz_select->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_z))
    {
        m_s_oa_event_inpdev_last="[KEY] CAMERA Enforce";
        m_trg_camera_rviz_enforce->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Escape))
    {
        m_s_oa_event_inpdev_last="[KEY] CAMERA Idle";
        m_trg_camera_rviz_idle->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_f))
    {
        m_s_oa_event_inpdev_last="[KEY] CAMERA Follow";
        m_trg_camera_rviz_follow->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_i)) // INTRODUCTION
    {
        m_s_oa_event_inpdev_last="[KEY] INTRO";
        m_trg_introduction->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_o))
    {
        m_s_oa_event_inpdev_last="[KEY] PRESENCE";
        m_trg_presence_mode->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_b))
    {
        m_s_oa_event_inpdev_last="[KEY] BODY INT";
        m_trg_body_interaction->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_v))
    {
        m_s_oa_event_inpdev_last="[KEY] AVATAR";
        m_trg_avatar->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_l))
    {
        m_s_oa_event_inpdev_last="[KEY] LECTERN";
        m_trg_lectern->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_t))
    {
        m_s_oa_event_inpdev_last="[KEY] TABLE";
        m_trg_table->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_q))
    {
        m_s_oa_event_inpdev_last="[KEY] AUDIO";
        m_trg_audio->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_p))
    {
        m_s_oa_event_inpdev_last="[KEY] SKETCH";
        m_trg_sketch->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_c))
    {
        m_s_oa_event_inpdev_last="[KEY] VOICE Command";
        m_trg_voice_command->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_x))
    {
        m_s_oa_event_inpdev_last="[KEY] VOICE Prompt";
        m_trg_voice_prompt->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Insert))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD REQ Reject";
        m_trg_audience_request_reject->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Home))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD REQ Accept";
        m_trg_audience_request_accept->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_s))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD FOC Down";
        m_trg_audience_focus_down->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_w))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD FOC Up";
        m_trg_audience_focus_up->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_a))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD FOC Left";
        m_trg_audience_focus_left->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_d))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD FOC Right";
        m_trg_audience_focus_right->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_less))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD FOC Num";
        m_trg_audience_focus_number->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Shift_R))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD Select";
        m_trg_audience_select->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_m))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD Message";
        m_trg_audience_message->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_n))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD LST Message";
        m_trg_audience_listener_message->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_comma))
    {
        m_s_oa_event_inpdev_last="[KEY] AUD Kick";
        m_trg_audience_kick->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_minus))
    {
        m_s_oa_event_inpdev_last="[KEY] PART Minus";
        m_trg_audience_eval_part_minus->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_numbersign))
    {
        m_s_oa_event_inpdev_last="[KEY] PART Tilde";
        m_trg_audience_eval_part_tilde->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_plus))
    {
        m_s_oa_event_inpdev_last="[KEY] PART Plus";
        m_trg_audience_eval_part_plus->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_period))
    {
        m_s_oa_event_inpdev_last="[KEY] PART Score";
        m_trg_audience_eval_part_score->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_5))
    {
        m_s_oa_event_inpdev_last="[KEY] EXAM Unsatisfactory";
        m_trg_audience_eval_exam_insufficient->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_4))
    {
        m_s_oa_event_inpdev_last="[KEY] EXAM Sufficient";
        m_trg_audience_eval_exam_sufficient->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_3))
    {
        m_s_oa_event_inpdev_last="[KEY] EXAM Satisfactory";
        m_trg_audience_eval_exam_satisfactory->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_2))
    {
        m_s_oa_event_inpdev_last="[KEY] EXAM Good";
        m_trg_audience_eval_exam_good->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_1))
    {
        m_s_oa_event_inpdev_last="[KEY] EXAM Very Good";
        m_trg_audience_eval_exam_very_good->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_0))
    {
        m_s_oa_event_inpdev_last="[KEY] EXAM Score";
        m_trg_audience_eval_exam_score->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_e))
    {
        m_s_oa_event_inpdev_last="[KEY] EVAL Graph";
        m_trg_audience_eval_graph->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_6))
    {
        m_s_oa_event_inpdev_last="[KEY] EVAL Bowl";
        m_trg_audience_eval_metaphore_bowl->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_7))
    {
        m_s_oa_event_inpdev_last="[KEY] EVAL Marvin";
        m_trg_audience_eval_metaphore_marvin->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_8))
    {
        m_s_oa_event_inpdev_last="[KEY] EVAL Balance";
        m_trg_audience_eval_metaphore_balance->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_j))
    {
        //m_s_oa_event_inpdev_last="[KEY] ...";
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_u))
    {
        //m_s_oa_event_inpdev_last="[KEY] ...";
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_h))
    {
        m_s_oa_event_inpdev_last="[KEY] DET Previous";
        m_trg_rep_detail_focus->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_k))
    {
        //m_s_oa_event_inpdev_last="[KEY] ...";
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_r))
    {
        m_s_oa_event_inpdev_last="[KEY] DET Rot";
        m_trg_rep_detail_rot->Activate();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_g))
    {
        m_s_oa_event_inpdev_last="[KEY] REP DET Select";
        m_trg_rep_detail_select->Activate();
    }
    else if(b_keyboard_modifier_alt && KeyIsPressed(XK_1))
    {
        m_s_oa_event_inpdev_last="[KEY] MODE Plenum";
        m_trg_learning_mode_plen->Activate();
    }
    else if(b_keyboard_modifier_alt && KeyIsPressed(XK_2))
    {
        m_s_oa_event_inpdev_last="[KEY] MODE Cooperative";
        m_trg_learning_mode_coop->Activate();
    }
    else
    {
        // Do nothing...
    }
}
bool WAIOATriggers::KeyIsPressed(KeySym ks)
{
    char keys_return[32];
    XQueryKeymap(dsp_x11_display, keys_return);
    KeyCode kc2=XKeysymToKeycode(dsp_x11_display, ks);
    bool isPressed= !!(keys_return[kc2 >> 3] & (1 << (kc2 & 7)));
    return isPressed;
}



/////////////////////////////////////////////////////
/// Methods to receive inputs from POINT AND CLICK
/////////////////////////////////////////////////////
void WAIOATriggers::cb_sub_pts_rviz_clicked(const geometry_msgs::PointStampedPtr& msg)
{
    m_msg_pts_rviz_clicked=*msg;
    m_wai_open_auditorium->SetRvizPointClicked(m_msg_pts_rviz_clicked);
    m_b_pts_rviz_clicked_received=true;
}
void WAIOATriggers::CheckTriggersPointAndClick()
{
    if((ros::Time::now()-m_msg_pts_rviz_clicked.header.stamp).toSec()<(m_f_trigger_timeout/6.0)
        && m_b_pts_rviz_clicked_received==true)
    {
        m_b_pts_rviz_clicked_received=false;
        if(m_wai_open_auditorium->CheckCollisionTriggerScenePrev())
        {
            m_msg_pts_rviz_clicked.point.x=0.0;
            m_msg_pts_rviz_clicked.point.y=0.0;
            m_msg_pts_rviz_clicked.point.x=0.0;
            m_wai_open_auditorium->SetRvizPointClicked(m_msg_pts_rviz_clicked);
            m_s_oa_event_inpdev_last="[PAC] SCENE Previous";
            m_trg_scene_select_prev->Activate();
        }
        else if(m_wai_open_auditorium->CheckCollisionTriggerSceneNext())
        {
            m_msg_pts_rviz_clicked.point.x=0.0;
            m_msg_pts_rviz_clicked.point.y=0.0;
            m_msg_pts_rviz_clicked.point.x=0.0;
            m_wai_open_auditorium->SetRvizPointClicked(m_msg_pts_rviz_clicked);
            m_s_oa_event_inpdev_last="[PAC] SCENE Next";
            m_trg_scene_select_next->Activate();
        }
        else if(m_wai_open_auditorium->CheckCollisionTriggerRepSelect())
        {
            m_s_oa_event_inpdev_last="[PAC] REP DET Select";
            m_trg_rep_detail_select->Activate();
        }
        else if(m_msg_pts_rviz_clicked.point.z>=15.0) // Eval. graph starts at z=15.0 in world frame
        {
            m_s_oa_event_inpdev_last="[PAC] GRAPH EVAL Inspect";
            m_trg_audience_eval_graph_inspect->Activate();
        }
        else
        {
            // Do nothing...
        }
    }
}



/////////////////////////////////////////////////
/// Callback to receive inputs from JOYPAD
/////////////////////////////////////////////////
void WAIOATriggers::cb_m_sub_joy_controller(const sensor_msgs::JoyPtr& msg)
{
    m_msg_joy_input_dev=*msg;
    m_msg_joy_input_dev.header.stamp=ros::Time::now();
}
void WAIOATriggers::cb_tmr_trigger_joypad_feedback(const ros::TimerEvent& event)
{
    sensor_msgs::JoyFeedback msg_jfb_joy_input;
    msg_jfb_joy_input.type=1;
    msg_jfb_joy_input.id=0;
    msg_jfb_joy_input.intensity=0.0;
    m_msg_jfa_joy_input_dev_feedback.array.push_back(msg_jfb_joy_input);
    m_pub_jfa_joy_input_dev_feedback.publish(m_msg_jfa_joy_input_dev_feedback);
}
void WAIOATriggers::CheckTriggersJoypad()
{
    // If no joypad attached, or disconnected skip check
    if(!m_msg_joy_input_dev.buttons.empty()
        && (ros::Time::now()-m_msg_joy_input_dev.header.stamp).toSec()<(m_f_trigger_timeout/6.0))
    {
        /*
         * PS3 Legacy Support:
         * Left Crosshair buttons start from
         * [13]Up, [14]Down, [15]Left and [16]Right
        */
        if(m_msg_joy_input_dev.buttons.size()==17)
        {
            // Trigger SCENE PREV - OPTIONS + CURSOR LEFT
            if( (m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[15]==1) )
            {
                //TriggerJoypadSendFeedback();
                TriggeredJoypadOptionsCursorLeft();
            }
            // Trigger SCENE NEXT - OPTIONS + CURSOR RIGHT
            else if( (m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[16]==1) )
            {
                //TriggerJoypadSendFeedback();
                TriggeredJoypadOptionsCursorRight();
            }
            // Trigger CAMERA RVIZ DEFAULT - OPTIONS + CURSOR DOWN
            else if( (m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[14]==1) )
            {
                TriggeredJoypadOptionsCursorDown();
            }
            // Trigger CAMERA RVIZ CYCLE - OPTIONS + CURSOR UP
            else if( (m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[13]==1) )
            {
                TriggeredJoypadOptionsCursorUp();
            }
        }
        else // Use other PS Controllers (PS4 and PS5)
        {
            // Trigger SCENE PREV - OPTIONS + CURSOR LEFT
            if( (m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.axes[6]==1.0) )
            {
                //TriggerJoypadSendFeedback();
                TriggeredJoypadOptionsCursorLeft();
            }
            // Trigger SCENE NEXT - OPTIONS + CURSOR RIGHT
            else if( (m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.axes[6]==-1.0) )
            {
                //TriggerJoypadSendFeedback();
                TriggeredJoypadOptionsCursorRight();
            }
            // Trigger CAMERA RVIZ DEFAULT - OPTIONS + CURSOR DOWN
            else if( (m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.axes[7]==-1.0) )
            {
                TriggeredJoypadOptionsCursorDown();
            }
            // Trigger CAMERA RVIZ CYCLE - OPTIONS + CURSOR UP
            else if( (m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.axes[7]==1.0) )
            {
                TriggeredJoypadOptionsCursorUp();
            }
        }

        // Trigger LOG PART MINUS - LEFT JOYSTICK BUTTON + CIRCLE
        if(m_msg_joy_input_dev.buttons[11]==1
            && m_msg_joy_input_dev.buttons[1]==1)
        {
            TriggeredJoypadJSLeftButtonCircle();
        }
        // Trigger LOG PART TILDE - LEFT JOYSTICK BUTTON + CROSS
        else if(m_msg_joy_input_dev.buttons[11]==1
            && m_msg_joy_input_dev.buttons[0]==1)
        {
            TriggeredJoypadJSLeftButtonCross();
        }
        // Trigger LOG PART PLUS - LEFT JOYSTICK BUTTON + TRIANGLE
        else if(m_msg_joy_input_dev.buttons[11]==1
            && m_msg_joy_input_dev.buttons[2]==1)
        {
            TriggeredJoypadJSLeftButtonTriangle();
        }
        // Trigger LOG PART SCORE - LEFT JOYSTICK BUTTON + SQUARE
        else if(m_msg_joy_input_dev.buttons[11]==1
            && m_msg_joy_input_dev.buttons[3]==1)
        {
            TriggeredJoypadJSLeftButtonSquare();
        }
        // Trigger LOG EXAM SCORE - RIGHT JOYSTICK BUTTON + SQUARE
        else if(m_msg_joy_input_dev.buttons[12]==1
            && m_msg_joy_input_dev.buttons[3]==1)
        {
            TriggeredJoypadJSRightButtonSquare();
        }
        // Trigger BODY INTERACTION - LEFT JOYSTICK BUTTON + RIGHT JOYSTICK BUTTON
        else if(m_msg_joy_input_dev.buttons[11]==1
            && m_msg_joy_input_dev.buttons[12]==1)
        {
            TriggeredJoypadJSLeftButtonJSRightButton();
        }
        // Trigger EVAL GRAPH - TRIGGER L1 + TRIGGER R1
        else if(m_msg_joy_input_dev.buttons[4]==1
            && m_msg_joy_input_dev.buttons[5]==1)
        {
            TriggeredJoypadTriggerL1TriggerR1();
        }
        // Trigger RESPAWN 2D/3D - TRIGGER L2
        else if(m_msg_joy_input_dev.buttons[6]==1)
        {
            TriggeredJoypadTriggerL2();
        }
        // Trigger FORCE - TRIGGER R2
        else if(m_msg_joy_input_dev.buttons[7]==1)
        {
            TriggerJoypadSendFeedback();
            TriggeredJoypadTriggerR2();
        }
        // Trigger PRESENTER MODE - SHARE + OPTIONS
        else if(m_msg_joy_input_dev.buttons[8]==1
            && m_msg_joy_input_dev.buttons[9]==1)
        {
            TriggeredJoypadShareOptions();
        }
        // Trigger CAMERA RVIZ FOLLOW - SHARE + PS BUTTON
        else if(m_msg_joy_input_dev.buttons[8]==1
            && m_msg_joy_input_dev.buttons[10]==1)
        {
            TriggeredJoypadSharePSButton();
        }
        // Trigger AVATAR - SHARE + CROSS
        else if(m_msg_joy_input_dev.buttons[8]==1
            && m_msg_joy_input_dev.buttons[0]==1)
        {
            TriggeredJoypadShareCross();
        }
        // Trigger LECTERN - SHARE + TRIANGLE
        else if(m_msg_joy_input_dev.buttons[8]==1
            && m_msg_joy_input_dev.buttons[2]==1)
        {
            TriggeredJoypadShareTriangle();
        }
        // Trigger TABLE - SHARE + SQUARE
        else if(m_msg_joy_input_dev.buttons[8]==1
            && m_msg_joy_input_dev.buttons[3]==1)
        {
            TriggeredJoypadShareSquare();
        }
        // Trigger AUDIO - SHARE + CIRCLE
        else if(m_msg_joy_input_dev.buttons[8]==1
            && m_msg_joy_input_dev.buttons[1]==1)
        {
            TriggeredJoypadShareCircle();
        }
        // Trigger AVAILABLE
        else if(m_msg_joy_input_dev.buttons[8]==1
            && m_msg_joy_input_dev.buttons[11]==1)
        {
            TriggeredJoypadShareJSLeftButton();
        }
        // Trigger VOICE LISTEN - PS BUTTON + CIRCLE
        else if(m_msg_joy_input_dev.buttons[10]==1
            && m_msg_joy_input_dev.buttons[1]==1)
        {
            TriggeredJoypadPSButtonCircle();
        }
        // Trigger VOICE LISTEN - PS BUTTON + JS RIGHT BUTTON
        else if(m_msg_joy_input_dev.buttons[10]==1
            && m_msg_joy_input_dev.buttons[12]==1)
        {
            TriggeredJoypadPSButtonJSRightButton();
        }
        // Trigger VOICE LISTEN - PS BUTTON + JS LEFT BUTTON
        else if(m_msg_joy_input_dev.buttons[10]==1
            && m_msg_joy_input_dev.buttons[11]==1)
        {
            TriggeredJoypadPSButtonJSLeftButton();
        }
        // Trigger CAMERA RVIZ IDLE - PS BUTTON + CROSS
        else if(m_msg_joy_input_dev.buttons[10]==1
            && m_msg_joy_input_dev.buttons[0]==1)
        {
            TriggeredJoypadPSButtonCross();
        }
        // Trigger INTRODUCTION - PS BUTTON + TRIANGLE
        else if(m_msg_joy_input_dev.buttons[10]==1
            && m_msg_joy_input_dev.buttons[2]==1)
        {
            TriggeredJoypadPSButtonTriangle();
        }
        // Trigger CAMERA RVIZ ENFORCE - PS BUTTON + RECTANGLE
        else if(m_msg_joy_input_dev.buttons[10]==1
            && m_msg_joy_input_dev.buttons[3]==1)
        {
            TriggeredJoypadPSButtonRectangle();
        }
        // Trigger WIM interactions - Focus on and select AUDIENCE - SHARE + RIGHT JOYSTICK AXES/BUTTON
        else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.axes[4]<-0.5) TriggeredJoypadShareJSRightAxesDown();
        else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.axes[4]>0.5) TriggeredJoypadShareJSRightAxesUp();
        else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.axes[3]>0.5) TriggeredJoypadShareJSRightAxesLeft();
        else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.axes[3]<-0.5) TriggeredJoypadShareJSRightAxesRight();
        else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.buttons[12]==1) TriggeredJoypadShareJSRightButton();
        // Trigger WIM interactions - Focus on and select REPS - OPTIONS + LEFT JOYSTICK AXES/BUTTON and PS BUTTON
        else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.axes[1]<-0.5) TriggeredJoypadOptionsJLeftAxesDown();
        else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.axes[1]>0.5) TriggeredJoypadOptionsJLeftAxesUp();
        else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.axes[0]>0.5) TriggeredJoypadOptionsJLeftAxesLeft();
        else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.axes[0]<-0.5) TriggeredJoypadOptionsJLeftAxesRight();
        else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[11]==1) TriggeredJoypadOptionsJLeftButton();
        else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[10]==1) TriggeredJoypadOptionsPSButton();
        else if(m_msg_joy_input_dev.buttons[0]==1 ||
                m_msg_joy_input_dev.buttons[1]==1 ||
                m_msg_joy_input_dev.buttons[2]==1 ||
                m_msg_joy_input_dev.buttons[3]==1 ||
                m_msg_joy_input_dev.buttons[4]==1 ||
                m_msg_joy_input_dev.buttons[5]==1 ||
                m_msg_joy_input_dev.buttons[6]==1 ||
                m_msg_joy_input_dev.buttons[7]==1 ||
                m_msg_joy_input_dev.buttons[8]==1 ||
                m_msg_joy_input_dev.buttons[9]==1 ||
                m_msg_joy_input_dev.buttons[10]==1 ||
                m_msg_joy_input_dev.buttons[11]==1) // TRIGGER - Any button
        {
            TriggeredJoypadButtonsAny();
        }
        else
        {
            // Do nothing...
        }
    }
}
tf::Vector3 WAIOATriggers::GetJoypadAxesBodyInteraction()
{
    if(m_msg_joy_input_dev.axes.size()!=0)
    {
        return tf::Vector3(
                -5.0*m_msg_joy_input_dev.axes[3]*fabs(m_msg_joy_input_dev.axes[3]),
                -1.0*m_msg_joy_input_dev.axes[1]*fabs(m_msg_joy_input_dev.axes[1]),
                +5.0*m_msg_joy_input_dev.axes[4]*fabs(m_msg_joy_input_dev.axes[4]));
    }
    else
    {
        return tf::Vector3(0.0,0.0,0.0);
    }
}
void WAIOATriggers::TriggerJoypadSendFeedback(float f_duration,float f_intensity)
{
    sensor_msgs::JoyFeedback msg_jfb_joy_input;
    msg_jfb_joy_input.type=1;
    msg_jfb_joy_input.id=0;
    msg_jfb_joy_input.intensity=f_intensity;
    m_msg_jfa_joy_input_dev_feedback.array.push_back(msg_jfb_joy_input);
    m_pub_jfa_joy_input_dev_feedback.publish(m_msg_jfa_joy_input_dev_feedback);

    m_tmr_trigger_joypad_feedback.setPeriod(ros::Duration(f_duration),true);
    m_tmr_trigger_joypad_feedback.start();
}
void WAIOATriggers::TriggeredJoypadButtonsAny()
{
    //...->Activate();
}
void WAIOATriggers::TriggeredJoypadOptionsCursorLeft()
{
    m_s_oa_event_inpdev_last="[JOY] SCENE Previous";
    m_trg_scene_select_prev->Activate();
}
void WAIOATriggers::TriggeredJoypadOptionsCursorRight()
{
    m_s_oa_event_inpdev_last="[JOY] SCENE Next";
    m_trg_scene_select_next->Activate();
}
void WAIOATriggers::TriggeredJoypadOptionsCursorDown()
{
    m_s_oa_event_inpdev_last="[JOY] CAMERA Default";
    m_trg_camera_rviz_default->Activate();
}
void WAIOATriggers::TriggeredJoypadOptionsCursorUp()
{
    m_s_oa_event_inpdev_last="[JOY] CAMERA Cycle";
    m_trg_camera_rviz_cycle->Activate();
}
void WAIOATriggers::TriggeredJoypadPSButtonCross()
{
    m_s_oa_event_inpdev_last="[JOY] CAMERA Idle";
    m_trg_camera_rviz_idle->Activate();
}
void WAIOATriggers::TriggeredJoypadSharePSButton()
{
    m_s_oa_event_inpdev_last="[JOY] CAMERA Follow";
    m_trg_camera_rviz_follow->Activate();
}
void WAIOATriggers::TriggeredJoypadJSLeftButtonCircle()
{
    m_s_oa_event_inpdev_last="[JOY] PART Minus";
    m_trg_audience_eval_part_minus->Activate();
}
void WAIOATriggers::TriggeredJoypadJSLeftButtonCross()
{
    m_s_oa_event_inpdev_last="[JOY] PART Tilde";
    m_trg_audience_eval_part_tilde->Activate();
}
void WAIOATriggers::TriggeredJoypadJSLeftButtonTriangle()
{
    m_s_oa_event_inpdev_last="[JOY] PART Plus";
    m_trg_audience_eval_part_plus->Activate();
}
void WAIOATriggers::TriggeredJoypadJSLeftButtonSquare()
{
    m_s_oa_event_inpdev_last="[JOY] PART Score";
    m_trg_audience_eval_part_score->Activate();
}
void WAIOATriggers::TriggeredJoypadJSRightButtonSquare()
{
    m_s_oa_event_inpdev_last="[JOY] EXAM Score";
    m_trg_audience_eval_exam_score->Activate();
}
void WAIOATriggers::TriggeredJoypadJSLeftButtonJSRightButton()
{
    m_s_oa_event_inpdev_last="[JOY] BODY INT";
    m_trg_body_interaction->Activate();
}
void WAIOATriggers::TriggeredJoypadTriggerL1TriggerR1()
{
    m_s_oa_event_inpdev_last="[JOY] EVAL Graph";
    m_trg_audience_eval_graph->Activate();
}
void WAIOATriggers::TriggeredJoypadTriggerL2()
{
    m_s_oa_event_inpdev_last="[JOY] RESPAWN 2D/3D";
    m_trg_respawn_2d3d->Activate();
}
void WAIOATriggers::TriggeredJoypadTriggerR2()
{
    m_s_oa_event_inpdev_last="[JOY] FORCE";
    m_trg_force->Activate();
}
void WAIOATriggers::TriggeredJoypadShareOptions()
{
    m_s_oa_event_inpdev_last="[JOY] PRESENCE";
    m_trg_presence_mode->Activate();
}
void WAIOATriggers::TriggeredJoypadShareCross()
{
    m_s_oa_event_inpdev_last="[JOY] AVATAR";
    m_trg_avatar->Activate();
}
void WAIOATriggers::TriggeredJoypadShareTriangle()
{
    m_s_oa_event_inpdev_last="[JOY] LECTERN";
    m_trg_lectern->Activate();
}
void WAIOATriggers::TriggeredJoypadShareSquare()
{
    m_s_oa_event_inpdev_last="[JOY] TABLE";
    m_trg_table->Activate();
}
void WAIOATriggers::TriggeredJoypadShareCircle()
{
    m_s_oa_event_inpdev_last="[JOY] AUDIO";
    m_trg_audio->Activate();
}
void WAIOATriggers::TriggeredJoypadShareJSLeftButton()
{
    m_s_oa_event_inpdev_last="[JOY] AVAILABLE";
    // m_trg_
}
void WAIOATriggers::TriggeredJoypadPSButtonCircle()
{
    m_s_oa_event_inpdev_last="[JOY] VOICE Command";
    m_trg_voice_command->Activate();
}
void WAIOATriggers::TriggeredJoypadPSButtonJSRightButton()
{
    m_s_oa_event_inpdev_last="[JOY] AUD REQ Reject";
    m_trg_audience_request_reject->Activate();
}
void WAIOATriggers::TriggeredJoypadPSButtonJSLeftButton()
{
    m_s_oa_event_inpdev_last="[JOY] AUD REQ Accept";
    m_trg_audience_request_accept->Activate();
}
void WAIOATriggers::TriggeredJoypadShareJSRightAxesDown()
{
    m_s_oa_event_inpdev_last="[JOY] AUD FOC Down";
    m_trg_audience_focus_down->Activate();
}
void WAIOATriggers::TriggeredJoypadShareJSRightAxesUp()
{
    m_s_oa_event_inpdev_last="[JOY] AUD FOC Up";
    m_trg_audience_focus_up->Activate();
}
void WAIOATriggers::TriggeredJoypadShareJSRightAxesLeft()
{
    m_s_oa_event_inpdev_last="[JOY] AUD FOC Left";
    m_trg_audience_focus_left->Activate();
}
void WAIOATriggers::TriggeredJoypadShareJSRightAxesRight()
{
    m_s_oa_event_inpdev_last="[JOY] AUD FOC Right";
    m_trg_audience_focus_right->Activate();
}
void WAIOATriggers::TriggeredJoypadShareJSRightButton()
{
    m_s_oa_event_inpdev_last="[JOY] AUD Select";
    m_trg_audience_select->Activate();
}
void WAIOATriggers::TriggeredJoypadOptionsJLeftAxesDown()
{
    m_s_oa_event_inpdev_last="[JOY] REP DET Focus";
    m_trg_rep_detail_focus->Activate();
}
void WAIOATriggers::TriggeredJoypadOptionsJLeftAxesUp()
{
    m_s_oa_event_inpdev_last="[JOY] ...";
}
void WAIOATriggers::TriggeredJoypadOptionsJLeftAxesLeft()
{
    m_s_oa_event_inpdev_last="[JOY] ...";
}
void WAIOATriggers::TriggeredJoypadOptionsJLeftAxesRight()
{
    m_s_oa_event_inpdev_last="[JOY] ...";
}
void WAIOATriggers::TriggeredJoypadOptionsPSButton()
{
    m_s_oa_event_inpdev_last="[JOY] DET Rotation";
    m_trg_rep_detail_rot->Activate();
}
void WAIOATriggers::TriggeredJoypadOptionsJLeftButton()
{
    m_s_oa_event_inpdev_last="[JOY] REP DET Select";
    m_trg_rep_detail_select->Activate();
}
void WAIOATriggers::TriggeredJoypadPSButtonTriangle()
{
    m_s_oa_event_inpdev_last="[JOY] INTRODUCTION";
    m_trg_introduction->Activate();
}
void WAIOATriggers::TriggeredJoypadPSButtonRectangle()
{
    m_s_oa_event_inpdev_last="[JOY] CAMERA Enforce";
    m_trg_camera_rviz_enforce->Activate();
}



/////////////////////////////////////////////////
/// Callback to receive inputs from HTC VIVE
/////////////////////////////////////////////////
void WAIOATriggers::cb_m_sub_htc_vive_controller_left(const rviz_vive_plugin_msgs::ControllerPtr& msg)
{
    m_msg_htc_vive_input_dev_left=*msg;
    m_msg_htc_vive_input_dev_left.header.stamp=ros::Time::now();
}
void WAIOATriggers::cb_m_sub_htc_vive_controller_right(const rviz_vive_plugin_msgs::ControllerPtr& msg)
{
    m_msg_htc_vive_input_dev_right=*msg;
    m_msg_htc_vive_input_dev_right.header.stamp=ros::Time::now();
}
void WAIOATriggers::CheckTriggersHTCVive()
{
    float f_htc_vive_trackpad_threshold=0.25;

    // HTC VIVE - LEFT CONTROLLER
    if((ros::Time::now()-m_msg_htc_vive_input_dev_left.header.stamp).toSec()<(m_f_trigger_timeout/6.0))
    {
        // Modifier is GRIP BUTTON
        if(m_msg_htc_vive_input_dev_left.grip_pressed)
        {
            if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && m_msg_htc_vive_input_dev_left.trackpad_position.x<-1.0+f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD LEFT
                TriggeredHTCViveControllerLeftGripTrackpadLeft();
            }
            else if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && m_msg_htc_vive_input_dev_left.trackpad_position.x>1.0-f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD RIGHT
                TriggeredHTCViveControllerLeftGripTrackpadRight();
            }
            else if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && m_msg_htc_vive_input_dev_left.trackpad_position.y<-1.0+f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD DOWN
                TriggeredHTCViveControllerLeftGripTrackpadDown();
            }
            else if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && m_msg_htc_vive_input_dev_left.trackpad_position.y>1.0-f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD UP
                TriggeredHTCViveControllerLeftGripTrackpadUp();
            }
            else if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD MIDDLE
                TriggeredHTCViveControllerLeftGripTrackpadMiddle();
                TriggerHTCViveSendFeedbackLeft();
            }
            else if(m_msg_htc_vive_input_dev_left.menu_pressed)
            {
                // Pressed LEFT MENU
                TriggeredHTCViveControllerLeftGripMenu();
            }
            else if(m_msg_htc_vive_input_dev_left.trigger==1.0)
            {
                // Pressed LEFT TRIGGER
                TriggeredHTCViveControllerLeftGripTrigger();
            }
            else
            {
                // Do nothing...
            }
        }
        else
        {
            if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && m_msg_htc_vive_input_dev_left.trackpad_position.x<-1.0+f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD LEFT
                TriggeredHTCViveControllerLeftTrackpadLeft();
            }
            else if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && m_msg_htc_vive_input_dev_left.trackpad_position.x>1.0-f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD RIGHT
                TriggeredHTCViveControllerLeftTrackpadRight();
            }
            else if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && m_msg_htc_vive_input_dev_left.trackpad_position.y<-1.0+f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD DOWN
                TriggeredHTCViveControllerLeftTrackpadDown();
            }
            else if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && m_msg_htc_vive_input_dev_left.trackpad_position.y>1.0-f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD UP
                TriggeredHTCViveControllerLeftTrackpadUp();
            }
            else if(m_msg_htc_vive_input_dev_left.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_left.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD MIDDLE
                TriggeredHTCViveControllerLeftTrackpadMiddle();
                TriggerHTCViveSendFeedbackLeft();
            }
            else if(m_msg_htc_vive_input_dev_left.menu_pressed)
            {
                // Pressed LEFT MENU
                TriggeredHTCViveControllerLeftMenu();
            }
            else if(m_msg_htc_vive_input_dev_left.trigger==1.0)
            {
                // Pressed LEFT TRIGGER
                TriggeredHTCViveControllerLeftTrigger();
            }
            else
            {
                // Do nothing...
            }
        }
    }



    // HTC VIVE - RIGHT CONTROLLER
    if((ros::Time::now()-m_msg_htc_vive_input_dev_right.header.stamp).toSec()<(m_f_trigger_timeout/6.0))
    {
        // Modifier is GRIP BUTTON
        if(m_msg_htc_vive_input_dev_right.grip_pressed)
        {
            if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && m_msg_htc_vive_input_dev_right.trackpad_position.x<-1.0+f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD LEFT
                TriggeredHTCViveControllerRightGripTrackpadLeft();
            }
            else if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && m_msg_htc_vive_input_dev_right.trackpad_position.x>1.0-f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD RIGHT
                TriggeredHTCViveControllerRightGripTrackpadRight();
            }
            else if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && m_msg_htc_vive_input_dev_right.trackpad_position.y<-1.0+f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD DOWN
                TriggeredHTCViveControllerRightGripTrackpadDown();
            }
            else if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && m_msg_htc_vive_input_dev_right.trackpad_position.y>1.0-f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD UP
                TriggeredHTCViveControllerRightGripTrackpadUp();
            }
            else if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD MIDDLE
                TriggeredHTCViveControllerRightGripTrackpadMiddle();
                TriggerHTCViveSendFeedbackRight();
            }
            else if(m_msg_htc_vive_input_dev_right.menu_pressed)
            {
                // Pressed LEFT MENU
                TriggeredHTCViveControllerRightGripMenu();
            }
            else if(m_msg_htc_vive_input_dev_right.trigger==1.0)
            {
                // Pressed LEFT TRIGGER
                TriggeredHTCViveControllerRightGripTrigger();
            }
            else
            {
                // Do nothing...
            }
        }
        else
        {
            if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && m_msg_htc_vive_input_dev_right.trackpad_position.x<-1.0+f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD LEFT
                TriggeredHTCViveControllerRightTrackpadLeft();
            }
            else if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && m_msg_htc_vive_input_dev_right.trackpad_position.x>1.0-f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD RIGHT
                TriggeredHTCViveControllerRightTrackpadRight();
            }
            else if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && m_msg_htc_vive_input_dev_right.trackpad_position.y<-1.0+f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD DOWN
                TriggeredHTCViveControllerRightTrackpadDown();
            }
            else if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && m_msg_htc_vive_input_dev_right.trackpad_position.y>1.0-f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD UP
                TriggeredHTCViveControllerRightTrackpadUp();
            }
            else if(m_msg_htc_vive_input_dev_right.trackpad_pressed
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.x)<f_htc_vive_trackpad_threshold
                    && fabs(m_msg_htc_vive_input_dev_right.trackpad_position.y)<f_htc_vive_trackpad_threshold)
            {
                // Pressed TRACKPAD MIDDLE
                TriggeredHTCViveControllerRightTrackpadMiddle();
                TriggerHTCViveSendFeedbackRight();
            }
            else if(m_msg_htc_vive_input_dev_right.menu_pressed)
            {
                // Pressed LEFT MENU
                TriggeredHTCViveControllerRightMenu();
            }
            else if(m_msg_htc_vive_input_dev_right.trigger==1.0)
            {
                // Pressed LEFT TRIGGER
                TriggeredHTCViveControllerRightTrigger();
            }
            else
            {
                // Do nothing...
            }
        }
    }
}
void WAIOATriggers::TriggerHTCViveSendFeedbackLeft(float f_duration,float f_intensity)
{
    rviz_vive_plugin_msgs::ControllerVibration msg_vib_htc_vive_input;
    msg_vib_htc_vive_input.header.stamp=ros::Time::now();
    msg_vib_htc_vive_input.amplitude=f_intensity;
    msg_vib_htc_vive_input.durationSeconds=f_duration;
    m_pub_vib_htc_vive_input_dev_feedback_left.publish(msg_vib_htc_vive_input);
}
void WAIOATriggers::TriggerHTCViveSendFeedbackRight(float f_duration,float f_intensity)
{
    rviz_vive_plugin_msgs::ControllerVibration msg_vib_htc_vive_input;
    msg_vib_htc_vive_input.header.stamp=ros::Time::now();
    msg_vib_htc_vive_input.amplitude=f_intensity;
    msg_vib_htc_vive_input.durationSeconds=f_duration;
    m_pub_vib_htc_vive_input_dev_feedback_right.publish(msg_vib_htc_vive_input);
}

// HTC VIVE Controller LEFT - WITHOUT GRIP
void WAIOATriggers::TriggeredHTCViveControllerLeftTrackpadLeft()
{
    m_s_oa_event_inpdev_last="[HTC] AUD FOC Left";
    m_trg_audience_focus_left->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerLeftTrackpadRight()
{
    m_s_oa_event_inpdev_last="[HTC] AUD FOC Right";
    m_trg_audience_focus_right->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerLeftTrackpadDown()
{
    m_s_oa_event_inpdev_last="[HTC] AUD FOC Down";
    m_trg_audience_focus_down->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerLeftTrackpadUp()
{
    m_s_oa_event_inpdev_last="[HTC] AUD FOC Up";
    m_trg_audience_focus_up->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerLeftTrackpadMiddle()
{
    m_s_oa_event_inpdev_last="[HTC] AUD Select";
    m_trg_audience_select->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerLeftMenu()
{

}
void WAIOATriggers::TriggeredHTCViveControllerLeftTrigger()
{

}
// HTC VIVE Controller LEFT - WITH GRIP ENABLED
void WAIOATriggers::TriggeredHTCViveControllerLeftGripTrackpadMiddle()
{

}
void WAIOATriggers::TriggeredHTCViveControllerLeftGripTrackpadLeft()
{

}
void WAIOATriggers::TriggeredHTCViveControllerLeftGripTrackpadRight()
{

}
void WAIOATriggers::TriggeredHTCViveControllerLeftGripTrackpadDown()
{

}
void WAIOATriggers::TriggeredHTCViveControllerLeftGripTrackpadUp()
{

}
void WAIOATriggers::TriggeredHTCViveControllerLeftGripMenu()
{

}
void WAIOATriggers::TriggeredHTCViveControllerLeftGripTrigger()
{

}

// HTC VIVE Controller RIGHT - WITHOUT GRIP
void WAIOATriggers::TriggeredHTCViveControllerRightTrackpadLeft()
{
    m_s_oa_event_inpdev_last="[HTC] SCENE Previous";
    m_trg_scene_select_prev->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerRightTrackpadRight()
{
    m_s_oa_event_inpdev_last="[HTC] SCENE Next";
    m_trg_scene_select_next->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerRightTrackpadDown()
{
    m_s_oa_event_inpdev_last="[HTC] CAMERA Default";
    m_trg_camera_rviz_default->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerRightTrackpadUp()
{
    m_s_oa_event_inpdev_last="[HTC] CAMERA Cycle";
    m_trg_camera_rviz_cycle->Activate();
}
void WAIOATriggers::TriggeredHTCViveControllerRightTrackpadMiddle()
{

}
void WAIOATriggers::TriggeredHTCViveControllerRightMenu()
{

}
void WAIOATriggers::TriggeredHTCViveControllerRightTrigger()
{

}
// HTC VIVE Controller RIGHT - WITH GRIP ENABLED
void WAIOATriggers::TriggeredHTCViveControllerRightGripTrackpadMiddle()
{

}
void WAIOATriggers::TriggeredHTCViveControllerRightGripTrackpadLeft()
{

}
void WAIOATriggers::TriggeredHTCViveControllerRightGripTrackpadRight()
{

}
void WAIOATriggers::TriggeredHTCViveControllerRightGripTrackpadDown()
{

}
void WAIOATriggers::TriggeredHTCViveControllerRightGripTrackpadUp()
{

}
void WAIOATriggers::TriggeredHTCViveControllerRightGripMenu()
{

}
void WAIOATriggers::TriggeredHTCViveControllerRightGripTrigger()
{

}



/////////////////////////////////////////////////
/// Methods to receive inputs from MOBILE
/////////////////////////////////////////////////
void WAIOATriggers::cb_sub_bol_trg_mob_scene_select_prev(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_scene_select_prev=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_scene_select_next(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_scene_select_next=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_scene_select_number(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_scene_select_number=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_default(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_camera_rviz_default=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_cycle(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_camera_rviz_cycle=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_select(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_camera_rviz_select=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_enforce(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_camera_rviz_enforce=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_idle(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_camera_rviz_idle=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_camera_rviz_follow(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_camera_rviz_follow=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_introduction(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_introduction=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_presence_mode(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_presence_mode=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_body_interaction(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_body_interaction=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_avatar(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_avatar=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_lectern(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_lectern=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_table(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_table=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audio(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audio=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_voice_command(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_voice_command=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_voice_prompt(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_voice_prompt=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_graph(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_graph=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_metaphore_bowl(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_metaphore_bowl=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_metaphore_balance(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_metaphore_balance=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_metaphore_marvin(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_metaphore_marvin=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_request_reject(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_request_reject=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_request_accept(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_request_accept=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_focus_down(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_focus_down=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_focus_up(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_focus_up=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_focus_left(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_focus_left=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_focus_right(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_focus_right=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_select(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_select=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_part_minus(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_part_minus=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_part_tilde(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_part_tilde=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_part_plus(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_part_plus=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_insufficient(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_exam_insufficient=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_sufficient(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_exam_sufficient=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_satisfactory(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_exam_satisfactory=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_good(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_exam_good=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_audience_eval_exam_very_good(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_audience_eval_exam_very_good=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_rep_detail_focus(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_rep_detail_focus=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_rep_detail_rot(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_rep_detail_rot=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_rep_detail_select(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_rep_select=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_learning_mode_plen(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_learning_mode_plen=msg->data;
}
void WAIOATriggers::cb_sub_bol_trg_mob_learning_mode_coop(const std_msgs::BoolPtr& msg)
{
    b_trg_mob_learning_mode_coop=msg->data;
}
void WAIOATriggers::CheckTriggersMobile()
{
    if(b_trg_mob_scene_select_prev)
    {
        m_s_oa_event_inpdev_last="[MOB] SCENE Previous";
        m_trg_scene_select_prev->Activate();
    }
    else if(b_trg_mob_scene_select_next)
    {
        m_s_oa_event_inpdev_last="[MOB] SCENE Next";
        m_trg_scene_select_next->Activate();
    }
    else if(b_trg_mob_scene_select_number)
    {
        m_s_oa_event_inpdev_last="[MOB] SCENE Number";
        m_trg_scene_select_next->Activate();
    }
    else if(b_trg_mob_camera_rviz_default)
    {
        m_s_oa_event_inpdev_last="[MOB] CAMERA Default";
        m_trg_camera_rviz_default->Activate();
    }
    else if(b_trg_mob_camera_rviz_cycle)
    {
        m_s_oa_event_inpdev_last="[MOB] CAMERA Select";
        m_trg_camera_rviz_cycle->Activate();
    }
    else if(b_trg_mob_camera_rviz_select)
    {
        m_s_oa_event_inpdev_last="[MOB] CAMERA Cycle";
        m_trg_camera_rviz_select->Activate();
    }
    else if(b_trg_mob_camera_rviz_enforce)
    {
        m_s_oa_event_inpdev_last="[MOB] CAMERA Enforce";
        m_trg_camera_rviz_enforce->Activate();
    }
    else if(b_trg_mob_camera_rviz_idle)
    {
        m_s_oa_event_inpdev_last="[MOB] CAMERA Idle";
        m_trg_camera_rviz_idle->Activate();
    }
    else if(b_trg_mob_camera_rviz_follow)
    {
        m_s_oa_event_inpdev_last="[MOB] CAMERA Follow";
        m_trg_camera_rviz_follow->Activate();
    }
    else if(b_trg_mob_introduction)
    {
        m_s_oa_event_inpdev_last="[MOB] INTRO";
        m_trg_introduction->Activate();
    }
    else if(b_trg_mob_presence_mode)
    {
        m_s_oa_event_inpdev_last="[MOB] PRESENCE";
        m_trg_presence_mode->Activate();
    }
    else if(b_trg_mob_body_interaction)
    {
        m_s_oa_event_inpdev_last="[MOB] BODY INT";
        m_trg_body_interaction->Activate();
    }
    else if(b_trg_mob_avatar)
    {
        m_s_oa_event_inpdev_last="[MOB] AVATAR";
        m_trg_avatar->Activate();
    }
    else if(b_trg_mob_lectern)
    {
        m_s_oa_event_inpdev_last="[MOB] LECTERN";
        m_trg_lectern->Activate();
    }
    else if(b_trg_mob_table)
    {
        m_s_oa_event_inpdev_last="[MOB] TABLE";
        m_trg_table->Activate();
    }
    else if(b_trg_mob_audio)
    {
        m_s_oa_event_inpdev_last="[MOB] AUDIO";
        m_trg_audio->Activate();
    }
    else if(b_trg_mob_voice_command)
    {
        m_s_oa_event_inpdev_last="[MOB] VOICE Command";
        m_trg_voice_command->Activate();
    }
    else if(b_trg_mob_voice_prompt)
    {
        m_s_oa_event_inpdev_last="[MOB] VOICE Prompt";
        m_trg_voice_prompt->Activate();
    }
    else if(b_trg_mob_audience_eval_graph)
    {
        m_s_oa_event_inpdev_last="[MOB] EVAL Graph";
        m_trg_audience_eval_graph->Activate();
    }
    else if(b_trg_mob_audience_eval_metaphore_bowl)
    {
        m_s_oa_event_inpdev_last="[MOB] EVAL Bowl";
        m_trg_audience_eval_metaphore_bowl->Activate();
    }
    else if(b_trg_mob_audience_eval_metaphore_balance)
    {
        m_s_oa_event_inpdev_last="[MOB] EVAL Balance";
        m_trg_audience_eval_metaphore_balance->Activate();
    }
    else if(b_trg_mob_audience_eval_metaphore_marvin)
    {
        m_s_oa_event_inpdev_last="[MOB] EVAL Marvin";
        m_trg_audience_eval_metaphore_marvin->Activate();
    } 
    else if(b_trg_mob_audience_request_reject)
    {
        m_s_oa_event_inpdev_last="[MOB] AUD REQ Reject";
        m_trg_audience_request_reject->Activate();
    }
    else if(b_trg_mob_audience_request_accept)
    {
        m_s_oa_event_inpdev_last="[MOB] AUD REQ Accept";
        m_trg_audience_request_accept->Activate();
    }
    else if(b_trg_mob_audience_focus_down)
    {
        m_s_oa_event_inpdev_last="[MOB] AUD FOC Down";
        m_trg_audience_focus_down->Activate();
    }
    else if(b_trg_mob_audience_focus_up)
    {
        m_s_oa_event_inpdev_last="[MOB] AUD FOC Up";
        m_trg_audience_focus_up->Activate();
    }
    else if(b_trg_mob_audience_focus_left)
    {
        m_s_oa_event_inpdev_last="[MOB] AUD FOC Left";
        m_trg_audience_focus_left->Activate();
    }
    else if(b_trg_mob_audience_focus_right)
    {
        m_s_oa_event_inpdev_last="[MOB] AUD FOC Right";
        m_trg_audience_focus_right->Activate();
    }
    else if(b_trg_mob_audience_select)
    {
        m_s_oa_event_inpdev_last="[MOB] AUD Select";
        m_trg_audience_select->Activate();
    }
    else if(b_trg_mob_audience_eval_part_minus)
    {
        m_s_oa_event_inpdev_last="[MOB] PART Minus";
        m_trg_audience_eval_part_minus->Activate();
    }
    else if(b_trg_mob_audience_eval_part_tilde)
    {
        m_s_oa_event_inpdev_last="[MOB] PART Tilde";
        m_trg_audience_eval_part_tilde->Activate();
    }
    else if(b_trg_mob_audience_eval_part_plus)
    {
        m_s_oa_event_inpdev_last="[MOB] PART Plus";
        m_trg_audience_eval_part_plus->Activate();
    }
    else if(b_trg_mob_audience_eval_exam_insufficient)
    {
        m_s_oa_event_inpdev_last="[MOB] EXAM Unsatsifactory";
        m_trg_audience_eval_exam_insufficient->Activate();
    }
    else if(b_trg_mob_audience_eval_exam_sufficient)
    {
        m_s_oa_event_inpdev_last="[MOB] EXAM Sufficient";
        m_trg_audience_eval_exam_sufficient->Activate();
    }
    else if(b_trg_mob_audience_eval_exam_satisfactory)
    {
        m_s_oa_event_inpdev_last="[MOB] EXAM Satsifactory";
        m_trg_audience_eval_exam_satisfactory->Activate();
    }
    else if(b_trg_mob_audience_eval_exam_good)
    {
        m_s_oa_event_inpdev_last="[MOB] EXAM Good";
        m_trg_audience_eval_exam_good->Activate();
    }// NEW 3
    else if(b_trg_mob_audience_eval_exam_very_good)
    {
        m_s_oa_event_inpdev_last="[MOB] EXAM Very Good";
        m_trg_audience_eval_exam_very_good->Activate();
    }
    else if(b_trg_mob_rep_detail_focus)
    {
        m_s_oa_event_inpdev_last="[MOB] REP DET Focus";
        m_trg_rep_detail_focus->Activate();
    }
    else if(b_trg_mob_rep_detail_rot)
    {
        m_s_oa_event_inpdev_last="[MOB] DET Rot";
        m_trg_rep_detail_rot->Activate();
    }
    else if(b_trg_mob_rep_select)
    {
        m_s_oa_event_inpdev_last="[MOB] REP DET Select";
        m_trg_rep_detail_select->Activate();
    }
    else if(b_trg_mob_learning_mode_plen)
    {
        m_s_oa_event_inpdev_last="[MOB] MODE Plenum";
        m_trg_learning_mode_plen->Activate();
    }
    else if(b_trg_mob_learning_mode_coop)
    {
        m_s_oa_event_inpdev_last="[MOB] MODE Cooperative";
        m_trg_learning_mode_coop->Activate();
    }
    else
    {
        // Do nothing...
    }

    ResetBoolEventTriggersMobile();
}
void WAIOATriggers::ResetBoolEventTriggersMobile()
{
    // MOBILE DEVICE - Init buffered triggers, received via topics
    b_trg_mob_scene_select_prev=false;
    b_trg_mob_scene_select_next=false;
    b_trg_mob_scene_select_number=false;
    b_trg_mob_camera_rviz_default=false;
    b_trg_mob_camera_rviz_cycle=false;
    b_trg_mob_camera_rviz_select=false;
    b_trg_mob_camera_rviz_enforce=false;
    b_trg_mob_camera_rviz_idle=false;
    b_trg_mob_camera_rviz_follow=false;
    b_trg_mob_introduction=false;
    b_trg_mob_presence_mode=false;
    b_trg_mob_body_interaction=false;
    b_trg_mob_avatar=false;
    b_trg_mob_lectern=false;
    b_trg_mob_table=false;
    b_trg_mob_audio=false;
    b_trg_mob_voice_command=false;
    b_trg_mob_voice_prompt=false;
    b_trg_mob_audience_eval_graph=false;
    b_trg_mob_audience_eval_metaphore_bowl=false;
    b_trg_mob_audience_eval_metaphore_balance=false;
    b_trg_mob_audience_eval_metaphore_marvin=false;
    b_trg_mob_audience_request_reject=false;
    b_trg_mob_audience_request_accept=false;
    b_trg_mob_audience_focus_down=false;
    b_trg_mob_audience_focus_up=false;
    b_trg_mob_audience_focus_left=false;
    b_trg_mob_audience_focus_right=false;
    b_trg_mob_audience_select=false;
    b_trg_mob_audience_eval_part_minus=false;
    b_trg_mob_audience_eval_part_tilde=false;
    b_trg_mob_audience_eval_part_plus=false;
    b_trg_mob_audience_eval_exam_insufficient=false;
    b_trg_mob_audience_eval_exam_sufficient=false;
    b_trg_mob_audience_eval_exam_satisfactory=false;
    b_trg_mob_audience_eval_exam_good=false;
    b_trg_mob_audience_eval_exam_very_good=false;
    b_trg_mob_rep_detail_focus=false;
    b_trg_mob_rep_detail_rot=false;
    b_trg_mob_rep_select=false;
    b_trg_mob_learning_mode_plen=false;
    b_trg_mob_learning_mode_coop=false;
}



/////////////////////////////////////////////////
/// Methods to receive inputs from VOICE
/////////////////////////////////////////////////
void WAIOATriggers::cb_sub_pic_intent_action_result(const picovoice_msgs::GetIntentActionResultConstPtr& msg)
{
    msg_pic_intent_action_result=*msg;
    if(msg_pic_intent_action_result.result.is_understood==1)
    {
        // Intel $politeness:politeness trigger (the) $trigger:trigger
        if(msg_pic_intent_action_result.result.intent.compare("IntelTrigger")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0
                || msg_pic_intent_action_result.result.picoslots[0].value.compare("please please please")==0)
            {
                if(msg_pic_intent_action_result.result.picoslots[1].value.compare("scene previous")==0)
                {
                    b_voi_scene_prev=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("scene next")==0)
                {
                    b_voi_scene_next=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("camera default")==0)
                {
                    b_voi_camera_rviz_default=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("camera cycle")==0)
                {
                    b_voi_camera_rviz_cycle=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("camera enforce")==0)
                {
                    b_voi_camera_rviz_enforce=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("camera idle")==0)
                {
                    b_voi_camera_rviz_idle=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("camera follow")==0)
                {
                    b_voi_camera_rviz_follow=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("introduction")==0)
                {
                    b_voi_camera_rviz_cycle=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("presence mode")==0)
                {
                    b_voi_presence_mode=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("body interaction")==0)
                {
                    b_voi_body_interaction=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("avatar")==0)
                {
                    b_voi_avatar=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("lectern")==0)
                {
                    b_voi_lectern=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("table")==0)
                {
                    b_voi_table=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audio")==0)
                {
                    b_voi_audio=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("graph evaluation view")==0)
                {
                    b_voi_audience_eval_graph=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("evaluation bowl")==0)
                {
                    b_voi_audience_eval_metaphorebowl=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("evaluation weight balance")==0)
                {
                    b_voi_audience_eval_metaphorebalance=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("evaluation marvin")==0)
                {
                    b_voi_audience_eval_metaphoremarvin=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience request reject")==0)
                {
                    b_voi_audience_request_reject=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience request accept")==0)
                {
                    b_voi_audience_request_accept=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience down")==0)
                {
                    b_voi_audience_focus_down=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience up")==0)
                {
                    b_voi_audience_focus_up=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience left")==0)
                {
                    b_voi_audience_focus_left=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience right")==0)
                {
                    b_voi_audience_focus_right=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience select")==0)
                {
                    b_voi_wim_audience_select=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience participation minus")==0)
                {
                    b_voi_audience_eval_part_minus=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience participation tilde")==0)
                {
                    b_voi_audience_eval_part_tilde=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience participation plus")==0)
                {
                    b_voi_audience_eval_part_plus=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience examination unsatisfactory")==0)
                {
                    b_voi_audience_eval_exam_insufficient=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience examination adequate")==0)
                {
                    b_voi_audience_eval_exam_sufficient=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience examination satisfactory")==0)
                {
                    b_voi_audience_eval_exam_satisfactory=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience examination good")==0)
                {
                    b_voi_audience_eval_exam_good=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("audience examination very good")==0)
                {
                    b_voi_audience_eval_exam_very_good=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("representative focus")==0)
                {
                    b_voi_rep_detail_focus=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("detail rotation")==0)
                {
                    b_voi_rep_detail_rot=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("representative select")==0)
                {
                    b_voi_rep_detail_select=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("learning mode plenum")==0)
                {
                    b_voi_learning_mode_plen=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("learning mode cooperative")==0)
                {
                    b_voi_learning_mode_coop=true;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("marvin")==0)
                {
                    m_wai_open_auditorium->PlaySound("IntelErrorMarvin");
                }
                else
                {
                    ROS_WARN("[INTEL] Voice: Unrecognized slot(s)!");
                    m_wai_open_auditorium->PlaySound("IntelError");
                }
            }
            else
            {
                ROS_WARN("[INTEL] Voice: You were not polite enough!");
                m_wai_open_auditorium->PlaySound("IntelErrorPoliteness");
            }
        }
        else if(msg_pic_intent_action_result.result.intent.compare("IntelShutdownRep")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0
                || msg_pic_intent_action_result.result.picoslots[0].value.compare("please please please")==0)
            {
                if(msg_pic_intent_action_result.result.picoslots[1].value.compare("workspace")==0)
                {
                    m_wai_open_auditorium->PlaySound("IntelShuttingDownWorkspace");
                    ros::spinOnce();
                    ros::Rate(0.15).sleep();
                    ros::shutdown();
                    int i_return_val=system("rosnode kill -a");
                    exit(0);
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("marvin")==0)
                {
                    m_wai_open_auditorium->ShutdownMarvin();
                }
                else
                {
                    ROS_WARN("[INTEL] Voice: Unrecognized slot(s)!");
                    m_wai_open_auditorium->PlaySound("IntelError");
                }
            }
            else
            {
                ROS_WARN("[INTEL] Voice: You were not polite enough!");
                m_wai_open_auditorium->PlaySound("IntelErrorPoliteness");
            }
        }
        else if(msg_pic_intent_action_result.result.intent.compare("IntelEngageRep")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0
                || msg_pic_intent_action_result.result.picoslots[0].value.compare("please please please")==0)
            {
                if(msg_pic_intent_action_result.result.picoslots[1].value.compare("marvin")==0)
                {
                    m_wai_open_auditorium->EngageMarvin();
                }
                else
                {
                    ROS_WARN("[INTEL] Voice: Unrecognized slot(s)!");
                    m_wai_open_auditorium->PlaySound("IntelError");
                }
            }
            else
            {
                ROS_WARN("[INTEL] Voice: You were not polite enough!");
                m_wai_open_auditorium->PlaySound("IntelErrorPoliteness");
            }
        }
        else if(msg_pic_intent_action_result.result.intent.compare("IntelTellAboutYourself")==0)
        {
            m_wai_open_auditorium->PlaySound("IntelTellAboutYourself");
        }
        else if(msg_pic_intent_action_result.result.intent.compare("IntelSelectRep")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0
                || msg_pic_intent_action_result.result.picoslots[0].value.compare("please please please")==0)
            {
                m_wai_open_auditorium->UpdateRepAndDetailSelected(false,msg_pic_intent_action_result.result.picoslots[1].value,"link_base");
                //m_trg_rep_detail_select->Activate();
                b_trg_mob_rep_select=true;
            }
            else
            {
                ROS_WARN("[INTEL] Voice: You were not polite enough!");
                m_wai_open_auditorium->PlaySound("IntelErrorPoliteness");
            }
        }
        else if(msg_pic_intent_action_result.result.intent.compare("IntelSelectAudience")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0
                || msg_pic_intent_action_result.result.picoslots[0].value.compare("please please please")==0)
            {
                // Currently max audience of 24 (ID0-ID23) is considered in OA:
                int i_id=0;
                if(msg_pic_intent_action_result.result.picoslots[1].value.compare("zero")==0) i_id=0;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("one")==0) i_id=1;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("two")==0) i_id=2;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("three")==0) i_id=3;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("four")==0) i_id=4;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("five")==0) i_id=5;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("six")==0) i_id=6;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("seven")==0) i_id=7;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("eight")==0) i_id=8;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("nine")==0) i_id=9;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("ten")==0) i_id=10;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("eleven")==0) i_id=11;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("twelve")==0) i_id=12;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("thirteen")==0) i_id=13;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("fourteen")==0) i_id=14;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("fifteen")==0) i_id=15;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("sixteen")==0) i_id=16;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("seventeen")==0) i_id=17;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("eighteen")==0) i_id=18;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("nineteen")==0) i_id=19;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("twenty")==0) i_id=20;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("twentyone")==0) i_id=21;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("twentytwo")==0) i_id=22;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("twentythree")==0) i_id=23;
                else
                {
                    ROS_WARN("[INTEL] Voice: Unrecognized slot(s)!");
                    m_wai_open_auditorium->PlaySound("IntelError");
                    return;
                }
                m_wai_open_auditorium->UpdateAudienceIDSelected(i_id);
                m_trg_audience_select->Activate();
            }
            else
            {
                ROS_WARN("[INTEL] Voice: You were not polite enough!");
                m_wai_open_auditorium->PlaySound("IntelErrorPoliteness");
            }
        }
        else
        {
            ROS_WARN("[INTEL] Voice: Understood, but inproper intent!");
        }
    }
    else
    {
        ROS_WARN("[INTEL] Voice: Did not understand request!");
    }
}
void WAIOATriggers::CheckTriggersVoice()
{
    if(b_voi_scene_prev)
    {
        m_s_oa_event_inpdev_last="[VOI] SCENE Previous";
        m_trg_scene_select_prev->Activate();
    }
    else if(b_voi_scene_next)
    {
        m_s_oa_event_inpdev_last="[VOI] SCENE Next";
        m_trg_scene_select_next->Activate();
    }
    else if(b_voi_camera_rviz_default)
    {
        m_s_oa_event_inpdev_last="[VOI] CAMERA Default";
        m_trg_camera_rviz_default->Activate();
    }
    else if(b_voi_camera_rviz_cycle)
    {
        m_s_oa_event_inpdev_last="[VOI] CAMERA Cycle";
        m_trg_camera_rviz_cycle->Activate();
    }
    else if(b_voi_camera_rviz_enforce)
    {
        m_s_oa_event_inpdev_last="[VOI] CAMERA Enforce";
        m_trg_camera_rviz_enforce->Activate();
    }
    else if(b_voi_camera_rviz_idle)
    {
        m_s_oa_event_inpdev_last="[VOI] CAMERA Idle";
        m_trg_camera_rviz_idle->Activate();
    }
    else if(b_voi_camera_rviz_follow)
    {
        m_s_oa_event_inpdev_last="[VOI] CAMERA Follow";
        m_trg_camera_rviz_follow->Activate();
    }
    else if(b_voi_introduction)
    {
        m_s_oa_event_inpdev_last="[VOI] INTRO";
        m_trg_introduction->Activate();
    }
    else if(b_voi_presence_mode)
    {
        m_s_oa_event_inpdev_last="[VOI] PRESENCE";
        m_trg_presence_mode->Activate();
    }
    else if(b_voi_body_interaction)
    {
        m_s_oa_event_inpdev_last="[VOI] BODY INT";
        m_trg_body_interaction->Activate();
    }
    else if(b_voi_avatar)
    {
        m_s_oa_event_inpdev_last="[VOI] AVATAR";
        m_trg_avatar->Activate();
    }
    else if(b_voi_lectern)
    {
        m_s_oa_event_inpdev_last="[VOI] LECTERN";
        m_trg_lectern->Activate();
    }
    else if(b_voi_table)
    {
        m_s_oa_event_inpdev_last="[VOI] TABLE";
        m_trg_table->Activate();
    }
    else if(b_voi_audio)
    {
        m_s_oa_event_inpdev_last="[VOI] AUDIO";
        m_trg_audio->Activate();
    }
    else if(b_voi_audience_eval_graph)
    {
        m_s_oa_event_inpdev_last="[VOI] EVAL Graph";
        m_trg_audience_eval_graph->Activate();
    }
    else if(b_voi_audience_eval_metaphorebowl)
    {
        m_s_oa_event_inpdev_last="[VOI] EVAL Bowl";
        m_trg_audience_eval_metaphore_bowl->Activate();
    }
    else if(b_voi_audience_eval_metaphorebalance)
    {
        m_s_oa_event_inpdev_last="[VOI] EVAL Balance";
        m_trg_audience_eval_metaphore_balance->Activate();
    }
    else if(b_voi_audience_eval_metaphoremarvin)
    {
        m_s_oa_event_inpdev_last="[VOI] EVAL Marvin";
        m_trg_audience_eval_metaphore_marvin->Activate();
    }
    else if(b_voi_audience_request_reject)
    {
        m_s_oa_event_inpdev_last="[VOI] AUD REQ Reject";
        m_trg_audience_request_reject->Activate();
    }
    else if(b_voi_audience_request_accept)
    {
        m_s_oa_event_inpdev_last="[VOI] AUD REQ Accept";
        m_trg_audience_request_accept->Activate();
    }
    else if(b_voi_audience_focus_down)
    {
        m_s_oa_event_inpdev_last="[VOI] AUD FOC Down";
        m_trg_audience_focus_down->Activate();
    }
    else if(b_voi_audience_focus_up)
    {
        m_s_oa_event_inpdev_last="[VOI] AUD FOC Up";
        m_trg_audience_focus_up->Activate();
    }
    else if(b_voi_audience_focus_left)
    {
        m_s_oa_event_inpdev_last="[VOI] AUD FOC Left";
        m_trg_audience_focus_left->Activate();
    }
    else if(b_voi_audience_focus_right)
    {
        m_s_oa_event_inpdev_last="[VOI] AUD FOC Right";
        m_trg_audience_focus_right->Activate();
    }
    else if(b_voi_wim_audience_select)
    {
        m_s_oa_event_inpdev_last="[VOI] AUD Select";
        m_trg_audience_select->Activate();
    }
    else if(b_voi_audience_eval_part_minus)
    {
        m_s_oa_event_inpdev_last="[VOI] PART Minus";
        m_trg_audience_eval_part_minus->Activate();
    }
    else if(b_voi_audience_eval_part_tilde)
    {
        m_s_oa_event_inpdev_last="[VOI] PART Tilde";
        m_trg_audience_eval_part_tilde->Activate();
    }
    else if(b_voi_audience_eval_part_plus)
    {
        m_s_oa_event_inpdev_last="[VOI] PART Plus";
        m_trg_audience_eval_part_plus->Activate();
    }
    else if(b_voi_audience_eval_exam_insufficient)
    {
        m_s_oa_event_inpdev_last="[VOI] EXAM Unsatisfactory";
        m_trg_audience_eval_exam_insufficient->Activate();
    }
    else if(b_voi_audience_eval_exam_sufficient)
    {
        m_s_oa_event_inpdev_last="[VOI] EXAM Sufficient";
        m_trg_audience_eval_exam_sufficient->Activate();
    }
    else if(b_voi_audience_eval_exam_satisfactory)
    {
        m_s_oa_event_inpdev_last="[VOI] EXAM Satisfactory";
        m_trg_audience_eval_exam_satisfactory->Activate();
    }
    else if(b_voi_audience_eval_exam_good)
    {
        m_s_oa_event_inpdev_last="[VOI] EXAM Good";
        m_trg_audience_eval_exam_good->Activate();
    }
    else if(b_voi_audience_eval_exam_very_good)
    {
        m_s_oa_event_inpdev_last="[VOI] EXAM Very Good";
        m_trg_audience_eval_exam_very_good->Activate();
    }
    else if(b_voi_rep_detail_focus)
    {
        m_s_oa_event_inpdev_last="[VOI] REP DET Focus";
        m_trg_rep_detail_focus->Activate();
    }
    else if(b_voi_rep_detail_rot)
    {
        m_s_oa_event_inpdev_last="[VOI] REP DET Rot";
        m_trg_rep_detail_rot->Activate();
    }
    else if(b_voi_rep_detail_select)
    {
        m_s_oa_event_inpdev_last="[VOI] REP DET Select";
        m_trg_rep_detail_select->Activate();
    }
    else if(b_voi_learning_mode_plen)
    {
        m_s_oa_event_inpdev_last="[VOI] MODE Plenum";
        m_trg_learning_mode_plen->Activate();
    }
    else if(b_voi_learning_mode_coop)
    {
        m_s_oa_event_inpdev_last="[VOI] MODE Cooperative";
        m_trg_learning_mode_coop->Activate();
    }
    else
    {
        // Do nothing...
    }

    ResetBoolEventTriggersVoice();
}
void WAIOATriggers::ResetBoolEventTriggersVoice()
{
    // VOICE INPUT - Init buffered triggers, received via topics
    b_voi_scene_prev=false;
    b_voi_scene_next=false;
    b_voi_camera_rviz_default=false;
    b_voi_camera_rviz_cycle=false;
    b_voi_camera_rviz_enforce=false;
    b_voi_camera_rviz_idle=false;
    b_voi_camera_rviz_follow=false;
    b_voi_introduction=false;
    b_voi_presence_mode=false;
    b_voi_body_interaction=false;
    b_voi_avatar=false;
    b_voi_lectern=false;
    b_voi_table=false;
    b_voi_audience_eval_graph=false;
    b_voi_audience_eval_metaphorebowl=false;
    b_voi_audience_eval_metaphorebalance=false;
    b_voi_audience_eval_metaphoremarvin=false;
    b_voi_audience_request_reject=false;
    b_voi_audience_request_accept=false;
    b_voi_audience_focus_down=false;
    b_voi_audience_focus_up=false;
    b_voi_audience_focus_left=false;
    b_voi_audience_focus_right=false;
    b_voi_wim_audience_select=false;
    b_voi_audience_eval_part_minus=false;
    b_voi_audience_eval_part_tilde=false;
    b_voi_audience_eval_part_plus=false;
    b_voi_audience_eval_exam_insufficient=false;
    b_voi_audience_eval_exam_sufficient=false;
    b_voi_audience_eval_exam_satisfactory=false;
    b_voi_audience_eval_exam_good=false;
    b_voi_audience_eval_exam_very_good=false;
    b_voi_rep_detail_focus=false;
    b_voi_rep_detail_rot=false;
    b_voi_rep_detail_select=false;
    b_voi_learning_mode_plen=false;
    b_voi_learning_mode_coop=false;
}



void WAIOATriggers::CheckTriggersBodyInteraction()
{
    if(m_wai_open_auditorium->CheckCollisionTriggerScenePrev())
    {
        m_s_oa_event_inpdev_last="[BOI] SCENE Previous";
        m_trg_scene_select_prev->Activate();
    }
    if(m_wai_open_auditorium->CheckCollisionTriggerSceneNext())
    {
        m_s_oa_event_inpdev_last="[BOI] SCENE Next";
        m_trg_scene_select_next->Activate();
    }
}



/////////////////////////////////////////////////
/// Methods to receive EXTERNAL EVENTS
/////////////////////////////////////////////////
void WAIOATriggers::cb_sub_hea_audience_request(const std_msgs::HeaderPtr& msg)
{
    m_msg_hea_audience_request=*msg;
    CheckTriggersEvents();
}
void WAIOATriggers::CheckTriggersEvents()
{
    m_tim_last_audience_request_received=ros::Time::now();
    m_wai_open_auditorium->SetAudienceRequestIncoming(m_msg_hea_audience_request);
    m_trg_audience_request_incoming->Activate();
}
