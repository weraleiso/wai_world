#include<wai_oa_state.h>



/////////////////////////////////////////////////
/// Implementation of state machine
/////////////////////////////////////////////////
void WAIOpenAuditorium::StateTransitionTo(WAIOAState* state,bool b_invoke_all_requests)
{
    ROS_INFO_STREAM("State Machine: Transition to \"" << typeid(*state).name() << "\" (Invoke all: " << b_invoke_all_requests << ")");

    if(this->wai_oa_state!=NULL)
    {
        delete this->wai_oa_state;
    }
    this->wai_oa_state=state;
    this->wai_oa_state->SetContext(this);

    if(b_invoke_all_requests)
    {
        this->RequestSetupModel();
        this->RequestSetupView();
        this->RequestLeaveState();
    }

    // Update current state label marker, cleanup strings
    wai_oa_session_manager.SetOAStatusLabel(
                "STA: "+
                GetStateNameCurrent()+
                ", LST TRG: "+
                GetEventNameInpDevLast()+
                ", CAM FRM: "+
                GetCameraFrameNames()+
                ", AUD SEL: "+
                std::to_string(i_audience_id_selected)+
                ", REP SEL: ["+
                GetRepSelectedClean()+
                "|"+
                GetRepDetailSelectedClean()+"]" );
}

// Common requests of context amongst all(!) states
void WAIOpenAuditorium::RequestSetupModel()
{
    if(GetSessionStartupFinished())
    {
        PlaySound(GetStateNameCurrent());
        PlaySound(GetStateNameCurrentTTS());
    }
    this->wai_oa_state->HandleSetupModel();
}
void WAIOpenAuditorium::RequestSetupView()
{
    this->wai_oa_state->HandleSetupView();
}
void WAIOpenAuditorium::RequestLeaveState()
{
    this->wai_oa_state->HandleLeaveState();
}



// STATE: StatePlenumPresenting
void StatePlenumPresenting::HandleSetupModel()
{
}
void StatePlenumPresenting::HandleSetupView()
{
}
void StatePlenumPresenting::HandleLeaveState()
{
}

// STATE: StatePlenumSettingUpScenePrev
void StatePlenumSettingUpScenePrev::HandleSetupModel()
{
    wai_oa_context->SetupScenePrevModel();
}
void StatePlenumSettingUpScenePrev::HandleSetupView()
{
    wai_oa_context->SetupScenePrevView();

    wai_oa_context->SetupProjectorStateFromScript();
    wai_oa_context->SetupProjectorFromScript();
    wai_oa_context->SetupLightFromScript();
    wai_oa_context->SetupCameraRvizFromScript();
    wai_oa_context->SetupTeleprompterFromScript();
    wai_oa_context->SetupBrowserLinkFromScript();
    wai_oa_context->SetupSoundFromScript();
    wai_oa_context->SetupLecternFromScript();
    wai_oa_context->SetupTableFromScript();
    wai_oa_context->SetupVirtualPresenterFromScript();
    wai_oa_context->SetupGraph3DFromScript();
    wai_oa_context->SetupGraphStatsFromScript();
    wai_oa_context->PrepareRespawn2D3DFromScript();
    wai_oa_context->SetupHandTextFromScript();
    wai_oa_context->SetupRepresentativeFromScript();
    wai_oa_context->SetupMarvinAsPresenterFromScript();
    wai_oa_context->SetupMarvinHologramFromScript();
    // wai_oa_context->SetupLearningModeFromScript();
}
void StatePlenumSettingUpScenePrev::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpSceneNext
void StatePlenumSettingUpSceneNext::HandleSetupModel()
{
    wai_oa_context->SetupSceneNextModel();
}
void StatePlenumSettingUpSceneNext::HandleSetupView()
{
    wai_oa_context->SetupSceneNextView();

    wai_oa_context->SetupProjectorStateFromScript();
    wai_oa_context->SetupProjectorFromScript();
    wai_oa_context->SetupLightFromScript();
    wai_oa_context->SetupCameraRvizFromScript();
    wai_oa_context->SetupTeleprompterFromScript();
    wai_oa_context->SetupBrowserLinkFromScript();
    wai_oa_context->SetupSoundFromScript();
    wai_oa_context->SetupLecternFromScript();
    wai_oa_context->SetupTableFromScript();
    wai_oa_context->SetupVirtualPresenterFromScript();
    wai_oa_context->SetupGraph3DFromScript();
    wai_oa_context->SetupGraphStatsFromScript();
    wai_oa_context->PrepareRespawn2D3DFromScript();
    wai_oa_context->SetupHandTextFromScript();
    wai_oa_context->SetupRepresentativeFromScript();
    wai_oa_context->SetupMarvinAsPresenterFromScript();
    wai_oa_context->SetupMarvinHologramFromScript();
    // wai_oa_context->SetupLearningModeFromScript();
}
void StatePlenumSettingUpSceneNext::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpSceneSelect
void StatePlenumSettingUpSceneSelect::HandleSetupModel()
{
    // First, prepare selected scene (-1) and then invoke "Scene Next"
    wai_oa_context->SetupSceneSelectModel();
}
void StatePlenumSettingUpSceneSelect::HandleSetupView()
{
    wai_oa_context->SetupSceneSelectView();
}
void StatePlenumSettingUpSceneSelect::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpSceneNext,true);
}


// STATE: StatePlenumSettingUpCameraRvizDefault
void StatePlenumSettingUpCameraRvizDefault::HandleSetupModel()
{
    wai_oa_context->SetupCameraRvizDefaultModel();
}
void StatePlenumSettingUpCameraRvizDefault::HandleSetupView()
{
    wai_oa_context->SetupCameraRvizDefaultView();
}
void StatePlenumSettingUpCameraRvizDefault::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpCameraRvizCycle
void StatePlenumSettingUpCameraRvizCycle::HandleSetupModel()
{
    wai_oa_context->SetupCameraRvizCycleModel();
}
void StatePlenumSettingUpCameraRvizCycle::HandleSetupView()
{
    wai_oa_context->SetupCameraRvizCycleView();
}
void StatePlenumSettingUpCameraRvizCycle::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpCameraRvizSelect
void StatePlenumSettingUpCameraRvizSelect::HandleSetupModel()
{
    wai_oa_context->SetupCameraRvizSelectModel();
}
void StatePlenumSettingUpCameraRvizSelect::HandleSetupView()
{
    wai_oa_context->SetupCameraRvizSelectView();
}
void StatePlenumSettingUpCameraRvizSelect::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpCameraRvizEnforce
void StatePlenumSettingUpCameraRvizEnforce::HandleSetupModel()
{
    wai_oa_context->SetupCameraRvizEnforceModel();
}
void StatePlenumSettingUpCameraRvizEnforce::HandleSetupView()
{
    wai_oa_context->SetupCameraRvizEnforceView();
}
void StatePlenumSettingUpCameraRvizEnforce::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpCameraRvizFollow
void StatePlenumSettingUpCameraRvizFollow::HandleSetupModel()
{
    wai_oa_context->SetupCameraRvizFollowModel();
}
void StatePlenumSettingUpCameraRvizFollow::HandleSetupView()
{
    wai_oa_context->SetupCameraRvizFollowView();
}
void StatePlenumSettingUpCameraRvizFollow::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpCameraRvizIdle
void StatePlenumSettingUpCameraRvizIdle::HandleSetupModel()
{
    wai_oa_context->SetupCameraRvizIdleModel();
}
void StatePlenumSettingUpCameraRvizIdle::HandleSetupView()
{
    wai_oa_context->SetupCameraRvizIdleView();
}
void StatePlenumSettingUpCameraRvizIdle::HandleLeaveState()
{
    wai_oa_context->SetupCameraRvizIdleLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpPresenceMode
void StatePlenumSettingUpPresenceMode::HandleSetupModel()
{
    wai_oa_context->SetupPresenceModeModel();
}
void StatePlenumSettingUpPresenceMode::HandleSetupView()
{
    wai_oa_context->SetupPresenceModeView();
}
void StatePlenumSettingUpPresenceMode::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpBodyInteraction
void StatePlenumSettingUpBodyInteraction::HandleSetupModel()
{
    wai_oa_context->SetupBodyInteractionModel();
}
void StatePlenumSettingUpBodyInteraction::HandleSetupView()
{
    wai_oa_context->SetupBodyInteractionView();
}
void StatePlenumSettingUpBodyInteraction::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpAvatar
void StatePlenumSettingUpAvatar::HandleSetupModel()
{
    wai_oa_context->SetupAvatarModel();
}
void StatePlenumSettingUpAvatar::HandleSetupView()
{
    wai_oa_context->SetupAvatarView();
}
void StatePlenumSettingUpAvatar::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpLectern
void StatePlenumSettingUpLectern::HandleSetupModel()
{
    wai_oa_context->SetupLecternModel();
}
void StatePlenumSettingUpLectern::HandleSetupView()
{
    wai_oa_context->SetupLecternView();
}
void StatePlenumSettingUpLectern::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpTable
void StatePlenumSettingUpTable::HandleSetupModel()
{
    wai_oa_context->SetupTableModel();
}
void StatePlenumSettingUpTable::HandleSetupView()
{
    wai_oa_context->SetupTableView();
}
void StatePlenumSettingUpTable::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpAudio
void StatePlenumSettingUpAudio::HandleSetupModel()
{
    wai_oa_context->SetupAudioModel();
}
void StatePlenumSettingUpAudio::HandleSetupView()
{
    wai_oa_context->SetupAudioView();
}
void StatePlenumSettingUpAudio::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpSketch
void StatePlenumSettingUpSketch::HandleSetupModel()
{
    wai_oa_context->SetupSketchModel();
}
void StatePlenumSettingUpSketch::HandleSetupView()
{
    wai_oa_context->SetupSketchView();
}
void StatePlenumSettingUpSketch::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StateLearningModePlenumPrepare
void StateLearningModePlenumPrepare::HandleSetupModel()
{
    wai_oa_context->SetupLearningModePlenumModel();
}
void StateLearningModePlenumPrepare::HandleSetupView()
{
    wai_oa_context->SetupLearningModePlenumView();
}
void StateLearningModePlenumPrepare::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StateLearningModeCooperativePrepare
void StateLearningModeCooperativePrepare::HandleSetupModel()
{
    wai_oa_context->SetupLearningModeCooperativeModel();
}
void StateLearningModeCooperativePrepare::HandleSetupView()
{
    wai_oa_context->SetupLearningModeCooperativeView();
}
void StateLearningModeCooperativePrepare::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpEvalGraph
void StatePlenumSettingUpEvalGraph::HandleSetupModel()
{
    wai_oa_context->SetupEvalModel();
}
void StatePlenumSettingUpEvalGraph::HandleSetupView()
{
    wai_oa_context->SetupEvalView();
}
void StatePlenumSettingUpEvalGraph::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpVoiceListen
void StatePlenumSettingUpVoiceListen::HandleSetupModel()
{
    wai_oa_context->SetupVoiceListenModel();
}
void StatePlenumSettingUpVoiceListen::HandleSetupView()
{
    wai_oa_context->SetupVoiceListenView();
}
void StatePlenumSettingUpVoiceListen::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpVoicePrompt
void StatePlenumSettingUpVoicePrompt::HandleSetupModel()
{
    wai_oa_context->SetupVoicePromptModel();
}
void StatePlenumSettingUpVoicePrompt::HandleSetupView()
{
    wai_oa_context->SetupVoicePromptView();
}
void StatePlenumSettingUpVoicePrompt::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpEvalGraph
void StatePlenumSettingUpEvalGraphInspect::HandleSetupModel()
{
    wai_oa_context->SetupEvalGraphInspectModel();
}
void StatePlenumSettingUpEvalGraphInspect::HandleSetupView()
{
    wai_oa_context->SetupEvalGraphInspectView();
}
void StatePlenumSettingUpEvalGraphInspect::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpEvalBowl
void StatePlenumSettingUpEvalBowl::HandleSetupModel()
{
    wai_oa_context->SetupEvalBowlModel();
}
void StatePlenumSettingUpEvalBowl::HandleSetupView()
{
    wai_oa_context->SetupEvalBowlView();
}
void StatePlenumSettingUpEvalBowl::HandleLeaveState()
{
    wai_oa_context->SetupEvalBowlLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpEvalBalance
void StatePlenumSettingUpEvalBalance::HandleSetupModel()
{
    wai_oa_context->SetupEvalBalanceModel();
}
void StatePlenumSettingUpEvalBalance::HandleSetupView()
{
    wai_oa_context->SetupEvalBalanceView();
}
void StatePlenumSettingUpEvalBalance::HandleLeaveState()
{
    wai_oa_context->SetupEvalBalanceLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpEvalMarvin
void StatePlenumSettingUpEvalMarvin::HandleSetupModel()
{
    wai_oa_context->SetupEvalMarvinModel();
}
void StatePlenumSettingUpEvalMarvin::HandleSetupView()
{
    wai_oa_context->SetupEvalMarvinView();
}
void StatePlenumSettingUpEvalMarvin::HandleLeaveState()
{
    wai_oa_context->SetupEvalMarvinLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUp2D3DRespawn
void StatePlenumSettingUp2D3DRespawn::HandleSetupModel()
{
}
void StatePlenumSettingUp2D3DRespawn::HandleSetupView()
{
    wai_oa_context->SetupRespawn2D3DFromScript();
}
void StatePlenumSettingUp2D3DRespawn::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpForceInit
void StatePlenumSettingUpForceInit::HandleSetupModel()
{
    if(wai_oa_context->SetupForceOrigin())
    {
        this->wai_oa_context->RequestSetupView();
    }
    else
    {
        // Leave state if no proper rep was selected
        this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting);
    }
}
void StatePlenumSettingUpForceInit::HandleSetupView()
{
    // Vis. force vector!
    wai_oa_context->SetupForceView(true);
}
void StatePlenumSettingUpForceInit::HandleLeaveState()
{
    wai_oa_context->SetupForceView(false);

    // Cont. to StatePlenumSettingUpForceApply
    this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpForceApply,true);
}

// STATE: StatePlenumSettingUpForceVector
void StatePlenumSettingUpForceVector::HandleSetupModel()
{

}
void StatePlenumSettingUpForceVector::HandleSetupView()
{

}
void StatePlenumSettingUpForceVector::HandleLeaveState()
{

}

// STATE: StatePlenumSettingUpForceApply
void StatePlenumSettingUpForceApply::HandleSetupModel()
{
    wai_oa_context->SetupForceFromTrigger();
}
void StatePlenumSettingUpForceApply::HandleSetupView()
{
}
void StatePlenumSettingUpForceApply::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpAudienceNumber
void StatePlenumSettingUpAudienceNumber::HandleSetupModel()
{
    int i_number_selected=wai_oa_context->SetupWIMAudienceNumberModel();
    wai_oa_context->UpdateAudienceIDSelected(i_number_selected);
    wai_oa_context->UpdateWIMFromAudienceSelected();
    wai_oa_context->SetupWIMAudienceSelectionModel();
}
void StatePlenumSettingUpAudienceNumber::HandleSetupView()
{
    wai_oa_context->SetupWIMAudienceSelectionView();
}
void StatePlenumSettingUpAudienceNumber::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumSettingUpAudienceDown
void StatePlenumSettingUpAudienceDown::HandleSetupModel()
{
    wai_oa_context->UpdateAudienceIDSelected(-1);
    wai_oa_context->UpdateWIMFromAudienceSelected();
    wai_oa_context->SetupWIMAudienceSelectionModel(); // Update only rep for now
}
void StatePlenumSettingUpAudienceDown::HandleSetupView()
{
    wai_oa_context->SetupWIMAudienceSelectionView(); // Update only rep for now
}
void StatePlenumSettingUpAudienceDown::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumSettingUpAudienceUp
void StatePlenumSettingUpAudienceUp::HandleSetupModel()
{
    wai_oa_context->UpdateAudienceIDSelected(-2);
    wai_oa_context->UpdateWIMFromAudienceSelected();
    wai_oa_context->SetupWIMAudienceSelectionModel(); // Update only rep for now
}
void StatePlenumSettingUpAudienceUp::HandleSetupView()
{
    wai_oa_context->SetupWIMAudienceSelectionView(); // Update only rep for now
}
void StatePlenumSettingUpAudienceUp::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumSettingUpAudienceLeft
void StatePlenumSettingUpAudienceLeft::HandleSetupModel()
{
    wai_oa_context->UpdateAudienceIDSelected(-3);
    wai_oa_context->UpdateWIMFromAudienceSelected();
    wai_oa_context->SetupWIMAudienceSelectionModel(); // Update only rep for now
}
void StatePlenumSettingUpAudienceLeft::HandleSetupView()
{
    wai_oa_context->SetupWIMAudienceSelectionView(); // Update only rep for now
}
void StatePlenumSettingUpAudienceLeft::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumSettingUpAudienceRight
void StatePlenumSettingUpAudienceRight::HandleSetupModel()
{
    wai_oa_context->UpdateAudienceIDSelected(-4);
    wai_oa_context->UpdateWIMFromAudienceSelected();
    wai_oa_context->SetupWIMAudienceSelectionModel(); // Update only rep for now
}
void StatePlenumSettingUpAudienceRight::HandleSetupView()
{
    wai_oa_context->SetupWIMAudienceSelectionView(); // Update only rep for now
}
void StatePlenumSettingUpAudienceRight::HandleLeaveState()
{
    // Return to presenting
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpAudienceSelect
void StatePlenumSettingUpAudienceSelect::HandleSetupModel()
{
    wai_oa_context->SetupWIMAudienceSelectionModel(true); // pass true to enable screenshare on select
}
void StatePlenumSettingUpAudienceSelect::HandleSetupView()
{
    wai_oa_context->SetupWIMAudienceSelectionView();
    wai_oa_context->SetupCameraRvizFromAudienceSelected();
}
void StatePlenumSettingUpAudienceSelect::HandleLeaveState()
{
    // Return to presenting
    wai_oa_context->SetupWIMAudienceSelectionLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpAudienceMessage
void StatePlenumSettingUpAudienceMessage::HandleSetupModel()
{
    wai_oa_context->SetupWIMAudienceMessageModel();
}
void StatePlenumSettingUpAudienceMessage::HandleSetupView()
{
    wai_oa_context->SetupWIMAudienceMessageView();
}
void StatePlenumSettingUpAudienceMessage::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpAudienceListenerMessage
void StatePlenumSettingUpAudienceListenerMessage::HandleSetupModel()
{
    wai_oa_context->SetUpWIMAudienceListenerMessageModel();
}
void StatePlenumSettingUpAudienceListenerMessage::HandleSetupView()
{
    wai_oa_context->SetUpWIMAudienceListenerMessageView();
}
void StatePlenumSettingUpAudienceListenerMessage::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpAudienceKick
void StatePlenumSettingUpAudienceKick::HandleSetupModel()
{
    wai_oa_context->SetupWIMAudienceKickModel();
}
void StatePlenumSettingUpAudienceKick::HandleSetupView()
{
    wai_oa_context->SetupWIMAudienceKickView();
}
void StatePlenumSettingUpAudienceKick::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpRepDetailFocus
void StatePlenumSettingUpRepDetailFocus::HandleSetupModel()
{
    wai_oa_context->UpdateRepAndDetailSelected(true);
}
void StatePlenumSettingUpRepDetailFocus::HandleSetupView()
{
    //wai_oa_context->SetupWIMRepSelectionView(); // Not used currently
}
void StatePlenumSettingUpRepDetailFocus::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumSettingUpRepSelect
void StatePlenumSettingUpRepSelect::HandleSetupModel()
{
    wai_oa_context->SetupWIMRepSelectionModel();
}
void StatePlenumSettingUpRepSelect::HandleSetupView()
{
    wai_oa_context->SetupWIMRepSelectionView();
    wai_oa_context->SetupCameraRvizFromRepSelected();
}
void StatePlenumSettingUpRepSelect::HandleLeaveState()
{
    // Return to presenting
    wai_oa_context->SetupWIMRepSelectionLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}



// STATE: StatePlenumLogAudienceIDParticipationMinus
void StatePlenumLogAudienceIDParticipationMinus::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDParticipationModel(0);
}
void StatePlenumLogAudienceIDParticipationMinus::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDParticipationView();
}
void StatePlenumLogAudienceIDParticipationMinus::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumLogAudienceIDParticipationTilde
void StatePlenumLogAudienceIDParticipationTilde::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDParticipationModel(1);
}
void StatePlenumLogAudienceIDParticipationTilde::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDParticipationView();
}
void StatePlenumLogAudienceIDParticipationTilde::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumLogAudienceIDParticipationPlus
void StatePlenumLogAudienceIDParticipationPlus::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDParticipationModel(2);
}
void StatePlenumLogAudienceIDParticipationPlus::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDParticipationView();
}
void StatePlenumLogAudienceIDParticipationPlus::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumLogAudienceIDParticipationScore
void StatePlenumLogAudienceIDParticipationScore::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDParticipationModel(3);
}
void StatePlenumLogAudienceIDParticipationScore::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDParticipationView();
}
void StatePlenumLogAudienceIDParticipationScore::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumLogAudienceIDExaminationInsufficient
void StatePlenumLogAudienceIDExaminationInsufficient::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDExaminationModel(0);
}
void StatePlenumLogAudienceIDExaminationInsufficient::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDExaminationView();
}
void StatePlenumLogAudienceIDExaminationInsufficient::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumLogAudienceIDExaminationSufficient
void StatePlenumLogAudienceIDExaminationSufficient::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDExaminationModel(1);
}
void StatePlenumLogAudienceIDExaminationSufficient::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDExaminationView();
}
void StatePlenumLogAudienceIDExaminationSufficient::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumLogAudienceIDExaminationSatisfactory
void StatePlenumLogAudienceIDExaminationSatisfactory::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDExaminationModel(2);
}
void StatePlenumLogAudienceIDExaminationSatisfactory::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDExaminationView();
}
void StatePlenumLogAudienceIDExaminationSatisfactory::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumLogAudienceIDExaminationGood
void StatePlenumLogAudienceIDExaminationGood::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDExaminationModel(3);
}
void StatePlenumLogAudienceIDExaminationGood::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDExaminationView();
}
void StatePlenumLogAudienceIDExaminationGood::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumLogAudienceIDExaminationVeryGood
void StatePlenumLogAudienceIDExaminationVeryGood::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDExaminationModel(4);
}
void StatePlenumLogAudienceIDExaminationVeryGood::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDExaminationView();
}
void StatePlenumLogAudienceIDExaminationVeryGood::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
// STATE: StatePlenumLogAudienceIDExaminationScore
void StatePlenumLogAudienceIDExaminationScore::HandleSetupModel()
{
    wai_oa_context->SetupLogAudienceIDExaminationModel(5);
}
void StatePlenumLogAudienceIDExaminationScore::HandleSetupView()
{
    wai_oa_context->SetupLogAudienceIDExaminationView();
}
void StatePlenumLogAudienceIDExaminationScore::HandleLeaveState()
{
    // Return to StatePlenumSettingUpEvalGraph
    //this->wai_oa_context->StateTransitionTo(new StatePlenumSettingUpEvalGraph,true);
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}




// STATE: StatePlenumReceivingRequestAudience
void StatePlenumReceivingRequestAudience::HandleSetupModel()
{
    wai_oa_context->SetupReceivingRequestAudienceModel();
}
void StatePlenumReceivingRequestAudience::HandleSetupView()
{
    wai_oa_context->SetupReceivingRequestAudienceView();
}
void StatePlenumReceivingRequestAudience::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumHandlingRequestAudienceAccept
void StatePlenumHandlingRequestAudienceAccept::HandleSetupModel()
{
    wai_oa_context->SetupHandlingRequestAudienceAcceptModel();
}
void StatePlenumHandlingRequestAudienceAccept::HandleSetupView()
{
    wai_oa_context->SetupHandlingRequestAudienceAcceptView();
}
void StatePlenumHandlingRequestAudienceAccept::HandleLeaveState()
{
    wai_oa_context->SetupHandlingRequestAudienceAcceptLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumHandlingRequestAudienceReject
void StatePlenumHandlingRequestAudienceReject::HandleSetupModel()
{
    wai_oa_context->SetupHandlingRequestAudienceRejectModel();
}
void StatePlenumHandlingRequestAudienceReject::HandleSetupView()
{
    wai_oa_context->SetupHandlingRequestAudienceRejectView();
}
void StatePlenumHandlingRequestAudienceReject::HandleLeaveState()
{
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpMarvin
void StatePlenumSettingUpMarvin::HandleSetupModel()
{
    wai_oa_context->SetupMarvinModel();
}
void StatePlenumSettingUpMarvin::HandleSetupView()
{
    wai_oa_context->SetupMarvinView();
}
void StatePlenumSettingUpMarvin::HandleLeaveState()
{
    wai_oa_context->SetupMarvinLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}


// STATE: StatePlenumSettingUpMarvinHologram
void StatePlenumSettingUpMarvinHologram::HandleSetupModel()
{
    wai_oa_context->SetupMarvinHologramModel();
}
void StatePlenumSettingUpMarvinHologram::HandleSetupView()
{
    wai_oa_context->SetupMarvinHologramView();
}
void StatePlenumSettingUpMarvinHologram::HandleLeaveState()
{
    wai_oa_context->SetupMarvinHologramLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}

// STATE: StatePlenumSettingUpIntroduction
void StatePlenumSettingUpIntroduction::HandleSetupModel()
{
    wai_oa_context->SetupIntroductionModel();
}
void StatePlenumSettingUpIntroduction::HandleSetupView()
{
    wai_oa_context->SetupIntroductionView();
}
void StatePlenumSettingUpIntroduction::HandleLeaveState()
{
    this->wai_oa_context->SetupIntroductionLeaveState();
    this->wai_oa_context->StateTransitionTo(new StatePlenumPresenting,true);
}
