#ifndef WAI_OA_STATE_H
#define WAI_OA_STATE_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<wai_oa.h>



/////////////////////////////////////////////////
/// State pattern of Auditorium Server
/////////////////////////////////////////////////
class WAIOAState
{
protected:
    WAIOpenAuditorium* wai_oa_context;

public:
    virtual ~WAIOAState()
    {
    }

    void SetContext(WAIOpenAuditorium* context)
    {
        this->wai_oa_context=context;
    }

    virtual void HandleSetupModel()=0;
    virtual void HandleSetupView()=0;
    virtual void HandleLeaveState()=0;
};

class StatePlenumPresenting : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpScenePrev : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpSceneNext : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpSceneSelect : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpCameraRvizDefault : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpCameraRvizCycle : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpCameraRvizSelect : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpCameraRvizEnforce : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpCameraRvizFollow : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpCameraRvizIdle : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpBodyInteraction : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpAvatar : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpLectern : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpTable : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpAudio : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpSketch : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StateLearningModePlenumPrepare : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StateLearningModeCooperativePrepare : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpEvalGraph : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpEvalGraphInspect : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpEvalBowl : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpEvalBalance : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpEvalMarvin : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpVoiceListen : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpVoicePrompt : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpPresenceMode : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUp2D3DRespawn : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpForceInit : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpForceVector : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpForceApply : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpAudienceNumber : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumSettingUpAudienceDown : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumSettingUpAudienceUp : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumSettingUpAudienceLeft : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumSettingUpAudienceRight : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpAudienceSelect : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpAudienceMessage : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpAudienceListenerMessage : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpAudienceKick : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpRepDetailFocus : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumSettingUpRepSelect : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumLogAudienceIDParticipationMinus : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumLogAudienceIDParticipationTilde : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumLogAudienceIDParticipationPlus : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumLogAudienceIDParticipationScore : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};


class StatePlenumLogAudienceIDExaminationInsufficient : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumLogAudienceIDExaminationSufficient : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumLogAudienceIDExaminationSatisfactory : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumLogAudienceIDExaminationGood : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumLogAudienceIDExaminationVeryGood : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};
class StatePlenumLogAudienceIDExaminationScore : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumReceivingRequestAudience : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumHandlingRequestAudienceAccept : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumHandlingRequestAudienceReject : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumReceivingRequestServer : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpMarvin : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpMarvinHologram : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};

class StatePlenumSettingUpIntroduction : public WAIOAState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleLeaveState();
};


#endif //WAI_OA_STATE_H
