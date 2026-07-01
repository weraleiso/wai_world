// Current Version 4.0

#include<ros/ros.h>
#include<ros/package.h>
#include<std_msgs/Empty.h>
#include<std_msgs/String.h>

#include<picovoice.h>
#include<pv_rhino.h>
#include<pv_cheetah.h>
#include<pv_orca.h>
#include<pv_picollm.h>
#include<pv_porcupine.h>
extern "C"
{
#include<pv_recorder.h>
#include<pv_speaker.h>
}

#include<iomanip>
#include<array>
#include<vector>
#include<map>
#include<ostream>
#include<sstream>
#include<string>
#include<bits/stdc++.h>

#include<stdio.h>
#include<string.h>
#include<math.h>
#include<getopt.h>
#include<dlfcn.h>
#include<sys/time.h>



// PICOLLM - Helper definitions
static struct timeval tic;
static int32_t num_tokens=-1;
static volatile bool b_picollm_interrupt=false; // Unused currently!

// ORCA - Helper definitions

/*
DR WAV - Deprecated
#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h" // Including it with catkin is a pain here, strictly obey include order
#define MAX_NUM_CHUNKS              (500)
#define MAX_NUM_BYTES_PER_CHARACTER (5)
typedef struct pcm_chunk pcm_chunk_t;
struct pcm_chunk
{
    int32_t num_samples;
    int16_t *pcm;
    pcm_chunk_t *next;
};
*/

// PVSPEAKER - Helper definitions
typedef struct
{
    uint8_t chunk_id[4];
    uint32_t chunk_size;
    uint8_t format[4];
    uint8_t subchunk1_id[4];
    uint32_t subchunk1_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    uint8_t subchunk2_id[4];
    uint32_t subchunk2_size;
} wav_header;



class WAIPicovoice
{
    ros::NodeHandle* m_p_hdl_node;
    std::string m_s_path_nodename;
    std::string m_s_path_resources;

    float F_NODE_SAMPLE_FREQUENCY;

    std::string S_PICOVOICE_ACCESS_KEY;

    std::string S_RHINO_MODEL_NAME;
    std::string S_RHINO_CONTEXT_NAME;
    std::string S_RHINO_MODEL_PATH;
    std::string S_RHINO_CONTEXT_PATH;
    float F_RHINO_SENSITIVITY;
    float F_RHINO_ENDPOINT_DURATION;
    bool B_RHINO_ENDPOINT_REQUIRED;
    bool B_RHINO_ENABLE_CONTEXT_INFO;
    std::string m_s_intent;
    std::vector<std::string> m_vec_s_slots;
    std::vector<std::string> m_vec_s_slots_values;

    std::string S_CHEETAH_MODEL_NAME;
    std::string S_CHEETAH_MODEL_PATH;
    float F_CHEETAH_ENDPOINT_DURATION;
    bool B_CHEETAH_ENABLE_PUNCTUATION;
    std::string m_s_cheetah_transcript;

    std::string S_PICOLLM_MODEL_NAME;
    std::string S_PICOLLM_MODEL_PATH;
    float F_PICOLLM_PRESENCE_PENALTY;
    float F_PICOLLM_FREQUENCY_PENALTY;
    int I_PICOLLM_SEED;
    float F_PICOLLM_TOP_P;
    float F_PICOLLM_TEMPERATURE;
    int I_PICOLLM_MAX_OUTPUT_TOKENS;
    std::string m_s_picollm_response;

    std::string S_ORCA_MODEL_NAME;
    std::string S_ORCA_MODEL_PATH;
    std::string S_ORCA_WAV_FILE_NAME;
    std::string S_ORCA_WAV_FILE_PATH;

    ros::Subscriber m_sub_str_picovoice_synthesize_text;
    ros::Subscriber m_sub_str_picovoice_prompt_text;
    ros::Subscriber m_sub_emp_picovoice_prompt_voice;

    ros::Timer m_tmr_picovoice;

public:
    WAIPicovoice()
    {
    }
    ~WAIPicovoice()
    {
    }
    // PICOVOICE - Initialize all parameters
    //=======================================
    void PicovoiceInitialize(ros::NodeHandle* p_hdl_node,
                             float f_node_sample_frequency,
                             std::string s_picovoice_access_key)
    {
        m_p_hdl_node=p_hdl_node;

        // Initialize Resources
        m_s_path_nodename=ros::this_node::getName();
        //m_s_path_resources=ros::package::getPath("wai_oa_gazebo")+"/resources/picovoice/";
        m_s_path_resources="/home/ias/picovoice/"; // Changed to home folder!

        // Initialize Picovoice Modules
        //==============================
        if(ros::param::has(m_s_path_nodename+"/"+"F_NODE_SAMPLE_FREQUENCY")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"F_NODE_SAMPLE_FREQUENCY",F_NODE_SAMPLE_FREQUENCY);
        else F_NODE_SAMPLE_FREQUENCY=f_node_sample_frequency;
        if(ros::param::has(m_s_path_nodename+"/"+"S_PICOVOICE_ACCESS_KEY")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"S_PICOVOICE_ACCESS_KEY",S_PICOVOICE_ACCESS_KEY);
        else S_PICOVOICE_ACCESS_KEY=s_picovoice_access_key;

        // RHINO - Speech-To-Intent Recognizer
        // (Using PVRECORDER - Cross-platform audio recorder)
        if(ros::param::has(m_s_path_nodename+"/"+"S_RHINO_MODEL_NAME")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"S_RHINO_MODEL_NAME",S_RHINO_MODEL_NAME);
        else S_RHINO_MODEL_NAME="rhino_params.pv";
        if(ros::param::has(m_s_path_nodename+"/"+"S_RHINO_CONTEXT_NAME")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"S_RHINO_CONTEXT_NAME",S_RHINO_CONTEXT_NAME);
        else S_RHINO_CONTEXT_NAME="OA_en_linux_v4_0_0.rhn";
        //else S_RHINO_CONTEXT_NAME="OA_en_linux_v3_0_0.rhn";
        if(ros::param::has(m_s_path_nodename+"/"+"F_RHINO_SENSITIVITY")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"F_RHINO_SENSITIVITY",F_RHINO_SENSITIVITY);
        else F_RHINO_SENSITIVITY=0.5;
        if(ros::param::has(m_s_path_nodename+"/"+"F_RHINO_ENDPOINT_DURATION")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"F_RHINO_ENDPOINT_DURATION",F_RHINO_ENDPOINT_DURATION);
        else F_RHINO_ENDPOINT_DURATION=2.0;
        if(ros::param::has(m_s_path_nodename+"/"+"B_RHINO_ENDPOINT_REQUIRED")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"B_RHINO_ENDPOINT_REQUIRED",B_RHINO_ENDPOINT_REQUIRED);
        else B_RHINO_ENDPOINT_REQUIRED=true;
        if(ros::param::has(m_s_path_nodename+"/"+"B_RHINO_ENABLE_CONTEXT_INFO")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"B_RHINO_ENABLE_CONTEXT_INFO",B_RHINO_ENABLE_CONTEXT_INFO);
        else B_RHINO_ENABLE_CONTEXT_INFO=false;
        S_RHINO_MODEL_PATH=m_s_path_resources+S_RHINO_MODEL_NAME;
        S_RHINO_CONTEXT_PATH=m_s_path_resources+S_RHINO_CONTEXT_NAME;
        m_s_intent="RhinoIntentError";
        m_vec_s_slots.clear();
        m_vec_s_slots_values.clear();

        // CHEETAH - Streaming Speech-To-Text Recognizer
        if(ros::param::has(m_s_path_nodename+"/"+"S_CHEETAH_MODEL_NAME")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"S_CHEETAH_MODEL_NAME",S_CHEETAH_MODEL_NAME);
        else S_CHEETAH_MODEL_NAME="cheetah_params.pv";
        if(ros::param::has(m_s_path_nodename+"/"+"F_CHEETAH_ENDPOINT_DURATION")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"F_CHEETAH_ENDPOINT_DURATION",F_CHEETAH_ENDPOINT_DURATION);
        else F_CHEETAH_ENDPOINT_DURATION=1.0;
        if(ros::param::has(m_s_path_nodename+"/"+"B_CHEETAH_ENABLE_PUNCTUATION")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"B_CHEETAH_ENABLE_PUNCTUATION",B_CHEETAH_ENABLE_PUNCTUATION);
        else B_CHEETAH_ENABLE_PUNCTUATION=true;
        S_CHEETAH_MODEL_PATH=m_s_path_resources+S_CHEETAH_MODEL_NAME;
        m_s_cheetah_transcript="No transcript available.";

        // PICOLLM - LLM Inference Engine
        if(ros::param::has(m_s_path_nodename+"/"+"S_PICOLLM_MODEL_NAME")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"S_PICOLLM_MODEL_NAME",S_PICOLLM_MODEL_NAME);
        else S_PICOLLM_MODEL_NAME="llama-3.2-3b-instruct-505.pllm";
        if(ros::param::has(m_s_path_nodename+"/"+"F_PICOLLM_PRESENCE_PENALTY")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"F_PICOLLM_PRESENCE_PENALTY",F_PICOLLM_PRESENCE_PENALTY);
        else F_PICOLLM_PRESENCE_PENALTY=0.75f;
        if(ros::param::has(m_s_path_nodename+"/"+"F_PICOLLM_FREQUENCY_PENALTY")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"F_PICOLLM_FREQUENCY_PENALTY",F_PICOLLM_FREQUENCY_PENALTY);
        else F_PICOLLM_FREQUENCY_PENALTY=0.75f;
        if(ros::param::has(m_s_path_nodename+"/"+"I_PICOLLM_SEED")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"I_PICOLLM_SEED",I_PICOLLM_SEED);
        else I_PICOLLM_SEED=5;
        if(ros::param::has(m_s_path_nodename+"/"+"F_PICOLLM_TOP_P")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"F_PICOLLM_TOP_P",F_PICOLLM_TOP_P);
        else F_PICOLLM_TOP_P=0.9;
        if(ros::param::has(m_s_path_nodename+"/"+"F_PICOLLM_TEMPERATURE")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"F_PICOLLM_TEMPERATURE",F_PICOLLM_TEMPERATURE);
        else F_PICOLLM_TEMPERATURE=0.0;
        if(ros::param::has(m_s_path_nodename+"/"+"I_PICOLLM_MAX_OUTPUT_TOKENS")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"I_PICOLLM_MAX_OUTPUT_TOKENS",I_PICOLLM_MAX_OUTPUT_TOKENS);
        else I_PICOLLM_MAX_OUTPUT_TOKENS=50;
        S_PICOLLM_MODEL_PATH=m_s_path_resources+S_PICOLLM_MODEL_NAME;
        m_s_picollm_response="No prompt answer available.";

        // ORCA - Streaming text-to-speech engine
        // (Using PVSPEAKER - Cross-platform audio player)
        if(ros::param::has(m_s_path_nodename+"/"+"S_ORCA_MODEL_NAME")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"S_ORCA_MODEL_NAME",S_ORCA_MODEL_NAME);
        else S_ORCA_MODEL_NAME="orca_params_en_female.pv";
        if(ros::param::has(m_s_path_nodename+"/"+"S_ORCA_WAV_FILE_NAME")) m_p_hdl_node->getParam(m_s_path_nodename+"/"+"S_ORCA_WAV_FILE_NAME",S_ORCA_WAV_FILE_NAME);
        else S_ORCA_WAV_FILE_NAME="orca_synthesized.wav";
        S_ORCA_MODEL_PATH=m_s_path_resources+S_ORCA_MODEL_NAME;
        S_ORCA_WAV_FILE_PATH=m_s_path_resources+S_ORCA_WAV_FILE_NAME;

        // Initialize Subscribers
        m_sub_str_picovoice_synthesize_text=m_p_hdl_node->subscribe("picovoice_synthesize_text",1,&WAIPicovoice::cb_sub_str_picovoice_synthesize_text,this);
        m_sub_str_picovoice_prompt_text=m_p_hdl_node->subscribe("picovoice_prompt_text",1,&WAIPicovoice::cb_sub_str_picovoice_prompt_text,this);
        m_sub_emp_picovoice_prompt_voice=m_p_hdl_node->subscribe("picovoice_prompt_voice",1,&WAIPicovoice::cb_sub_emp_picovoice_prompt_voice,this);

        // Initialize Publishers

        // Initialize Timers
        //m_tmr_picovoice=m_p_hdl_node->createTimer(ros::Duration(1.0/F_NODE_SAMPLE_FREQUENCY),&WAIPicovoice::cb_tmr_picovoice,this,false,false);
    }
    void PicoVoicePipelineTest()
    {
        RhinoInitialize(S_RHINO_MODEL_NAME,
                        S_RHINO_CONTEXT_NAME,
                        F_RHINO_SENSITIVITY,
                        F_RHINO_ENDPOINT_DURATION,
                        B_RHINO_ENDPOINT_REQUIRED,
                        B_RHINO_ENABLE_CONTEXT_INFO);
        RhinoStartRecognizing();

        CheetahInitialize(S_CHEETAH_MODEL_NAME,
                        F_CHEETAH_ENDPOINT_DURATION,
                        B_CHEETAH_ENABLE_PUNCTUATION);
        CheetahStartRecognizing();

        PicoLLMInitialize(S_PICOLLM_MODEL_NAME);
        PicoLLMStartProcessing(CheetahGetLatestTranscript());

        OrcaInitialize(S_ORCA_MODEL_PATH,
                       S_ORCA_WAV_FILE_NAME);
        OrcaStartProcessing(PicoLLMGetLatestResponse());
    }

    void PlaySynthesizedWAVFile()
    {
        // Prompt response is always non-blocking
        std::string s_command="canberra-gtk-play -V 0.0 -f "+OrcaGetSynthesizedWAVFilePath()+" &";
        int i_retval=system(s_command.c_str());
    }

    void PicovoiceProcessTextToSpeech(std::string s_text,bool b_male=false)
    {
        if(b_male==true)
        {
            OrcaSelectModel("orca_params_en_male.pv");
        }
        else
        {
            OrcaSelectModel("orca_params_en_female.pv");
        }
        OrcaStartProcessing(s_text);
        PlaySynthesizedWAVFile();
    }

    void PicovoiceProcessIntent()
    {
        RhinoStartRecognizing();
    }
    void PicovoiceProcessTextPrompt(std::string s_prompt_text_incoming)
    {
        PicoLLMStartProcessing(s_prompt_text_incoming);
        OrcaStartProcessing(PicoLLMGetLatestResponse());
        PlaySynthesizedWAVFile();
    }
    void PicovoiceProcessVoicePrompt()
    {
        CheetahStartRecognizing();
        PicoLLMStartProcessing(CheetahGetLatestTranscript());
        OrcaStartProcessing(PicoLLMGetLatestResponse());
        PlaySynthesizedWAVFile();
    }

    void cb_sub_str_picovoice_synthesize_text(const std_msgs::StringConstPtr& msg)
    {
        OrcaSelectModel("orca_params_en_female.pv");
        OrcaStartProcessing(msg->data);
        PlaySynthesizedWAVFile();
    }
    void cb_sub_str_picovoice_prompt_text(const std_msgs::StringConstPtr& msg)
    {
        PicovoiceProcessTextPrompt(msg->data);
    }
    void cb_sub_emp_picovoice_prompt_voice(const std_msgs::EmptyConstPtr& msg)
    {
        PicovoiceProcessVoicePrompt();
    }

    void cb_tmr_picovoice(const ros::TimerEvent& event)
    {
        ros::spinOnce();
    }


    // RHINO - Speech-To-Intent Recognizer
    //=====================================
    void print_error_message(char **message_stack, int32_t message_stack_depth)
    {
        for(int32_t i=0; i<message_stack_depth; i++)
        {
            fprintf(stderr, "  [%d] %s\n", i, message_stack[i]);
        }
    }
    std::string RhinoGetLatestIntent()
    {
        return m_s_intent;
    }
    std::vector<std::string> RhinoGetLatestIntentSlots()
    {
        return m_vec_s_slots;
    }
    std::vector<std::string> RhinoGetLatestIntentSlotsValues()
    {
        return m_vec_s_slots_values;
    }
    void RhinoInitialize(std::string s_model_name,
                        std::string s_context_name,
                        float f_sensitivity,
                        float f_endpoint_duration,
                        bool b_endpoint_required,
                        bool b_enable_context_info)
    {
        S_RHINO_MODEL_NAME=s_model_name;
        S_RHINO_MODEL_PATH=m_s_path_resources+S_RHINO_MODEL_NAME;
        S_RHINO_CONTEXT_NAME=s_context_name;
        S_RHINO_CONTEXT_PATH=m_s_path_resources+S_RHINO_CONTEXT_NAME;
        F_RHINO_SENSITIVITY=f_sensitivity;
        F_RHINO_ENDPOINT_DURATION=f_endpoint_duration;
        B_RHINO_ENDPOINT_REQUIRED=b_endpoint_required;
        B_RHINO_ENABLE_CONTEXT_INFO=b_enable_context_info;
    }
    void RhinoStartRecognizing()
    {
        ROS_WARN_STREAM("Picovoice - RHINO: STARTED with speech intent recognition...");

        const char *access_key=S_PICOVOICE_ACCESS_KEY.c_str();
        const char *model_path=S_RHINO_MODEL_PATH.c_str();
        const char *context_path=S_RHINO_CONTEXT_PATH.c_str();
        int32_t device_index=-1;
        float sensitivity=F_RHINO_SENSITIVITY;
        float endpoint_duration_sec=F_RHINO_ENDPOINT_DURATION;
        bool require_endpoint=B_RHINO_ENDPOINT_REQUIRED;
        m_s_intent="RhinoIntentError";
        m_vec_s_slots.clear();
        m_vec_s_slots_values.clear();

        char **message_stack=NULL;
        int32_t message_stack_depth=0;
        pv_status_t error_status=PV_STATUS_RUNTIME_ERROR;

        pv_rhino_t *rhino=NULL;
        pv_status_t status=pv_rhino_init(
                access_key,
                model_path,
                "best",
                context_path,
                sensitivity,
                endpoint_duration_sec,
                require_endpoint,
                &rhino);
        if(status!=PV_STATUS_SUCCESS)
        {
            fprintf(stderr, "'pv_rhino_init' failed with '%s'", pv_status_to_string(status));
            error_status=pv_get_error_stack(&message_stack, &message_stack_depth);
            if(error_status!=PV_STATUS_SUCCESS)
            {
                fprintf(stderr, ".\nUnable to get Rhino error state with '%s'\n", pv_status_to_string(error_status));
                return; //exit(1);
            }
            if(message_stack_depth>0)
            {
                fprintf(stderr, ":\n");
                print_error_message(message_stack, message_stack_depth);
            }
            pv_free_error_stack(message_stack);
            return; //exit(1);
        }

        fprintf(stdout, "Picovoice Rhino Speech-to-Intent (%s)\n\n", pv_rhino_version());

        const int32_t frame_length=pv_rhino_frame_length();
        pv_recorder_t *recorder=NULL;
        pv_recorder_status_t recorder_status=pv_recorder_init(frame_length, device_index, 100, &recorder);
        if(recorder_status!=PV_RECORDER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to initialize device with %s.\n", pv_recorder_status_to_string(recorder_status));
            return; //exit(1);
        }

        if(B_RHINO_ENABLE_CONTEXT_INFO==true)
        {
            const char *context_info=NULL;
            status=pv_rhino_context_info(rhino, &context_info);
            if(status!=PV_STATUS_SUCCESS)
            {
                fprintf(stderr, "'pv_rhino_context_info' failed with '%s'", pv_status_to_string(status));
                error_status=pv_get_error_stack(&message_stack, &message_stack_depth);

                if(error_status!=PV_STATUS_SUCCESS)
                {
                    fprintf(stderr, ".\nUnable to get Rhino error state with '%s'\n", pv_status_to_string(error_status));
                    return; //exit(1);
                }

                if(message_stack_depth>0)
                {
                    fprintf(stderr, ":\n");
                    print_error_message(message_stack, message_stack_depth);
                }

                pv_free_error_stack(message_stack);
                return; //exit(1);
            }
            fprintf(stdout, "%s\n\n", context_info);
        }

        const char *selected_device=pv_recorder_get_selected_device(recorder);
        fprintf(stdout, "Selected device: %s.\n", selected_device);
        fprintf(stdout, "Listening...\n\n");

        recorder_status=pv_recorder_start(recorder);
        if(recorder_status!=PV_RECORDER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to start device with %s.\n", pv_recorder_status_to_string(recorder_status));
            return; //exit(1);
        }

        int16_t *pcm=(int16_t*)malloc(frame_length * sizeof(int16_t));
        if(!pcm)
        {
            fprintf(stderr, "Failed to allocate pcm memory.\n");
            return; //exit(1);
        }

        while(true)
        {
            recorder_status=pv_recorder_read(recorder, pcm);
            if(recorder_status!=PV_RECORDER_STATUS_SUCCESS)
            {
                fprintf(stderr, "Failed to read with %s.\n", pv_recorder_status_to_string(recorder_status));
                return; //exit(1);
            }

            bool is_finalized=false;
            status=pv_rhino_process(
                    rhino,
                    pcm,
                    &is_finalized);

            if(status!=PV_STATUS_SUCCESS)
            {
                fprintf(stderr, "'pv_rhino_process' failed with '%s'", pv_status_to_string(status));
                error_status=pv_get_error_stack(&message_stack, &message_stack_depth);

                if(error_status!=PV_STATUS_SUCCESS)
                {
                    fprintf(stderr, ".\nUnable to get Rhino error state with '%s'\n", pv_status_to_string(error_status));
                    return; //exit(1);
                }

                if(message_stack_depth>0)
                {
                    fprintf(stderr, ":\n");
                    print_error_message(message_stack, message_stack_depth);
                }

                pv_free_error_stack(message_stack);
                return; //exit(1);
            }

            if(is_finalized)
            {
                bool is_understood=false;
                status=pv_rhino_is_understood(rhino, &is_understood);
                if(status!=PV_STATUS_SUCCESS)
                {
                    fprintf(stderr, "'pv_rhino_is_understood' failed with '%s'", pv_status_to_string(status));
                    error_status=pv_get_error_stack(&message_stack, &message_stack_depth);

                    if(error_status!=PV_STATUS_SUCCESS)
                    {
                        fprintf(stderr, ".\nUnable to get Rhino error state with '%s'\n", pv_status_to_string(error_status));
                        return; //exit(1);
                    }

                    if(message_stack_depth>0)
                    {
                        fprintf(stderr, ":\n");
                        print_error_message(message_stack, message_stack_depth);
                    }

                    pv_free_error_stack(message_stack);
                    return; //exit(1);
                }

                const char *intent=NULL;
                int32_t num_pvslots=0;
                const char **pvslots=NULL;
                const char **values=NULL;

                if(is_understood)
                {
                    status=pv_rhino_get_intent(
                            rhino,
                            &intent,
                            &num_pvslots,
                            &pvslots,
                            &values);
                    if(status!=PV_STATUS_SUCCESS)
                    {
                        fprintf(stderr, "'pv_rhino_get_intent' failed with '%s'", pv_status_to_string(status));
                        error_status=pv_get_error_stack(&message_stack, &message_stack_depth);

                        if(error_status!=PV_STATUS_SUCCESS)
                        {
                            fprintf(stderr, ".\nUnable to get Rhino error state with '%s'\n", pv_status_to_string(error_status));
                            return; //exit(1);
                        }

                        if(message_stack_depth>0)
                        {
                            fprintf(stderr, ":\n");
                            print_error_message(message_stack, message_stack_depth);
                        }

                        pv_free_error_stack(message_stack);
                        return; //exit(1);
                    }
                }

                fprintf(stdout, "{\n");
                fprintf(stdout, "    'is_understood' : '%s',\n", is_understood ? "true" : "false");
                if(is_understood)
                {
                    m_s_intent=intent;
                    for(int32_t i=0;i<num_pvslots;i++)
                    {
                        m_vec_s_slots.push_back(pvslots[i]);
                        m_vec_s_slots_values.push_back(values[i]);
                    }
                    fprintf(stdout, "    'intent' : '%s',\n", intent);
                    if(num_pvslots>0)
                    {
                        fprintf(stdout, "    'pvslots' : {\n");
                        for(int32_t i=0; i<num_pvslots; i++)
                        {
                            fprintf(stdout, "        '%s' : '%s',\n", pvslots[i], values[i]);
                        }
                        fprintf(stdout, "    }\n");
                    }
                }
                fprintf(stdout, "}\n");
                fflush(stdout);

                if(is_understood)
                {
                    status=pv_rhino_free_slots_and_values(rhino, pvslots, values);
                    if(status!=PV_STATUS_SUCCESS)
                    {
                        fprintf(stderr, "'pv_rhino_free_slots_and_values' failed with '%s'", pv_status_to_string(status));
                        error_status=pv_get_error_stack(&message_stack, &message_stack_depth);

                        if(error_status!=PV_STATUS_SUCCESS)
                        {
                            fprintf(stderr, ".\nUnable to get Rhino error state with '%s'\n", pv_status_to_string(error_status));
                            return; //exit(1);
                        }

                        if(message_stack_depth>0)
                        {
                            fprintf(stderr, ":\n");
                            print_error_message(message_stack, message_stack_depth);
                        }

                        pv_free_error_stack(message_stack);
                        return; //exit(1);
                    }
                }

                status=pv_rhino_reset(rhino);
                if(status!=PV_STATUS_SUCCESS)
                {
                    fprintf(stderr, "'pv_rhino_reset' failed with '%s'", pv_status_to_string(status));
                    error_status=pv_get_error_stack(&message_stack, &message_stack_depth);

                    if(error_status!=PV_STATUS_SUCCESS)
                    {
                        fprintf(stderr, ".\nUnable to get Rhino error state with '%s'\n", pv_status_to_string(error_status));
                        return; //exit(1);
                    }

                    if(message_stack_depth>0)
                    {
                        fprintf(stderr, ":\n");
                        print_error_message(message_stack, message_stack_depth);
                    }

                    pv_free_error_stack(message_stack);
                    return; //exit(1);
                }

                break;
            }
        }

        recorder_status=pv_recorder_stop(recorder);
        if(recorder_status!=PV_RECORDER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to stop device with %s.\n", pv_recorder_status_to_string(recorder_status));
            return; //exit(1);
        }

        free(pcm);
        pv_recorder_delete(recorder);
        pv_rhino_delete(rhino);

        ROS_WARN_STREAM("Picovoice - RHINO: FINISHED with speech intent recognition!" << std::endl << std::endl);
    }



    // CHEETAH - Streaming Speech-To-Text Recognizer
    //===============================================
    std::string CheetahGetLatestTranscript()
    {
        return m_s_cheetah_transcript;
    }
    void CheetahInitialize(std::string s_model_name,
                         float f_endpoint_duration,
                         bool b_enable_automatic_punctuation)
    {
        S_CHEETAH_MODEL_NAME=s_model_name;
        S_CHEETAH_MODEL_PATH=m_s_path_resources+S_CHEETAH_MODEL_NAME;
        F_CHEETAH_ENDPOINT_DURATION=f_endpoint_duration;
        B_CHEETAH_ENABLE_PUNCTUATION=b_enable_automatic_punctuation;
    }
    void CheetahStartRecognizing()
    {
        ROS_WARN_STREAM("Picovoice - CHEETAH: STARTED with streaming speech-to-text recognition...");

        const char *access_key=S_PICOVOICE_ACCESS_KEY.c_str();
        const char *model_path=S_CHEETAH_MODEL_PATH.c_str();
        float endpoint_duration_sec=F_CHEETAH_ENDPOINT_DURATION;
        bool enable_automatic_punctuation=B_CHEETAH_ENABLE_PUNCTUATION;
        int32_t device_index=-1;
        m_s_cheetah_transcript="";

        char **message_stack=NULL;
        int32_t message_stack_depth=0;
        pv_status_t error_status=PV_STATUS_RUNTIME_ERROR;

        pv_cheetah_t *cheetah=NULL;
        pv_status_t status=pv_cheetah_init(
            access_key,
            model_path,
            endpoint_duration_sec,
            enable_automatic_punctuation,
            &cheetah);
        if(status!=PV_STATUS_SUCCESS)
        {
            fprintf(
                stderr,
                "Failed to init with '%s'",
                pv_status_to_string(status));
            error_status=pv_get_error_stack(&message_stack, &message_stack_depth);
            if(error_status!=PV_STATUS_SUCCESS)
            {
                fprintf(
                        stderr,
                        ".\nUnable to get Cheetah error state with '%s'.\n",
                        pv_status_to_string(error_status));
                return; //exit(1);
            }

            if(message_stack_depth>0)
            {
                fprintf(stderr, ":\n");
                print_error_message(message_stack, message_stack_depth);
            }
            else
            {
                fprintf(stderr, ".\n");
            }

            pv_free_error_stack(message_stack);
            return; //exit(1);
        }

        fprintf(stdout, "Cheetah V%s\n", pv_cheetah_version());

        const int32_t frame_length=pv_cheetah_frame_length();
        pv_recorder_t *recorder=NULL;
        pv_recorder_status_t recorder_status=pv_recorder_init(frame_length, device_index, 1000, &recorder);
        if(recorder_status!=PV_RECORDER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to initialize audio device with '%s'.\n", pv_recorder_status_to_string(recorder_status));
            return; //exit(1);
        }

        int16_t *pcm=(int16_t*)malloc(frame_length * sizeof(int16_t));
        if(!pcm)
        {
            fprintf(stderr, "Failed to allocate pcm memory.\n");
            return; //exit(1);
        }

        const char *selected_device=pv_recorder_get_selected_device(recorder);
        fprintf(stdout, "selected device: %s.\n", selected_device);

        fprintf(stdout, "start recording...\n");
        recorder_status=pv_recorder_start(recorder);
        if(recorder_status!=PV_RECORDER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to start device with %s.\n", pv_recorder_status_to_string(recorder_status));
            return; //exit(1);
        }

        while(true)
        {
            recorder_status=pv_recorder_read(recorder, pcm);
            if(recorder_status!=PV_RECORDER_STATUS_SUCCESS)
            {
                fprintf(stderr, "Failed to read with '%s'.\n", pv_recorder_status_to_string(recorder_status));
                return; //exit(1);
            }

            char *partial_transcript=NULL;
            bool is_endpoint=false;
            status=pv_cheetah_process(cheetah, pcm, &partial_transcript, &is_endpoint);
            if(status!=PV_STATUS_SUCCESS)
            {
                fprintf(
                    stderr,
                    "Failed to process with '%s'",
                    pv_status_to_string(status));
                error_status=pv_get_error_stack(&message_stack, &message_stack_depth);
                if(error_status!=PV_STATUS_SUCCESS)
                {
                    fprintf(
                            stderr,
                            ".\nUnable to get Cheetah error state with '%s'.\n",
                            pv_status_to_string(error_status));
                    return; //exit(1);
                }

                if(message_stack_depth>0)
                {
                    fprintf(stderr, ":\n");
                    print_error_message(message_stack, message_stack_depth);
                }
                else
                {
                    fprintf(stderr, ".\n");
                }

                pv_free_error_stack(message_stack);
                return; //exit(1);
            }
            m_s_cheetah_transcript+=partial_transcript;
            fprintf(stdout, "%s", partial_transcript);
            fflush(stdout);
            pv_cheetah_transcript_delete(partial_transcript);
            if(is_endpoint)
            {
                char *final_transcript=NULL;
                status=pv_cheetah_flush(cheetah, &final_transcript);
                if(status!=PV_STATUS_SUCCESS)
                {
                    fprintf(
                        stderr,
                        "Failed to flush with '%s'",
                        pv_status_to_string(status));
                    error_status=pv_get_error_stack(&message_stack, &message_stack_depth);
                    if(error_status!=PV_STATUS_SUCCESS)
                    {
                        fprintf(
                                stderr,
                                ".\nUnable to get Cheetah error state with '%s'.\n",
                                pv_status_to_string(error_status));
                        return; //exit(1);
                    }

                    if(message_stack_depth>0)
                    {
                        fprintf(stderr, ":\n");
                        print_error_message(message_stack, message_stack_depth);
                    }
                    else
                    {
                        fprintf(stderr, ".\n");
                    }

                    pv_free_error_stack(message_stack);
                    return; //exit(1);
                }
                m_s_cheetah_transcript+=final_transcript;
                fprintf(stdout, "%s\n", final_transcript);
                pv_cheetah_transcript_delete(final_transcript);
                break;
            }
        }
        fprintf(stdout, "\n");

        recorder_status=pv_recorder_stop(recorder);
        if(recorder_status!=PV_RECORDER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to stop device with '%s'.\n", pv_recorder_status_to_string(recorder_status));
            return; //exit(1);
        }

        free(pcm);
        pv_recorder_delete(recorder);
        pv_cheetah_delete(cheetah);

        ROS_WARN_STREAM("Picovoice - CHEEATH: FINISHED with streaming speech-to-text recognition!" << std::endl << std::endl);
    }



    // PICOLLM - LLM inference engine
    //================================
    void picollm_print_error_message(char **message_stack,
                                    int32_t message_stack_depth,
                                    pv_status_t (*pv_get_error_stack_func)(char ***, int32_t *),
                                    void (*pv_free_error_stack_func)(char **),
                                    const char *(*pv_status_to_string_func)(pv_status_t))
    {
        pv_status_t error_status=pv_get_error_stack_func(&message_stack, &message_stack_depth);
        if(error_status!=PV_STATUS_SUCCESS)
        {
            fprintf(
                    stderr,
                    "Unable to get Picovoice error state with '%s'.\n",
                    pv_status_to_string_func(error_status));
            //exit(EXIT_FAILURE);
        }

        if(message_stack_depth>0)
        {
            fprintf(stderr, ":\n");
            for(int32_t i=0; i<message_stack_depth; i++)
            {
                fprintf(stderr, "  [%d] %s\n", i, message_stack[i]);
            }
        }
        else
        {
            fprintf(stderr, ".\n");
        }

        pv_free_error_stack_func(message_stack);
        //exit(EXIT_FAILURE);
    }
    static const char *picollm_endpoint_to_string(pv_picollm_endpoint_t x)
    {
        static const char *STRINGS[]={
                "END_OF_SENTENCE",
                "COMPLETION_TOKEN_LIMIT_REACHED",
                "STOP_PHRASE_ENCOUNTERED",
                "INTERRUPTED"
        };
        return STRINGS[x];
    }
    static void picollm_progress_callback(const char *token, void *context)
    {
        (void) context;
        // Use external event to interrupt generation (unused currently!)
        if(b_picollm_interrupt==true)
        {
            //pv_picollm_interrupt(picollm);
        }
        else
        {
            fprintf(stdout, "%s", token);
            fflush(stdout);
            if(num_tokens == -1)
            {
                gettimeofday(&tic, NULL);
            }
            num_tokens += 1;
        }
    }
    std::string PicoLLMPostProcessResponse(std::string s_resp_processed,pv_picollm_endpoint_t pv_endpoint)
    {
        // Remove "end of response key":
        std::string word_to_replace="<|eot_id|>";
        std::string replace_by="";
        size_t pos=s_resp_processed.find(word_to_replace);
        while(pos!=std::string::npos)
        {
            s_resp_processed.replace(pos,word_to_replace.size(),replace_by);
            pos=s_resp_processed.find(word_to_replace,
            pos + replace_by.size());
        }

        // Add "interrupted phrase" or "welcome phrase"
        if(pv_endpoint==PV_PICOLLM_ENDPOINT_COMPLETION_TOKEN_LIMIT_REACHED)
        {
            s_resp_processed=s_resp_processed+" And so on, but you got the idea! Its nice if you ask questions, never give up on learning!";
        }
        else
        {
            s_resp_processed=s_resp_processed+" Its nice if you ask questions, never give up on learning!";
        }

        return s_resp_processed;
    }
    std::string PicoLLMGetLatestResponse()
    {
        return m_s_picollm_response;
    }
    void PicoLLMInitialize(std::string s_model_name)
    {
        S_PICOLLM_MODEL_NAME=s_model_name;
        S_PICOLLM_MODEL_PATH=m_s_path_resources+S_PICOLLM_MODEL_NAME;
    }
    void PicoLLMStartProcessing(std::string s_prompt)
    {
        std::string s_prompt_modified="Please do not use more than 50 words for your answer to the following prompt and please do not use any special characters for your answer, besides of fullstops, commas, quotation marks, question marks, and exclamation marks: "+s_prompt;
        m_s_picollm_response="No prompt answer available.";

        ROS_WARN_STREAM("Picovoice PICOLLM - STARTED with generating response to prompt: " << s_prompt_modified);

        pv_picollm_t *picollm=NULL;
        b_picollm_interrupt=false; // Unused currently!
        num_tokens=-1;

        const char *access_key=S_PICOVOICE_ACCESS_KEY.c_str();
        const char *model_path=S_PICOLLM_MODEL_PATH.c_str();
        const char *device_string="best";
        float presence_penalty=F_PICOLLM_PRESENCE_PENALTY;//0.75f;
        float frequency_penalty=F_PICOLLM_FREQUENCY_PENALTY;//0.75f;
        int32_t seed=I_PICOLLM_SEED;//5;
        float top_p=F_PICOLLM_TOP_P;//0.9f;
        float temperature=F_PICOLLM_TEMPERATURE;//0.0f;
        int32_t max_output_tokens=I_PICOLLM_MAX_OUTPUT_TOKENS;//50;
        int32_t num_stop_phrases=0;
        const char **stop_phrases=NULL;
        int32_t num_top_choices=0;
        bool verbose=false;
        bool show_devices=false;
        const char *prompt=s_prompt_modified.c_str();

        fprintf(stdout, "picoLLM: '%s'\n", pv_picollm_version());

        char **message_stack=NULL;
        int32_t message_stack_depth=0;
        pv_status_t error_status=PV_STATUS_RUNTIME_ERROR;

        if(show_devices)
        {
            char **hardware_devices=NULL;
            int32_t num_hardware_devices=0;
            pv_status_t status=pv_picollm_list_hardware_devices(&hardware_devices, &num_hardware_devices);
            if(status!=PV_STATUS_SUCCESS)
            {
                fprintf(
                        stderr,
                        "Failed to list hardware devices with '%s'.\n",
                        pv_status_to_string(status));
                picollm_print_error_message(
                    message_stack,
                    message_stack_depth,
                    pv_get_error_stack,
                    pv_free_error_stack,
                    pv_status_to_string
                );
            }

            for(int32_t i=0; i<num_hardware_devices; i++)
            {
                fprintf(stdout, "%s\n", hardware_devices[i]);
            }
            pv_picollm_free_hardware_devices(hardware_devices, num_hardware_devices);
            return;
        }

        const int32_t max_top_choices=pv_picollm_max_top_choices();
        if(num_top_choices>max_top_choices)
        {
            fprintf(
                    stderr,
                    "Number of top choices must be less than or equal to %d.\n",
                    max_top_choices);
            return; //exit(EXIT_FAILURE);
        }

        pv_status_t status=pv_picollm_init(
                access_key,
                model_path,
                device_string,
                &picollm);
        if(status!=PV_STATUS_SUCCESS)
        {
            fprintf(
                    stderr,
                    "failed to init with '%s'",
                    pv_status_to_string(status));
            picollm_print_error_message(
                message_stack,
                message_stack_depth,
                pv_get_error_stack,
                pv_free_error_stack,
                pv_status_to_string
            );
        }

        int32_t context_length=0;
        status=pv_picollm_context_length(picollm, &context_length);
        if(status!=PV_STATUS_SUCCESS)
        {
            fprintf(
                    stderr,
                    "Failed to get context length with '%s'.\n",
                    pv_status_to_string(status));
            picollm_print_error_message(
                message_stack,
                message_stack_depth,
                pv_get_error_stack,
                pv_free_error_stack,
                pv_status_to_string
            );
        }

        if(max_output_tokens>context_length)
        {
            fprintf(
                    stderr,
                    "Max output tokens must be less than or equal to %d.\n",
                    context_length);
            return; //exit(EXIT_FAILURE);
        }

        const char *model=NULL;
        status=pv_picollm_model(picollm, &model);
        if(status!=PV_STATUS_SUCCESS)
        {
            fprintf(
                    stderr,
                    "Failed to get model with '%s'.\n",
                    pv_status_to_string(status));
            picollm_print_error_message(
                message_stack,
                message_stack_depth,
                pv_get_error_stack,
                pv_free_error_stack,
                pv_status_to_string
            );
        }

        fprintf(stdout, "Loaded model: '%s'\n", model);
        fprintf(stdout, "Generating... (press Ctrl+C to interrupt)\n");

        pv_picollm_usage_t usage;
        pv_picollm_endpoint_t endpoint;
        int32_t num_completion_tokens=0;
        pv_picollm_completion_token_t *completion_tokens=NULL;
        char *completion=NULL;
        ROS_WARN_STREAM("Picovoice PICOLLM - Starting to generate response...");
        status=pv_picollm_generate(
                picollm,
                prompt,
                max_output_tokens,
                stop_phrases,
                num_stop_phrases,
                seed,
                presence_penalty,
                frequency_penalty,
                temperature,
                top_p,
                num_top_choices,
                picollm_progress_callback,
                NULL,
                &usage,
                &endpoint,
                &completion_tokens,
                &num_completion_tokens,
                &completion);
        //free(prompt);
        //free(stop_phrases);
        if(status!=PV_STATUS_SUCCESS)
        {
            fprintf(
                    stderr,
                    "Failed to generate with '%s'.\n",
                    pv_status_to_string(status));
            picollm_print_error_message(
                message_stack,
                message_stack_depth,
                pv_get_error_stack,
                pv_free_error_stack,
                pv_status_to_string
            );
        }
        std::string s_completion_raw=completion;
        m_s_picollm_response=PicoLLMPostProcessResponse(s_completion_raw,endpoint);
        //fprintf(stdout, "\n");

        struct timeval toc;
        gettimeofday(&toc, NULL);

        fprintf(stdout,
                "Generated %.2f tokens per sec\n",
                (double) (usage.completion_tokens - 1) /
                ((double) (toc.tv_sec - tic.tv_sec) + (double) (toc.tv_usec - tic.tv_usec) / 1e6));

        if(verbose)
        {
            fprintf(stdout, "\nUsage:\n");
            fprintf(stdout, "  prompt tokens: %d\n", usage.prompt_tokens);
            fprintf(stdout, "  completion tokens: %d\n", usage.completion_tokens);

            fprintf(stdout, "endpoint: %s\n", picollm_endpoint_to_string(endpoint));
            fprintf(stdout, "completion-tokes:\n");
            for(int32_t i=0; i<num_completion_tokens; i++)
            {
                fprintf(stdout, "token: %-10s - log_prob: %3.1f\n", completion_tokens[i].token.token,completion_tokens[i].token.log_prob);
                if(num_top_choices>0)
                {
                    for(int32_t j=0; j<(num_top_choices - 1); j++)
                    {
                        fprintf(
                                stdout,
                                "  top-choice: %-10s - log_prob: %3.1f\n",
                                completion_tokens[i].top_choices[j].token,
                                completion_tokens[i].top_choices[j].log_prob);
                    }
                    fprintf(
                            stdout,
                            "  top-choice: %-10s - log_prob: %3.1f\n",
                            completion_tokens[i].top_choices[num_top_choices - 1].token,
                            completion_tokens[i].top_choices[num_top_choices - 1].log_prob);

                }
            }
            fprintf(stdout, "completion: %s\n", completion);
        }

        pv_picollm_delete_completion(completion);
        pv_picollm_delete_completion_tokens(completion_tokens, num_completion_tokens);
        pv_picollm_delete(picollm);

        ROS_WARN_STREAM("Picovoice - PICOLLM: FINISHED with generating response: " << m_s_picollm_response << std::endl << std::endl);
    }



    // ORCA - Streaming text-to-speech engine
    //========================================
    /*
    static double get_time()
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return (double) tv.tv_sec + ((double) tv.tv_usec * 1e-6);
    }
    static pv_status_t num_bytes_character(unsigned char c, int32_t *num_bytes)
    {
        *num_bytes=0;
        int32_t nb;
        if((c & 0x80) == 0x00) nb=1;
        else if((c & 0xE0) == 0xC0) nb=2;
        else if((c & 0xF0) == 0xE0) nb=3;
        else if((c & 0xF8) == 0xF0) nb=4;
        else return PV_STATUS_INVALID_ARGUMENT;
        *num_bytes=nb;
        return PV_STATUS_SUCCESS;
    }
    static pv_status_t pcm_chunk_init(int32_t num_samples,int16_t *pcm,pcm_chunk_t **chunk)
    {
        *chunk=NULL;
        pcm_chunk_t *c=(pcm_chunk_t*)calloc(1, sizeof(pcm_chunk_t));
        if(!c)
        {
            return PV_STATUS_OUT_OF_MEMORY;
        }
        c->pcm=pcm;
        c->num_samples=num_samples;
        c->next=NULL;
        *chunk=c;
        return PV_STATUS_SUCCESS;
    }
    static pv_status_t pcm_chunk_delete(pcm_chunk_t *chunk)
    {
        if(chunk)
        {
            free(chunk->pcm);
            free(chunk);
        }
        return PV_STATUS_SUCCESS;
    }
    void handle_error(
            char **message_stack,
            int32_t message_stack_depth,
            pv_status_t (*pv_get_error_stack_func)(char ***, int32_t *),
            void (*pv_free_error_stack_func)(char **),
            const char *(*pv_status_to_string_func)(pv_status_t))
    {
        pv_status_t error_status=pv_get_error_stack_func(&message_stack, &message_stack_depth);
        if(error_status!=PV_STATUS_SUCCESS)
        {
            fprintf(stderr, ".\nUnable to get Orca error state with '%s'\n", pv_status_to_string_func(error_status));
            return; //exit(EXIT_FAILURE);
        }
        if(message_stack_depth>0)
        {
            fprintf(stderr, ":\n");
            for(int32_t i=0; i<message_stack_depth; i++)
            {
                fprintf(stderr, "  [%d] %s\n", i, message_stack[i]);
            }
        }
        pv_free_error_stack_func(message_stack);
    }
    */
    std::string OrcaGetSynthesizedWAVFilePath()
    {
        return S_ORCA_WAV_FILE_PATH;
    }
    void OrcaInitialize(std::string s_model_name,
                        std::string s_wav_file_name)
    {
        S_ORCA_MODEL_NAME=s_model_name;
        S_ORCA_MODEL_PATH=m_s_path_resources+S_ORCA_MODEL_NAME;
        S_ORCA_WAV_FILE_NAME=s_wav_file_name;
        S_ORCA_WAV_FILE_PATH=m_s_path_resources+S_ORCA_WAV_FILE_NAME;
    }
    void OrcaSelectModel(std::string s_model_name)
    {
        S_ORCA_MODEL_NAME=s_model_name;
        S_ORCA_MODEL_PATH=m_s_path_resources+S_ORCA_MODEL_NAME;
    }

    void OrcaStartProcessing(std::string s_text_to_speech)
    {
        const char *access_key=S_PICOVOICE_ACCESS_KEY.c_str();
        const char *model_path=S_ORCA_MODEL_PATH.c_str();
        const char *text=s_text_to_speech.c_str();
        const char *output_path=S_ORCA_WAV_FILE_PATH.c_str();

        struct timeval before;
        gettimeofday(&before, NULL);

        char **message_stack = NULL;
        int32_t message_stack_depth = 0;
        pv_status_t error_status;

        pv_orca_t *orca = NULL;
        pv_status_t orca_status = pv_orca_init(access_key, model_path, &orca);
        if (orca_status != PV_STATUS_SUCCESS) {
            fprintf(stderr, "Failed to create an instance of Orca with '%s'", pv_status_to_string(orca_status));
            error_status = pv_get_error_stack(&message_stack, &message_stack_depth);
            if (error_status != PV_STATUS_SUCCESS) {
                fprintf(stderr, ".\nUnable to get Orca error state with '%s'.\n", pv_status_to_string(error_status));
                return; //exit(EXIT_FAILURE);
            }

            if (message_stack_depth > 0) {
                fprintf(stderr, ":\n");
                print_error_message(message_stack, message_stack_depth);
                pv_free_error_stack(message_stack);
            }
            return; //exit(EXIT_FAILURE);
        }

        struct timeval after;
        gettimeofday(&after, NULL);

        double init_sec =
                ((double) (after.tv_sec - before.tv_sec) +
                 ((double) (after.tv_usec - before.tv_usec)) * 1e-6);
        fprintf(stdout, "\nInitialized Orca in %.1f sec\n", init_sec);

        pv_orca_synthesize_params_t *synthesize_params = NULL;
        pv_status_t synthesize_params_status = pv_orca_synthesize_params_init(&synthesize_params);
        if (synthesize_params_status != PV_STATUS_SUCCESS) {
            fprintf(
                    stderr,
                    "Failed to create an instance of Orca synthesize params with '%s'",
                    pv_status_to_string(synthesize_params_status));
            error_status = pv_get_error_stack(&message_stack, &message_stack_depth);
            if (error_status != PV_STATUS_SUCCESS) {
                fprintf(
                        stderr,
                        ".\nUnable to get Orca synthesize params error state with '%s'.\n",
                        pv_status_to_string(error_status));
                return; //exit(EXIT_FAILURE);
            }

            if (message_stack_depth > 0) {
                fprintf(stderr, ":\n");
                print_error_message(message_stack, message_stack_depth);
                pv_free_error_stack(message_stack);
            }
            return; //exit(EXIT_FAILURE);
        }

        double proc_sec = 0.;
        gettimeofday(&before, NULL);

        fprintf(stdout, "Synthesizing text '%s'\n", text);

        int32_t num_alignments = 0;
        pv_orca_word_alignment_t **alignments = NULL;
        pv_status_t synthesize_status = pv_orca_synthesize_to_file(
                orca,
                text,
                synthesize_params,
                output_path,
                &num_alignments,
                &alignments);
        if (synthesize_status != PV_STATUS_SUCCESS) {
            fprintf(
                    stderr,
                    "Failed to synthesize text with '%s'",
                    pv_status_to_string(synthesize_params_status));
            error_status = pv_get_error_stack(&message_stack, &message_stack_depth);
            if (error_status != PV_STATUS_SUCCESS) {
                fprintf(
                        stderr,
                        ".\nUnable to get Orca synthesize error state with '%s'.\n",
                        pv_status_to_string(error_status));
                return; //exit(EXIT_FAILURE);
            }

            if (message_stack_depth > 0) {
                fprintf(stderr, ":\n");
                print_error_message(message_stack, message_stack_depth);
                pv_free_error_stack(message_stack);
            }
            return; //exit(EXIT_FAILURE);
        }

        gettimeofday(&after, NULL);

        proc_sec +=
                ((double) (after.tv_sec - before.tv_sec) +
                 ((double) (after.tv_usec - before.tv_usec)) * 1e-6);

        /*
        if (num_alignments > 0) {
            fprintf(stdout, "\nWord alignments");
            if (num_alignments > 3) {
                fprintf(stdout, " (only showing first 3):\n");
            } else {
                fprintf(stdout, ":\n");
            }
            int32_t num_alignments_shown = num_alignments > 3 ? 3 : num_alignments;
            for (int32_t i = 0; i < num_alignments_shown; i++) {
                fprintf(
                        stdout,
                        "word=\"%s\", start_sec=%.2f, end_sec=%.2f\n",
                        alignments[i]->word,
                        alignments[i]->start_sec,
                        alignments[i]->end_sec);
                for (int32_t j = 0; j < alignments[i]->num_phonemes; j++) {
                    fprintf(
                            stdout,
                            "\tphoneme=\"%s\", start_sec=%.2f, end_sec=%.2f\n",
                            alignments[i]->phonemes[j]->phoneme,
                            alignments[i]->phonemes[j]->start_sec,
                            alignments[i]->phonemes[j]->end_sec);
                }
            }
        }
        */

        fprintf(stdout, "Synthesized text in %.2f sec\n", proc_sec);
        fprintf(stdout, "Saved audio to '%s'\n", output_path);

        pv_status_t delete_status = pv_orca_word_alignments_delete(num_alignments, alignments);
        if (delete_status != PV_STATUS_SUCCESS) {
            fprintf(stderr, "Failed to delete word alignments with '%s'.\n", pv_status_to_string(delete_status));
            return; //exit(EXIT_FAILURE);
        }

        pv_orca_synthesize_params_delete(synthesize_params);
        pv_orca_delete(orca);
    }

    /* Deprecated with Dr. WAV library
    void OrcaStartProcessingDrWav(std::string s_text_to_speech)
    {
        ROS_WARN_STREAM("Picovoice - ORCA: STARTED with streaming text-to-speech (*.wav file)...");

        const char *access_key=S_PICOVOICE_ACCESS_KEY.c_str();
        const char *model_path=S_ORCA_MODEL_PATH.c_str();
        const char *text=s_text_to_speech.c_str();
        const char *output_path=S_ORCA_WAV_FILE_PATH.c_str();

        char **message_stack=NULL;
        int32_t message_stack_depth=0;

        fprintf(stdout, "Orca version: %s\n\n", pv_orca_version());

        double time_before_init=get_time();

        pv_orca_t *orca=NULL;
        pv_status_t orca_status=pv_orca_init(access_key, model_path, &orca);
        if(orca_status!=PV_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to create an instance of Orca with '%s'", pv_status_to_string(orca_status));
            handle_error(
                    message_stack,
                    message_stack_depth,
                    pv_get_error_stack,
                    pv_free_error_stack,
                    pv_status_to_string);
            return; //exit(EXIT_FAILURE);
        }

        double init_sec=get_time() - time_before_init;
        fprintf(stdout, "Initialized Orca in %.1f sec\n", init_sec);

        int32_t sample_rate=0;
        pv_status_t status=pv_orca_sample_rate(orca, &sample_rate);
        if(status!=PV_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to get Orca sample rate with '%s'", pv_status_to_string(status));
            handle_error(
                    message_stack,
                    message_stack_depth,
                    pv_get_error_stack,
                    pv_free_error_stack,
                    pv_status_to_string);
            return; //exit(EXIT_FAILURE);
        }

        drwav_data_format format;
        format.container=drwav_container_riff;
        format.format=DR_WAVE_FORMAT_PCM;
        format.channels=1;
        format.sampleRate=sample_rate;
        format.bitsPerSample=16;

        drwav output_file;
        unsigned int drwav_init_file_status=drwav_init_file_write(&output_file, output_path, &format, NULL);
        if(!drwav_init_file_status)
        {
            fprintf(stderr, "Failed to open the output wav file at '%s'.", output_path);
            return; //exit(EXIT_FAILURE);
        }

        pv_orca_synthesize_params_t *synthesize_params=NULL;
        pv_status_t synthesize_params_status=pv_orca_synthesize_params_init(&synthesize_params);
        if(synthesize_params_status!=PV_STATUS_SUCCESS)
        {
            fprintf(
                    stderr,
                    "Failed to create an instance of Orca synthesize params with '%s'",
                    pv_status_to_string(synthesize_params_status));
            handle_error(
                    message_stack,
                    message_stack_depth,
                    pv_get_error_stack,
                    pv_free_error_stack,
                    pv_status_to_string);
            return; //exit(EXIT_FAILURE);
        }

        fprintf(stdout, "\nSynthesizing text '%s' \n", text);

        int32_t num_samples_chunks[MAX_NUM_CHUNKS]={0};
        double start_chunks[MAX_NUM_CHUNKS]={0};
        start_chunks[0]=get_time();
        double end_chunks[MAX_NUM_CHUNKS]={0};
        int32_t num_chunks=0;

        pcm_chunk_t *pcm_chunk_prev=NULL;
        pcm_chunk_t *pcm_chunk_head=NULL;

        pv_orca_stream_t *orca_stream=NULL;
        pv_status_t stream_open_status=pv_orca_stream_open(orca, synthesize_params, &orca_stream);
        if(stream_open_status!=PV_STATUS_SUCCESS)
        {
            fprintf(stderr, "Error opening stream");
            handle_error(
                    message_stack,
                    message_stack_depth,
                    pv_get_error_stack,
                    pv_free_error_stack,
                    pv_status_to_string);
            return; //exit(EXIT_FAILURE);
        }

        char character[MAX_NUM_BYTES_PER_CHARACTER]={0};
        int32_t i=0;
        while(i<(int32_t) strlen(text))
        {
            if(num_chunks>(MAX_NUM_CHUNKS - 1))
            {
                fprintf(stderr, "Trying to synthesize too many chunks. Only '%d' chunks are supported.\n", MAX_NUM_CHUNKS);
                return; //exit(EXIT_FAILURE);
            }

            int32_t num_bytes=0;
            status=num_bytes_character((unsigned char) text[i], &num_bytes);
            if(status!=PV_STATUS_SUCCESS)
            {
                fprintf(stderr, "Error getting number of bytes for character: '%c'", text[i]);
                return; //exit(EXIT_FAILURE);
            }

            for(int32_t j=0; j<num_bytes; j++)
            {
                character[j]=text[i + j];
            }
            character[num_bytes]='\0';

            int32_t num_samples_chunk=0;
            int16_t *pcm_chunk=NULL;
            // pv_orca_synthesize_to_file()
            status=pv_orca_stream_synthesize(orca_stream, character, &num_samples_chunk, &pcm_chunk);
            if(status!=PV_STATUS_SUCCESS)
            {
                fprintf(stderr, "Error adding token: '%s'", character);
                handle_error(
                        message_stack,
                        message_stack_depth,
                        pv_get_error_stack,
                        pv_free_error_stack,
                        pv_status_to_string);
                return; //exit(EXIT_FAILURE);
            }

            if(num_samples_chunk>0)
            {
                if(pcm_chunk_prev == NULL)
                {
                    pcm_chunk_init(num_samples_chunk, pcm_chunk, &pcm_chunk_prev);
                    pcm_chunk_head=pcm_chunk_prev;
                }
                else
                {
                    pcm_chunk_init(num_samples_chunk, pcm_chunk, &(pcm_chunk_prev->next));
                    pcm_chunk_prev=pcm_chunk_prev->next;
                }

                double timestamp=get_time();
                num_samples_chunks[num_chunks]=num_samples_chunk;
                end_chunks[num_chunks++]=timestamp;
                start_chunks[num_chunks]=timestamp;
            }
            i += num_bytes;
        }

        int32_t num_samples_chunk=0;
        int16_t *pcm_chunk=NULL;
        status=pv_orca_stream_flush(orca_stream, &num_samples_chunk, &pcm_chunk);
        if(status!=PV_STATUS_SUCCESS)
        {
            fprintf(stderr, "Error flushing Orca stream");
            handle_error(
                    message_stack,
                    message_stack_depth,
                    pv_get_error_stack,
                    pv_free_error_stack,
                    pv_status_to_string);
            return; //exit(EXIT_FAILURE);
        }

        if(num_samples_chunk>0)
        {
            if(pcm_chunk_prev == NULL)
            {
                pcm_chunk_init(num_samples_chunk, pcm_chunk, &pcm_chunk_prev);
                pcm_chunk_head=pcm_chunk_prev;
            }
            else
            {
                pcm_chunk_init(num_samples_chunk, pcm_chunk, &(pcm_chunk_prev->next));
            }

            double timestamp=get_time();
            num_samples_chunks[num_chunks]=num_samples_chunk;
            end_chunks[num_chunks++]=timestamp;
            start_chunks[num_chunks]=timestamp;
        }

        pv_orca_stream_close(orca_stream);
        pv_orca_synthesize_params_delete(synthesize_params);
        pv_orca_delete(orca);

        // *.WAV FILE Output
        int32_t num_samples=0;
        pcm_chunk_t *pcm_chunk_iter=pcm_chunk_head;
        while(pcm_chunk_iter!=NULL)
        {
            num_samples += pcm_chunk_iter->num_samples;
            pcm_chunk_iter=pcm_chunk_iter->next;
        }

        int16_t *pcm=(int16_t*)malloc(num_samples * sizeof(int16_t));
        int32_t offset=0;
        pcm_chunk_iter=pcm_chunk_head;
        while(pcm_chunk_iter!=NULL)
        {
            memcpy(&pcm[offset], pcm_chunk_iter->pcm, pcm_chunk_iter->num_samples * sizeof(int16_t));
            offset += pcm_chunk_iter->num_samples;
            pcm_chunk_iter=pcm_chunk_iter->next;
        }

        pcm_chunk_iter=pcm_chunk_head;
        while(pcm_chunk_iter!=NULL)
        {
            pcm_chunk_t *tmp=pcm_chunk_iter;
            pcm_chunk_iter=pcm_chunk_iter->next;
            pcm_chunk_delete(tmp);
        }

        if((int32_t) drwav_write_pcm_frames(&output_file, num_samples, pcm)!=num_samples)
        {
            fprintf(stderr, "Failed to write to output file.\n");
            return; //exit(EXIT_FAILURE);
        }

        drwav_uninit(&output_file);
        free(pcm);

        fprintf(
                stdout,
                "\nGenerated %d audio chunk%s in %.2f seconds.\n",
                num_chunks, num_chunks == 1 ? "" : "s",
                end_chunks[num_chunks - 1] - start_chunks[0]);

        // for(int32_t i=0; i<num_chunks; i++)
        // {
        //     float num_seconds=(float) num_samples_chunks[i] / (float) sample_rate;
        //     double process_time=end_chunks[i] - start_chunks[i];
        //     fprintf(stdout,"Audio chunk #%d: length: %.2f s, processing time %.2f s\n",i,num_seconds,process_time);
        // }

        fprintf(stdout, "\nSaved final audio to '%s' and starting pv_speaker...\n", output_path);

        PVSpeakerStartProcessing(output_path);

        ROS_WARN_STREAM("Picovoice - ORCA: FINISHED with streaming text-to-speech (*.wav file)!" << std::endl << std::endl);
    }



    // PVSPEAKER - Cross-platform audio player
    //=========================================
    void *read_wav_file(const char *filename,
                        uint32_t *num_samples,
                        uint32_t *sample_rate,
                        uint16_t *bits_per_sample)
    {
        FILE *file=fopen(filename, "rb");
        if(!file)
        {
            perror("Unable to open file");
            return NULL;
        }

        wav_header header;

        fread(&header, sizeof(header), 1, file);

        if(header.chunk_id[0]!='R' || header.chunk_id[1]!='I' || header.chunk_id[2]!='F' || header.chunk_id[3]!='F' ||
            header.format[0]!='W' || header.format[1]!='A' || header.format[2]!='V' || header.format[3]!='E')
        {
            fclose(file);
            fprintf(stderr, "Invalid WAV file\n");
            return NULL;
        }

        if(header.audio_format!=1)
        {
            fclose(file);
            fprintf(stderr, "WAV file format must be PCM type\n");
            return NULL;
        }

        if(header.num_channels!=1)
        {
            fclose(file);
            fprintf(stderr, "WAV file must have a single channel (MONO)\n");
            return NULL;
        }

        *sample_rate=header.sample_rate;
        *bits_per_sample=header.bits_per_sample;
        uint32_t bytes_per_sample=header.bits_per_sample / 8;
        *num_samples=header.subchunk2_size / bytes_per_sample;

        void *pcm_data=malloc(header.subchunk2_size);

        if(!pcm_data)
        {
            perror("Memory allocation failed");
            fclose(file);
            return NULL;
        }

        fread(pcm_data, header.subchunk2_size, 1, file);

        fclose(file);

        return pcm_data;
    }
    void PVSpeakerStartProcessing(std::string s_input_file)
    {
        ROS_WARN_STREAM("Picovoice - PVSPEAKER: STARTED with playing audio (*.wav file)...");
        pv_speaker_t *speaker=NULL;

        const char *input_wav_path=s_input_file.c_str();
        int32_t device_index=-1;
        int32_t buffer_size_secs=20;
        const char *output_wav_path=NULL;

        fprintf(stdout, "pv_speaker version: %s\n", pv_speaker_version());

        uint32_t num_samples, sample_rate;
        uint16_t bits_per_sample;
        void *pcm_data=read_wav_file(input_wav_path, &num_samples, &sample_rate, &bits_per_sample);

        fprintf(stdout, "Initializing pv_speaker...\n");
        pv_speaker_status_t status=pv_speaker_init(
                sample_rate,
                bits_per_sample,
                buffer_size_secs,
                device_index,
                &speaker);
        if(status!=PV_SPEAKER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to initialize device with %s.\n", pv_speaker_status_to_string(status));
            exit(1);
        }

        const char *selected_device=pv_speaker_get_selected_device(speaker);
        fprintf(stdout, "Selected device: %s.\n", selected_device);

        if(output_wav_path!=NULL)
        {
            pv_speaker_write_to_file(speaker, output_wav_path);
        }

        status=pv_speaker_start(speaker);
        if(status!=PV_SPEAKER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to start device with %s.\n", pv_speaker_status_to_string(status));
            exit(1);
        }

        fprintf(stdout, "Playing audio...\n");
        if(pcm_data)
        {
            int8_t *pcm=(int8_t *) pcm_data;
            int32_t total_written_length=0;

            while (total_written_length<num_samples)
            {
                int32_t written_length=0;
                status=pv_speaker_write(
                        speaker,
                        &pcm[total_written_length * bits_per_sample / 8],
                        num_samples - total_written_length,
                        &written_length);
                if(status!=PV_SPEAKER_STATUS_SUCCESS)
                {
                    fprintf(stderr, "Failed to write with %s.\n", pv_speaker_status_to_string(status));
                    exit(1);
                }
                total_written_length += written_length;
            }

            free(pcm);
        }

        fprintf(stdout, "Waiting for audio to finish...\n");
        int8_t *pcm=NULL;
        int32_t pcm_length=0;
        int32_t written_length=0;
        status=pv_speaker_flush(speaker, pcm, pcm_length, &written_length);
        if(status!=PV_SPEAKER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to flush pcm with %s.\n", pv_speaker_status_to_string(status));
            exit(1);
        }

        fprintf(stdout, "Finished playing audio...\n");
        status=pv_speaker_stop(speaker);
        if(status!=PV_SPEAKER_STATUS_SUCCESS)
        {
            fprintf(stderr, "Failed to stop device with %s.\n", pv_speaker_status_to_string(status));
            exit(1);
        }

        fprintf(stdout, "Deleting pv_speaker...\n");
        pv_speaker_delete(speaker);

        ROS_WARN_STREAM("Picovoice - PVSPEAKER: FINISHED with playing audio (*.wav file)!" << std::endl << std::endl);
    }
    */
};
