/*
 * Copyright 2021, Rein Appeldoorn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <picovoice_msgs/GetSynthetizationAction.h>
#include <ros/init.h>

#include "./orca_recognizer.h"
#include "./recognizer_node.h"

namespace picovoice_driver
{
    using namespace picovoice_msgs;
    class OrcaNode : public RecognizerNode<OrcaRecognizerData,OrcaRecognizer,GetSynthetizationAction>
    {
    public:
        OrcaNode(const OrcaRecognizerData::Parameters& parameters)
            : RecognizerNode("orca","get_synthetization",parameters)
        {
        }

    private:
        void updateParameters(const GetSynthetizationGoal& goal,OrcaRecognizerData::Parameters& parameters) override
        {
            parameters.transcript_=goal.transcript;
            parameters.male_or_female_voice_=goal.male_or_female_voice;
            if(parameters.male_or_female_voice_==true)
            {
                parameters.model_path_=pathFromUrl(defaultResourceUrl()+"/models/orca_params_female.pv");
            }
            else
            {
                parameters.model_path_=pathFromUrl(defaultResourceUrl()+"/models/orca_params_male.pv");
            }
            parameters.synthesize_to_wav_=goal.synthesize_to_wav;
            parameters.synthesize_to_wav_path_=goal.synthesize_to_wav_path;
        }

        void updateResult(const OrcaRecognizerData::Result& result, GetSynthetizationResult& action_result) override
        {
            action_result.synthesized_success=result.synthesized_success_;
        }
        std::string contexts_directory_url_;
    };
} // namespace picovoice_driver

int main(int argc, char** argv)
{
    using namespace picovoice_driver;

    ros::init(argc, argv, "orca");

    ros::NodeHandle local_nh("~");
    auto model_url = local_nh.param("model_url", defaultResourceUrl() + "/models/orca_params_male.pv");

    try
    {
        OrcaRecognizerData::Parameters parameters;
        parameters.model_path_ = pathFromUrl(model_url);
        OrcaNode node(parameters);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("OrcaNode exception: %s", e.what());
        return 1;
    }
    return 0;
}
