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

#include <picovoice_msgs/GetTranscriptAction.h>
#include <ros/init.h>

#include "./cheetah_recognizer.h"
#include "./recognizer_node.h"

namespace picovoice_driver
{
    using namespace picovoice_msgs;
    class CheetahNode : public RecognizerNode<CheetahRecognizerData,CheetahRecognizer,GetTranscriptAction>
    {
    public:
        CheetahNode(const CheetahRecognizerData::Parameters& parameters)
            : RecognizerNode("cheetah","get_transcript",parameters)
        {
        }

    private:
        void updateParameters(const GetTranscriptGoal& goal,CheetahRecognizerData::Parameters& parameters) override
        {
            parameters.enable_automatic_punctuation=goal.enable_automatic_punctuation;
        }

        void updateResult(const CheetahRecognizerData::Result& result, GetTranscriptResult& action_result) override
        {
            action_result.transcript=result.transcript_;
        }
        std::string contexts_directory_url_;
    };
} // namespace picovoice_driver

int main(int argc, char** argv)
{
    using namespace picovoice_driver;

    ros::init(argc, argv, "cheetah");

    ros::NodeHandle local_nh("~");
    auto model_url = local_nh.param("model_url", defaultResourceUrl() + "/models/cheetah_params.pv");

    try
    {
        CheetahRecognizerData::Parameters parameters;
        parameters.model_path_ = pathFromUrl(model_url);
        CheetahNode node(parameters);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("CheetahNode exception: %s", e.what());
        return 1;
    }
    return 0;
}
