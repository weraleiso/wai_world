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

#include <picovoice_msgs/GetLLMResponseAction.h>
#include <ros/init.h>

#include "./picollm_recognizer.h"
#include "./recognizer_node.h"

namespace picovoice_driver
{
    using namespace picovoice_msgs;
    class PicoLLMNode : public RecognizerNode<PicoLLMRecognizerData,PicoLLMRecognizer,GetLLMResponseAction>
    {
    public:
        PicoLLMNode(const PicoLLMRecognizerData::Parameters& parameters)
            : RecognizerNode("picollm","get_llm_response",parameters)
        {
        }

    private:
        void updateParameters(const GetLLMResponseGoal& goal,PicoLLMRecognizerData::Parameters& parameters) override
        {
            parameters.prompt_=goal.prompt;
        }

        void updateResult(const PicoLLMRecognizerData::Result& result,GetLLMResponseResult& action_result) override
        {
            action_result.llm_response=result.llm_response_;
        }
        std::string contexts_directory_url_;
    };
} // namespace picovoice_driver

int main(int argc, char** argv)
{
    using namespace picovoice_driver;

    ros::init(argc, argv, "picollm");

    ros::NodeHandle local_nh("~");
    auto model_url = local_nh.param("model_url", defaultHomeUrl()+"/catkin_ws/src/wai_world/wai_world_tools/picovoice_ros/picovoice_driver/resources/models/llama-2-7b-chat-342.pllm");

    try
    {
        PicoLLMRecognizerData::Parameters parameters;
        parameters.model_path_ = pathFromUrl(model_url);
        PicoLLMNode node(parameters);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("PicoLLMNode exception: %s", e.what());
        return 1;
    }
    return 0;
}
