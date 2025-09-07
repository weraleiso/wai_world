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

#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "./picollm_recognizer.h"

namespace picovoice_driver
{
    std::ostream& operator<<(std::ostream& os, const PicoLLMRecognizerData::Parameters& p)
    {
        os << "Parameters(access_key=" << p.access_key_
           << ", model_path=" << p.model_path_
           << ", prompt=" << p.prompt_ << ")";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const PicoLLMRecognizerData::Result& r)
    {
        os << "Result(llm_response=" << r.llm_response_ << ")";
        return os;
    }

    PicoLLMRecognizer::~PicoLLMRecognizer()
    {
        if (picollm_ != NULL)
        {
            pv_picollm_delete(picollm_);
        }
    }

    void PicoLLMRecognizer::configure(const PicoLLMRecognizerData::Parameters& parameters)
    {
        if (picollm_ != NULL)
        {
            pv_picollm_delete(picollm_);
        }

        pv_status_t status =
        pv_picollm_init(
              parameters.access_key_.data(),
              parameters.model_path_.data(),
              "best",
              &picollm_);
        if (status != PV_STATUS_SUCCESS)
        {
            throw std::runtime_error("Failed to initialize picovoice PicoLLM with parameters " + toString(parameters) + ": " +
                             std::string(pv_status_to_string(status)));
        }

        s_prompt=parameters.prompt_;
    }

    PicoLLMRecognizerData::Result PicoLLMRecognizer::getResult()
    {
        return result_;
    }

    Recognizer::RecordSettings PicoLLMRecognizer::getRecordSettings()
    {
      Recognizer::RecordSettings settings;
      settings.audio_device_access=0; // Set no audio device mode
      // settings.frame_length_ = pv_cheetah_frame_length();
      // settings.sample_rate_ = pv_sample_rate();
      return settings;
    }

    void PicoLLMRecognizer::recognizeInit()
    {
      result_ = PicoLLMRecognizerData::Result();
      s_llm_response="";
    }

    bool PicoLLMRecognizer::recognizeProcess(int16_t* frames)
    {
        pv_picollm_usage_t usage;
        pv_picollm_endpoint_t endpoint;
        int32_t num_completion_tokens;
        pv_picollm_completion_token_t *completion_tokens;
        char *output;
        pv_picollm_generate(
            picollm_,
            s_prompt.c_str(),
            -1,    // completion_token_limit
            NULL,  // stop_phrases
            0,     // num_stop_phrases
            -1,    // seed
            0.f,   // presence_penalty
            0.f,   // frequency_penalty
            0.f,   // temperature
            1.f,   // top_p
            0,     // num_top_choices
            NULL,  // stream_callback
            NULL,  // stream_callback_context
            &usage,
            &endpoint,
            &completion_tokens,
            &num_completion_tokens,
            &output);
        s_llm_response=output;
        result_.llm_response_=s_llm_response;
        return true;
    }
}  // namespace picovoice_driver
