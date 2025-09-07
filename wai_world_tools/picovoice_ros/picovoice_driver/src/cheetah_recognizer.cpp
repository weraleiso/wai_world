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

#include "./cheetah_recognizer.h"

namespace picovoice_driver
{
    std::ostream& operator<<(std::ostream& os, const CheetahRecognizerData::Parameters& p)
    {
        os << "Parameters(access_key=" << p.access_key_
           << ", model_path=" << p.model_path_
           << ", endpoint_duration_sec_=" << p.endpoint_duration_sec_
           << ", enable_automatic_punctuation=" << p.enable_automatic_punctuation << ")";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const CheetahRecognizerData::Result& r)
    {
        os << "Result(transcript=" << r.transcript_ << ")";
        return os;
    }

    CheetahRecognizer::~CheetahRecognizer()
    {
        if (cheetah_ != NULL)
        {
            pv_cheetah_delete(cheetah_);
        }
    }

    void CheetahRecognizer::configure(const CheetahRecognizerData::Parameters& parameters)
    {
      if (cheetah_ != NULL)
      {
        pv_cheetah_delete(cheetah_);
      }

      pv_status_t status =
          pv_cheetah_init(
                  parameters.access_key_.data(),
                  parameters.model_path_.data(),
                  static_cast<float>(parameters.endpoint_duration_sec_),
                  parameters.enable_automatic_punctuation,
                  &cheetah_);
      if (status != PV_STATUS_SUCCESS)
      {
        throw std::runtime_error("Failed to initialize picovoice cheetah with parameters " + toString(parameters) + ": " +
                                 std::string(pv_status_to_string(status)));
      }
    }

    CheetahRecognizerData::Result CheetahRecognizer::getResult()
    {
        return result_;
    }

    Recognizer::RecordSettings CheetahRecognizer::getRecordSettings()
    {
      Recognizer::RecordSettings settings;
      settings.audio_device_access=-1; // Audio device IN (record) access
      settings.frame_length_ = pv_cheetah_frame_length();
      settings.sample_rate_ = pv_sample_rate();
      return settings;
    }

    void CheetahRecognizer::recognizeInit()
    {
      result_ = CheetahRecognizerData::Result();
      //pv_cheetah_flush(cheetah_,&full_transcript);
      sst_full_transcript.str("");
    }

    bool CheetahRecognizer::recognizeProcess(int16_t* frames)
    {
        bool is_endpoint = false;
        char* partial_transcript = NULL;
        auto process_status = pv_cheetah_process(cheetah_,frames,&partial_transcript,&is_endpoint);
        if(process_status != PV_STATUS_SUCCESS)
        {
            throw std::runtime_error("Cheetah process failed: " + std::string(pv_status_to_string(process_status)));
            return false;
        }
        sst_full_transcript << partial_transcript;
        free(partial_transcript);

        if(is_endpoint)
        {
            char* final_transcript = NULL;
            const pv_status_t status = pv_cheetah_flush(cheetah_,&final_transcript);
            if(status != PV_STATUS_SUCCESS)
            {
                throw std::runtime_error("Cheetah process failed: " + std::string(pv_status_to_string(process_status)));
                return false;
            }
            // do something with transcript
            sst_full_transcript << final_transcript;
            result_.transcript_=sst_full_transcript.str();
            free(final_transcript);
            return true;
        }

        return false;
    }
}  // namespace picovoice_driver
