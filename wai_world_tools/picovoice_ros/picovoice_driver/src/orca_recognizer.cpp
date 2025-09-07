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

#include "./orca_recognizer.h"

namespace picovoice_driver
{
    std::ostream& operator<<(std::ostream& os, const OrcaRecognizerData::Parameters& p)
    {
        os << "Parameters(access_key=" << p.access_key_
           << ", model_path=" << p.model_path_
           << ", transcript="<< p.transcript_
           << ", male_or_female_voice="<< p.male_or_female_voice_
           << ", synthesize_to_wav="<< p.synthesize_to_wav_
           << ", synthesize_to_wav_path=" << p.synthesize_to_wav_path_ << ")";
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const OrcaRecognizerData::Result& r)
    {
        os << "Result(synthesized_success=" << r.synthesized_success_ << ")";
        return os;
    }

    OrcaRecognizer::~OrcaRecognizer()
    {
        if(synthesize_params_!=NULL)
        {
            pv_orca_synthesize_params_delete(synthesize_params_);
        }
        if (orca_!=NULL)
        {
            pv_orca_delete(orca_);
        }
    }

    void OrcaRecognizer::configure(const OrcaRecognizerData::Parameters& parameters)
    {
        if(synthesize_params_!=NULL)
        {
            pv_orca_synthesize_params_delete(synthesize_params_);
        }
        if (orca_!=NULL)
        {
            pv_orca_delete(orca_);
        }

        pv_status_t orca_status_init=pv_orca_init(
                  parameters.access_key_.data(),
                  parameters.model_path_.data(),
                  &orca_);
        if(orca_status_init!=PV_STATUS_SUCCESS)
        {
            throw std::runtime_error("Failed to initialize picovoice orca with parameters "+
                                     toString(parameters)+": "+std::string(pv_status_to_string(orca_status_init)));
        }

        s_transcript_=parameters.transcript_;
        b_male_or_female_voice_=parameters.male_or_female_voice_;
        b_synthesize_to_wav_=parameters.synthesize_to_wav_;
        s_synthesize_to_wav_path_=parameters.synthesize_to_wav_path_;
    }

    OrcaRecognizerData::Result OrcaRecognizer::getResult()
    {
        return result_;
    }

    Recognizer::RecordSettings OrcaRecognizer::getRecordSettings()
    {
      Recognizer::RecordSettings settings;
      settings.audio_device_access=1; // ORCA: Audio device OUT ("speaker" setting...)
      settings.frame_length_ = 1; // ORCA: Unused currently!
      settings.sample_rate_ = 512; // ORCA: Unused currently!
      return settings;
    }

    void OrcaRecognizer::recognizeInit()
    {
      result_ = OrcaRecognizerData::Result();
      b_synthesized_success_=false;
    }

    bool OrcaRecognizer::recognizeProcess(int16_t* frames)
    {
        pv_status_t orca_status_synthesize_params=pv_orca_synthesize_params_init(&synthesize_params_);
        if(orca_status_synthesize_params!=PV_STATUS_SUCCESS)
        {
            throw std::runtime_error("Failed to initialize picovoice orca synthesize parameters"+
                                     toString(synthesize_params_)+": "+std::string(pv_status_to_string(orca_status_synthesize_params)));
            result_.synthesized_success_=false;
            return result_.synthesized_success_;
        }

        int32_t num_alignments = 0;
        pv_orca_word_alignment_t **alignments = NULL;
        pv_status_t orca_status_synthesizer=pv_orca_synthesize_to_file(
                orca_,
                s_transcript_.c_str(),
                synthesize_params_,
                s_synthesize_to_wav_path_.c_str(),
                &num_alignments,
                &alignments);
        if(orca_status_synthesizer!=PV_STATUS_SUCCESS)
        {
            throw std::runtime_error("Failed to initialize picovoice orca synthetizatioan to .WAV file"+
                                     toString(synthesize_params_)+": "+std::string(pv_status_to_string(orca_status_synthesizer)));
            result_.synthesized_success_=false;
        }

        pv_status_t orca_status_delete = pv_orca_word_alignments_delete(num_alignments, alignments);
        if (orca_status_delete != PV_STATUS_SUCCESS) {
            fprintf(stderr, "Failed to delete word alignments with `%s`.\n", pv_status_to_string(orca_status_delete));
            exit(EXIT_FAILURE);
        }

        result_.synthesized_success_=true;
        return result_.synthesized_success_;
    }
}  // namespace picovoice_driver
