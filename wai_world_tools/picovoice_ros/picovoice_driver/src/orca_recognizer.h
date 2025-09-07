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

#pragma once

#include <map>
#include <ostream>
#include <picovoice.h>
#include <pv_orca.h>
#include <string>
#include <vector>

#include "./recognizer.h"
#include "./util.h"

namespace picovoice_driver
{
    struct OrcaRecognizerData
    {
        struct Parameters : RecognizerData::Parameters
        {
            double sensitivity_=0.5; // Unsued currently!

            std::string transcript_="Who are you?";
            bool male_or_female_voice_=false; // Synthesize with male (false) or female (true) voice.
            bool synthesize_to_wav_=true;
            std::string synthesize_to_wav_path_="/home/ias/pv_orca_sythesized.wav";
        };

        struct Result
        {
            //!
            //! \brief intent_ The recognized intent
            //!
            bool synthesized_success_;
        };
    };
    std::ostream& operator<<(std::ostream& os, const OrcaRecognizerData::Parameters& p);
    std::ostream& operator<<(std::ostream& os, const OrcaRecognizerData::Result& r);

    class OrcaRecognizer : public RecognizerT<OrcaRecognizerData>
    {
    public:
        ~OrcaRecognizer();
        void configure(const OrcaRecognizerData::Parameters& parameters) override;
        OrcaRecognizerData::Result getResult() override;

    private:
        RecordSettings getRecordSettings() override;
        void recognizeInit() override;
        bool recognizeProcess(int16_t* frames) override;

        std::string s_transcript_;
        bool b_male_or_female_voice_;
        bool b_synthesize_to_wav_;
        std::string s_synthesize_to_wav_path_;
        bool b_synthesized_success_;
        OrcaRecognizerData::Result result_;
        pv_orca_t* orca_=NULL;
        pv_orca_synthesize_params_t* synthesize_params_=NULL;
    };
}  // namespace picovoice_driver
