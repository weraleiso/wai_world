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
#include <pv_picollm.h>
#include <string>
#include <vector>

#include "./recognizer.h"
#include "./util.h"

namespace picovoice_driver
{
    struct PicoLLMRecognizerData
    {
        struct Parameters : RecognizerData::Parameters
        {
            bool require_endpoint_=false; // Unsued currently!
            std::string prompt_="Who are you?";
            double sensitivity_ = 0.5; // Unsued currently!
        };

        struct Result
        {
            //!
            //! \brief intent_ The recognized intent
            //!
            std::string llm_response_;
        };
    };
    std::ostream& operator<<(std::ostream& os, const PicoLLMRecognizerData::Parameters& p);
    std::ostream& operator<<(std::ostream& os, const PicoLLMRecognizerData::Result& r);

    class PicoLLMRecognizer : public RecognizerT<PicoLLMRecognizerData>
    {
    public:
        ~PicoLLMRecognizer();
        void configure(const PicoLLMRecognizerData::Parameters& parameters) override;
        PicoLLMRecognizerData::Result getResult() override;

    private:
        RecordSettings getRecordSettings() override;
        void recognizeInit() override;
        bool recognizeProcess(int16_t* frames) override;

        std::string s_prompt;
        std::string s_llm_response;
        PicoLLMRecognizerData::Result result_;
        pv_picollm_t* picollm_ = NULL;
    };
}  // namespace picovoice_driver
