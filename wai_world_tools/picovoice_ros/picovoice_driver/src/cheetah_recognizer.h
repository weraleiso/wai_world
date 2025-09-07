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
#include <pv_cheetah.h>
#include <string>
#include <vector>

#include "./recognizer.h"
#include "./util.h"

namespace picovoice_driver
{
    struct CheetahRecognizerData
    {
        struct Parameters : RecognizerData::Parameters
        {
            double sensitivity_ = 0.5; // Unsued currently!
            bool require_endpoint_=false; // Unsued currently!
            double endpoint_duration_sec_=2.0;
            bool enable_automatic_punctuation=true;
        };

        struct Result
        {
            //!
            //! \brief intent_ The recognized intent
            //!
            std::string transcript_;
        };
    };
    std::ostream& operator<<(std::ostream& os, const CheetahRecognizerData::Parameters& p);
    std::ostream& operator<<(std::ostream& os, const CheetahRecognizerData::Result& r);

    class CheetahRecognizer : public RecognizerT<CheetahRecognizerData>
    {
    public:
        ~CheetahRecognizer();
        void configure(const CheetahRecognizerData::Parameters& parameters) override;
        CheetahRecognizerData::Result getResult() override;

    private:
        RecordSettings getRecordSettings() override;
        void recognizeInit() override;
        bool recognizeProcess(int16_t* frames) override;

        std::stringstream sst_full_transcript;
        CheetahRecognizerData::Result result_;
        pv_cheetah_t* cheetah_ = NULL;
    };
}  // namespace picovoice_driver
