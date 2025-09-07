#ifndef WAI_OA_IMAGE_PROCESSOR_H
#define WAI_OA_IMAGE_PROCESSOR_H



/////////////////////////////////////////////////
/// Include different strategies
/////////////////////////////////////////////////
#include<wai_oa_image_processing_strategy.h>



/////////////////////////////////////////////////
/// Implementation class of processing strategy
/////////////////////////////////////////////////
class WAIOAImageProcessor
{
private:
     cv::Mat m_mat_img;
     WAIOAImageProcessingStrategy* m_processor;

public:
   // Set the processing strategy
   WAIOAImageProcessor(WAIOAImageProcessingStrategy*);
   ~WAIOAImageProcessor();

   // Set the image to process
   void SetProcessedImage(cv::Mat);

   // Here goes the new processing strategy
   void ChangeProcessingStrategy(WAIOAImageProcessingStrategy*);

   // Do the actual processing here
   tf::Matrix3x3 Process();
};


#endif //WAI_OA_IMAGE_PROCESSOR_H
