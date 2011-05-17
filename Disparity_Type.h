#ifndef DENSE_DISPARITY_TYPES__H
#define DENSE_DISPARITY_TYPES__H

#include <base/time.h>

namespace dense_stereo{
  struct disparity_image
  {
    float* data;
    
    int height;
    int width;
    
    base::Time time;
  };
  
}

#endif
