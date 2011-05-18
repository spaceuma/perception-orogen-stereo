#ifndef DENSE_DISPARITY_TYPES__H
#define DENSE_DISPARITY_TYPES__H

#include <base/time.h>
#include <vector>

namespace dense_stereo{

  struct disparity_image
  {
    std::vector<float> data;

    int height;
    int width;

    base::Time time;
  };

}

#endif
