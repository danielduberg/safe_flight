#pragma once

#include <nodelet/nodelet.h>

namespace sensor_readings
{
    
    class SRNodelet : public nodelet::Nodelet
    {
        private:
            virtual void onInit();
    };
    
}
