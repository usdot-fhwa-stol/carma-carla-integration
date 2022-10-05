#include "data_stream_toggle.h"

namespace ds_toggle
{
    void DataStreamToggle::initialize()
    {
        pnh_.reset(new ros::CARMANodeHandle("~"));
        

    }

    void DataStreamToggle::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

}