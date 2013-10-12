
// ROS includes
#include <ros/ros.h>
// external includes
#include <schunk_sdh/sdh.h>
#include <schunk_sdh/dsa.h>
#include <kvaser_canlib/canlib.h>


int main(int argc, char** argv)
{
    canHandle h = canOpenChannel(2, canWANT_EXCLUSIVE |canWANT_EXTENDED);
    int ret = (int)canSetBusParams(h, BAUD_500K,4,3,1,1,0);

    canSetBusOutputControl(h, canDRIVER_NORMAL);
    ret = (int)canBusOn(h);


    SDH::cSDH *sdh_;
    sdh_ = new SDH::cSDH(false, false, 0);

    try
    {
        sdh_->OpenCAN_ESD((SDH::tDeviceHandle)(&h), 0.04, 43, 42 );
    }

    catch (SDH::cSDHLibraryException* e)
    {
        ROS_ERROR("An exception was caught: %s", e->what());
        delete e;
    }

    std::vector<double> actualAngles(7);
    std::vector<int> axes_(7);
    for(unsigned int i=0; i<7; i++)
        axes_[i] = i;
    try
    {
        actualAngles = sdh_->GetAxisActualAngle( axes_ );
    }

    catch (SDH::cSDHLibraryException* e)
    {
        ROS_ERROR("An exception was caught: %s", e->what());
        delete e;
    }

    std::cout << "Angles: " << std::endl;
    for(unsigned int i=0; i<7; i++)
        std::cout << actualAngles[i] << std::endl;


    sdh_->Close();
    delete sdh_;

    return 0;
}
