/*
 * test_hand
 *
 *  Created on: Jan 15, 2013
 *  Authors:   Francisco Viña 
 *            fevb <at> kth.se
 */

/* Copyright (c) 2013, Francisco Vina, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
