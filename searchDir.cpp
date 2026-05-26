#include "searchDir.hpp"
#include <cmath>
#include <iostream>
#include "aris.hpp"



using namespace std;

auto SearchDir::getSign(double inputNum_, double& outputSign_) -> void
{
    if(inputNum_ < 0)
    {
        outputSign_ = -1.0;
    }
    else if (inputNum_ > 0)
    {
        outputSign_ = 1.0;
    }
    else
    {
        outputSign_ = 0.0;
    }


}

auto SearchDir::getDir(double force_data_[6], double& theta_) -> void
{
    double indicator[6]{0};
    double tx = 0;
    double ty = 0;

    std::copy(force_data_, force_data_+6, indicator);

    if(abs(indicator[0]) < 0.15 && abs(indicator[1]) < 0.15)
    {
        getSign(force_data_[4]/force_data_[2], tx);
        getSign(-force_data_[3]/force_data_[2], ty);

        indicator[0] = tx;
        indicator[1] = ty;
    }
    
    for(int i = 0; i<2; i++)
    {
        if(std::abs(indicator[i]) < 0.15)
        {
            indicator[i] = 0;
        }
    }

    theta_ = std::atan2(indicator[1], indicator[0]);

    
    

}