#ifndef DATA_H
#define DATA_H

namespace CommonUserData
{

    struct StateEstimator
    {
        float      p[3];

        float      vWorld[3];

        float      vBody[3];

        float      rpy[3];

        float      omegaBody[3];

        float      omegaWorld[3];

        float      quat[4];
    };


    struct JointData
    {
        double pos_[12];
        double vel_[12];
        double cur_[12];
        double toq_[12];
    };

}












#endif // DATA_H
