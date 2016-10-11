#ifndef FUSION_H
#define FUSION_H

#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>
#include <ecn_sensorbased/utils.h>



class ServoFusion
{
    ServoFusion()
    {
        Jv_.clear();
        sv_.clear();
        sdv_.clear();
    }

    void addEquality(vpMatrix &_J, vpColVector &_s, vpColVector &_sd, const double &_l)
    {
        Jv_.push_back(&_J);
        sv_.push_back(&_s);
        sdv_.push_back(&_sd);
        lv_.push_back(_l);
    }

    void addLowerBound(vpMatrix &_J, vpColVector &_s, vpColVector &_s_act, vpColVector &_s_min, const double &_l)
    {


    }


    void addBounds(vpMatrix &_J, vpColVector &_s, vpColVector &_s_act, vpColVector &_s_min, const double &_l)
    {




    }

    void init()
    {
        // to be called after all initializations

    }









protected:
    // global Jacobian
    vpMatrix J_, H_;
    // global desired error variation
    vpColVector ed_;

    // vectors for given tasks
    std::vector<vpMatrix*> Jv_;
    std::vector<vpColVector*> sv_, sdv_;
    std::vector<double> lv_;



};





#endif // FUSION_H
