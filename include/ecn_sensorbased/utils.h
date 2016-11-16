#ifndef UTILS_H
#define UTILS_H

#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>

// weighting function for extended Jacobian approach
double weight(double s, double s_act, double s_max)
{
    if(s < s_act)
        return 0;
    return (s-s_act)/(s_max-s);
}

double weightBothSigns(double s, double s_act, double s_max)
{
    return weight(s, s_act, s_max) + weight(-s, s_act, s_max);
}

// put a matrix inside another
void putAt(vpMatrix &_J, const vpMatrix &_Jsub, const unsigned int r, const unsigned int c)
{
    vpSubMatrix Js(_J, r, c, _Jsub.getRows(), _Jsub.getCols());
    Js = _Jsub;
}

void putAt(vpColVector &_e, const vpColVector &_esub, const unsigned int r)
{
    vpSubColVector es(_e, r, _esub.getRows());
    es = _esub;
}


#endif // UTILS_H
