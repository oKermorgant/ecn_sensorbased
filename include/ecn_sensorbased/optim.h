#ifndef OPTIM_H
#define OPTIM_H
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>


namespace ecn
{

// Utility functions for varying weights
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



// ViSP vpMatrix utilities to put a matrix inside another
void putAt(vpMatrix &_J, const vpMatrix &_Jsub, const unsigned int r, const unsigned int c)
{
    vpSubMatrix Js(_J, r, c, _Jsub.getRows(), _Jsub.getCols());
    Js = _Jsub;
}

// put a vector inside another
void putAt(vpColVector &_e, const vpColVector &_esub, const unsigned int r)
{
    vpSubColVector es(_e, r, _esub.getRows());
    es = _esub;
}



// basic QP solvers

/* Solves a quadratic minimization under equality constraint with projection approach
 * Just a particular case of general problem
 * min_x ||Q.x - r||^2
 * st. A.x = b
 */
void solveQPe ( const vpMatrix &_Q, const vpColVector &_r, const vpMatrix &_A, const vpColVector &_b, vpColVector &_x);


/* Solves a quadratic minimization under inequality constraint
 * Just a particular case of general problem
 * min_x ||Q.x - r||^2
 * st. C.x <= d
 */
void solveQPi ( const vpMatrix &Q, const vpColVector &r, vpMatrix C, const vpColVector &d, vpColVector &x);


/* Solves a quadratic minimization under equality and inequality constraint, uses projection
 * min_x ||Q.x - r||^2
 * st. A.x = b
 * st. C.x <= d
 */
void solveQP ( const vpMatrix &_Q, const vpColVector &_r, vpMatrix _A, vpColVector _b, const vpMatrix &_C, const vpColVector &_d, vpColVector &_x);

}


#endif // OPTIM_H
