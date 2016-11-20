#include <ecn_sensorbased/qp.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <algorithm>
//#include <ok_tools/okExperiment.h>

using std::string;
using std::vector;
using std::cout;
using std::endl;

double okSolveQP::thresh0 = 1e-6;
// init vectors
okSolveQP::okSolveQP()
{
    const unsigned int N = 10;
    Ms.reserve(2*N);
    Vs.reserve(4*N);
    Hs.reserve(2*N);
    Ts.reserve(2*N);
    Ss.reserve(N);
    Sds.reserve(N);
    Bact.reserve(N);
    Labels.reserve(N);
    hMax = 0;

    accMax.resize(0);

    dt = 1;
    lambda = 1;
    rho = .05;
    nX = 0;
    initStruct = false;
}

okSolveQP::~okSolveQP()
{

    // current list
    for(std::vector<vpBasicFeature *>::iterator it = Ss.begin(); it != Ss.end(); ++it) {
        if ((*it)->getDeallocate() == vpBasicFeature::vpServo) {
            delete (*it) ;
            (*it) = NULL ;
        }
    }
    //desired list
    for(std::vector<vpBasicFeature *>::iterator it = Sds.begin(); it != Sds.end(); ++it) {
        if ((*it)->getDeallocate() == vpBasicFeature::vpServo) {
            delete (*it) ;
            (*it) = NULL ;
        }
    }

    Ss.clear() ;
    Sds.clear() ;


}


// instance-specific methods for cascade of QP
void okSolveQP::updateXdim(const unsigned int &rowM, const unsigned int &colM, const unsigned int &rowV, const string &msg)
{
    // check consistency between matrix and vector
    if(rowM != rowV)
    {
        cout << "okSolveQP::" << msg << ": inconsistent rows, M: " << rowM << " vs V: " << rowV << endl;
    }

    // check consistency between previous and new problem dimension
    if(colM != 0 && colM != nX)
    {
        if(nX != 0)
            cout << "okSolveQP::" << msg << ": inconsistent dimension , was " << nX << " and just got " << colM << endl;
        else
            nX = colM;
    }
    initStruct = false;
}


// Ax = b
void okSolveQP::addEquality(vpMatrix &A, vpColVector &b, const unsigned int &h, const string &label)
{
    updateXdim(A.getRows(), A.getCols(), b.getRows(), "addEquality");
    Ms.push_back(&A);
    Vs.push_back(&b);
    Ts.push_back(EQUALITY);
    Labels.push_back(label);

    Hs.push_back(h);
    if (hMax < h)
        hMax = h;
}

// Cx < d
void okSolveQP::addInequality(vpMatrix &C, vpColVector &d, const unsigned int &h, const string &label)
{
    updateXdim(C.getRows(), C.getCols(), d.getRows(), "addInequality");
    Ms.push_back(&C);
    Vs.push_back(&d);
    Ts.push_back(INEQUALITY);
    Labels.push_back(label);

    Hs.push_back(h);
    if (hMax < h)
        hMax = h;
}

// s = sd
void okSolveQP::addFeatureEquality(vpBasicFeature &s, vpBasicFeature &sd, vpMatrix &J, const unsigned int &h, const string &label)
{
    updateXdim(6, J.getCols(), 6, "addFeatureEquality");
    Ms.push_back(&J);
    Ss.push_back(&s);
    Sds.push_back(&sd);
    Ts.push_back(FEATURE_EQUALITY);
    Labels.push_back(label);

    Hs.push_back(h);
    if (hMax < h)
        hMax = h;
}

// s < bU
void okSolveQP::addFeatureBoundUp(vpBasicFeature &s, vpMatrix &J, vpColVector &bU, const unsigned int &h, const string &label)
{
    updateXdim(s.getDimension(), J.getCols(), bU.getRows(), "addFeatureBoundUp");
    Ms.push_back(&J);
    Ss.push_back(&s);
    Vs.push_back(&bU);
    Ts.push_back(FEATURE_BOUND_UPPER);
    Labels.push_back(label);

    // build activation bounds
    vpColVector b(bU.getRows());
    for(unsigned int i=0;i<b.getRows();++i)
    {
        if(bU[i] > 0)
            b[i] = (1-rho)*bU[i];
        else
            b[i] = (1+rho)*bU[i];
    }
    Bact.push_back(b);

    Hs.push_back(h);
    if (hMax < h)
        hMax = h;
}

// bL < s
void okSolveQP::addFeatureBoundLow(vpBasicFeature &s, vpMatrix &J, vpColVector &bL, const unsigned int &h, const string &label)
{
    updateXdim(s.getDimension(), J.getCols(), bL.getRows(), "addFeatureBoundLow");
    Ms.push_back(&J);
    Ss.push_back(&s);
    Vs.push_back(&bL);
    Ts.push_back(FEATURE_BOUND_LOWER);
    Labels.push_back(label);

    // build activation bounds
    vpColVector b(bL.getRows());
    for(unsigned int i=0;i<b.getRows();++i)
    {
        if(bL[i] > 0)
            b[i] = (1+rho)*bL[i];
        else
            b[i] = (1-rho)*bL[i];
    }
    Bact.push_back(b);

    Hs.push_back(h);
    if (hMax < h)
        hMax = h;
}

// bL < s < bU
void okSolveQP::addFeatureBounds(vpBasicFeature &s, vpMatrix &J, vpColVector &bL, vpColVector &bU, const unsigned int &h, const string &label)
{
    updateXdim(s.getDimension(), J.getCols(), (bL.getRows()+bU.getRows())/2, "addFeatureBounds");
    if(bL.getRows() != bU.getRows())
        cout << "okSolveQP::addFeatureBounds: inconsistent rows, bL: " << bL.getRows() << " vs bU: " << bU.getRows() << endl;
    Ms.push_back(&J);
    Ss.push_back(&s);
    Vs.push_back(&bL);
    Vs.push_back(&bU);
    Ts.push_back(FEATURE_BOUND_BOTH);
    Labels.push_back(label);

    // build activation values
    vpColVector b1(bU.getRows()), b2(bU.getRows());
    for(unsigned int i=0;i<b1.getRows();++i)
    {
        b1[i] = bL[i] + rho*(bU[i]-bL[i]);
        b2[i] = bU[i] - rho*(bU[i]-bL[i]);
    }
    Bact.push_back(b1);
    Bact.push_back(b2);

    Hs.push_back(h);
    if (hMax < h)
        hMax = h;
}


void okSolveQP::updateIndex(const qpType & type, unsigned int &indM, unsigned int &indV, unsigned int &indS, unsigned int &indSd)
{
    switch(type)
    {
    case EQUALITY:
    case INEQUALITY:
        indM++;		indV++;		break;
    case FEATURE_EQUALITY:
        indSd++;
    case FEATURE_BOUND_BOTH:
    case FEATURE_BOUND_UPPER:
    case FEATURE_BOUND_LOWER:
        // for feature bounds, indV and indAct have been updated in computeVmax
        indM++;		indS++;
    }
}

void okSolveQP::changeHierarchy(const string &label, const unsigned int &h)
{
    // change in list
    for(unsigned i=0;i<Labels.size();++i)
    {
        if(Labels[i] == label)
            Hs[i] = h;
    }

    if (hMax < h)
        hMax = h;

    initStruct = false;
}

void okSolveQP::setMaxAccel(const vpColVector &aMax)
{
    if(nX == 0)
        nX = aMax.getRows();
    if(aMax.getRows() != nX)
        cout << " okSolveQP::setMaxAccel: inconsistent dimension, nX: " << nX << " vs aMax: " << aMax.getRows() << endl;
    else
    {
        accMax = aMax;
        Ip.eye(nX);
        Im = -Ip;
        xMax.resize(nX);
        xMin.resize(nX);
        xPrev.resize(nX);xPrev = 0;

        addInequality(Ip, xMax, 0, "Max Acceleration");
        addInequality(Im, xMin, 0, "Min Acceleration");
    }
}


void okSolveQP::computeVmax(const vpColVector &s, unsigned int &indV, unsigned int &indAct, vpColVector &vPos, vpColVector &vNeg)
{
    vPos.resize(s.getRows());
    vNeg.resize(s.getRows());
    double v;
    for(unsigned int i=0;i<vPos.getRows();++i)
    {
        v = ((*(Vs[indV+1]))[i] - (*(Vs[indV]))[i])/dt;
        if(s[i] < (*(Vs[indV]))[i])
        {	// below lower limit: only allow positive
            vPos[i] = v;
            vNeg[i] = 0;
        }
        else if(s[i] > (*(Vs[indV+1]))[i])
        {	// above upper limit: only allow negative
            vPos[i] = 0;
            vNeg[i] = v;
        }
        else if(s[i] < Bact[indAct][i])
        {	// in lower threshold: allow positive and weighted negative
            vPos[i] = v;
            vNeg[i] = v*(s[i]-(*(Vs[indV]))[i])/(Bact[indAct][i]-(*(Vs[indV]))[i]);
        }
        else if(s[i] > Bact[indAct+1][i])
        {	// in upper threshold: allow negative and weighted positive
            vPos[i] = v*(s[i]-(*(Vs[indV+1]))[i])/(Bact[indAct+1][i]-(*(Vs[indV+1]))[i]);
            vNeg[i] = v;
        }
        else
        {	// confidence region: allow full motion
            vPos[i] = v;
            vNeg[i] = v;
        }
    }
    // update indexes
    indV += 2;
    indAct += 2;
}

void okSolveQP::computeVmaxLow(const vpColVector &s, unsigned int &indV, unsigned int &indAct, vpColVector &vNeg)
{
    vNeg.resize(s.getRows());
    double v;
    for(unsigned int i=0;i<vNeg.getRows();++i)
    {
        v = vpMath::abs((*(Vs[indV]))[i])/dt;
        if(s[i] < (*(Vs[indV]))[i])
        {	// below lower limit: don't move
            vNeg[i] = 0;
        }
        else if(s[i] < Bact[indAct][i])
        {	// in lower threshold: allow weighted negative
            vNeg[i] = v*(s[i]-(*(Vs[indV]))[i])/(Bact[indAct][i]-(*(Vs[indV]))[i]);
        }
        else
        {	// confidence region: allow full motion
            vNeg[i] = v;
        }
    }
    // update indexes
    indV++;
    indAct++;
}

void okSolveQP::computeVmaxUp(const vpColVector &s, unsigned int &indV, unsigned int &indAct, vpColVector &vPos)
{
    vPos.resize(s.getRows());
    double v;
    for(unsigned int i=0;i<vPos.getRows();++i)
    {
        v = vpMath::abs((*(Vs[indV]))[i])/dt;

        if(s[i] > (*(Vs[indV]))[i])
        {	// above upper limit: don't move
            vPos[i] = 0;
        }
        else if(s[i] > Bact[indAct][i])
        {	// in upper threshold: allow weighted positive
            vPos[i] = v*(s[i]-(*(Vs[indV]))[i])/(Bact[indAct][i]-(*(Vs[indV]))[i]);
        }
        else
        {	// confidence region: allow full motion
            vPos[i] = v;
        }
    }
    // update indexes
    indV++;
    indAct++;
}


void okSolveQP::print()
{
    unsigned int i;
    unsigned int countA, countC, countJ, countL, countU, countLU;
    vector<string> labl;
    unsigned int indM, indV, indS, indSd;

    cout << "okSolveQP: dimension=" << nX << endl;

    // print info for all hierarchy levels
    for (unsigned int h=0;h<hMax+1;++h)
    {
        indM = indV = indS = indSd = 0;
        countA = countC = countJ = countL = countU = countLU = 0;
        labl.clear();

        for(i=0;i<Hs.size();++i)
        {
            if(Hs[i] == h)
            {
                if(Labels[i] != "")
                    labl.push_back(Labels[i]);

                switch(Ts[i])
                {
                case EQUALITY:
                    countA += Vs[indV]->getRows();
                    break;
                case INEQUALITY:
                    countC += Vs[indV]->getRows();
                    break;
                case FEATURE_EQUALITY:
                    countJ += Ss[indS]->getDimension();
                    break;
                case FEATURE_BOUND_LOWER:
                    countL += Ss[indS]->getDimension();
                    break;
                case FEATURE_BOUND_UPPER:
                    countU += Ss[indS]->getDimension();
                    break;
                case FEATURE_BOUND_BOTH:
                    countLU += Ss[indS]->getDimension();
                    break;
                }
            }

            updateIndex(Ts[i], indM, indV, indS, indSd);
        }

        if (countA + countC + countJ + countL + countU + countLU > 0)
        {
            cout << " --------------- Priority " << h << " ----------------------" << endl;

            if(labl.size() > 0)
            {
                cout << "  Labels: ";
                for(i=0;i<labl.size()-1;++i)
                    cout << labl[i] << " - ";
                cout << labl[labl.size()-1] << endl;
            }
            else
                cout << "(No Labels)" << endl;

            if (countA > 0)
                cout << "  QP equalities      : " << countA << endl;
            if (countC > 0)
                cout << "  QP inequalities    : " << countC << endl;
            if (countJ > 0)
                cout << "  Features equalities: " << countJ << endl;
            if (countL > 0)
                cout << "  Feature bound lower: " << countL << endl;
            if (countU > 0)
                cout << "  Feature bound upper: " << countU << endl;
            if (countLU > 0)
                cout << "  Feature bound both : " << countLU << endl;
            cout << endl;
        }
    }
}



// solve the cascade
void okSolveQP::solveCascade(vpColVector &solution)
{
    int h, i, j, k;

    // initialize the matrices Ak, Bk, Ck, Dk from stored values
    // if it is the first time, this is done by stacking matrices
    // otherwise, these matrices should already have the good dimension, hence we perform an element-wise copy
    if(!initStruct)
    {
        Ak.resize(hMax+1);
        Ck.resize(hMax+1);
        Bk.resize(hMax+1);
        Dk.resize(hMax+1);
        for(h=0;h<hMax+1;++h)
        {
            Ak[h].resize(0,nX);
            Ck[h].resize(0,nX);
            Bk[h].resize(0);
            Dk[h].resize(0);
        }
        preactive_.resize(hMax+1);
    }

    // if maximum acceleration, update values of xMin and xMax
    if(accMax.getRows() > 0)
    {
        xMax = xPrev + accMax;
        xMin = accMax - xPrev;
        //   printvar("QP/accMax: xMax", xMax.t());
        //   printvar("QP/accMax: xMin", xMin.t());
    }

    unsigned int indM = 0, indV = 0, indS = 0, indSd = 0, indAct = 0;
    vector<unsigned int> indA(hMax+1,0), indC(hMax+1,0);
    vpMatrix LJ;
    vpColVector e, vPos, vNeg;
    for(k=0;k<Hs.size();++k)
    {
        h = Hs[k];
        //printvar("Dealing with hierarchy", h);
        switch(Ts[k])
        {
        case EQUALITY:
            // copy Ms into Ak and Vs into Bk
            if(initStruct)
            {
                for(i=0;i<Ms[indM]->getRows();++i)
                {
                    for(j=0;j<Ms[indM]->getCols();++j)
                        Ak[h][indA[h]+i][j] = (*(Ms[indM]))[i][j];
                    Bk[h][indA[h]+i] = (*(Vs[indV]))[i];
                }
                indA[h] += Ms[indM]->getRows();
            }
            else
            {
                Ak[h].stack((*(Ms[indM])));
                Bk[h].stack(*(Vs[indV]));
            }
            break;
        case INEQUALITY:
            // copy Ms into Ck and Vs into Dk
            if(initStruct)
            {
                for(i=0;i<Ms[indM]->getRows();++i)
                {
                    for(j=0;j<Ms[indM]->getCols();++j)
                        Ck[h][indC[h]+i][j] = (*(Ms[indM]))[i][j];
                    Dk[h][indC[h]+i] = (*(Vs[indV]))[i];
                }
                indC[h] += Ms[indM]->getRows();
            }
            else
            {
                Ck[h].stack(*(Ms[indM]));
                Dk[h].stack(*(Vs[indV]));
            }
            break;
        case FEATURE_EQUALITY:
            // copy interaction*Ms into Ak and -lambda*error into Bk
            LJ = Ss[indS]->interaction() * *Ms[indM];
            e = -lambda*Ss[indS]->error(*(Sds[indS]));
            if(initStruct)
            {
                for(i=0;i<LJ.getRows();++i)
                {
                    for(j=0;j<LJ.getCols();++j)
                        Ak[h][indA[h]+i][j] = LJ[i][j];
                    Bk[h][indA[h]+i] = e[i];
                }
                indA[h] += LJ.getRows();
            }
            else
            {
                Ak[h].stack(LJ);
                Bk[h].stack(e);
            }
            break;
        case FEATURE_BOUND_LOWER:
            // copy -interaction*Ms into Ck and negative bound into Dk
            LJ = Ss[indS]->interaction() * *Ms[indM];
            computeVmaxLow(Ss[indS]->get_s(), indV, indAct, vNeg);
            if(initStruct)
            {
                for(i=0;i<LJ.getRows();++i)
                {
                    for(j=0;j<LJ.getCols();++j)
                        Ck[h][indC[h]+i][j] = -LJ[i][j];
                    Dk[h][indC[h]+i] = vNeg[i];
                }
                indC[h] += LJ.getRows();
            }
            else
            {
                Ck[h].stack(-LJ);
                Dk[h].stack(vNeg);
            }
            break;
        case FEATURE_BOUND_UPPER:
            // copy interaction*Ms into Ck and positive bound into Dk
            LJ = Ss[indS]->interaction() * *Ms[indM];
            computeVmaxUp(Ss[indS]->get_s(), indV, indAct, vPos);
            if(initStruct)
            {
                for(i=0;i<LJ.getRows();++i)
                {
                    for(j=0;j<LJ.getCols();++j)
                        Ck[h][indC[h]+i][j] = LJ[i][j];
                    Dk[h][indC[h]+i] = vPos[i];
                }
                indC[h] += LJ.getRows();
            }
            else
            {
                Ck[h].stack(LJ);
                Dk[h].stack(vPos);
            }
            break;
        case FEATURE_BOUND_BOTH:
            // copy interaction*Ms into Ck and positive bound into Dk
            // copy -interaction*Ms into Ck and negative bound into Dk
            LJ = Ss[indS]->interaction() * *Ms[indM];
            computeVmax(Ss[indS]->get_s(), indV, indAct, vPos, vNeg);
            if(initStruct)
            {
                for(i=0;i<LJ.getRows();++i)
                {
                    for(j=0;j<LJ.getCols();++j)
                    {
                        Ck[h][indC[h]+i][j] = LJ[i][j];
                        Ck[h][indC[h]+i+LJ.getRows()][j] = -LJ[i][j];
                    }
                    Dk[h][indC[h]+i] = vPos[i];
                    Dk[h][indC[h]+i+LJ.getRows()] = vNeg[i];
                }
                indC[h] += 2*LJ.getRows();
            }
            else
            {
                Ck[h].stack(LJ);
                Ck[h].stack(-LJ);
                Dk[h].stack(vPos);
                Dk[h].stack(vNeg);
            }
            break;
        }
        updateIndex(Ts[k], indM, indV, indS, indSd);
    }
    // end initialization of Ak, Bk, Ck, Dk
    initStruct = true;

    /*for(h=0;h<hMax+1;++h)
 {
  cout << " --------------- Priority " << h << " ----------------------" << endl;
  printvar("Ak[h]", Ak[h]);
  printvar("Bk[h]^t", Bk[h].t());
  printvar("Ck[h]", Ck[h]);
  printvar("Dk[h]^t", Dk[h].t());
 }*/

    // init meta-matrices AA, BB, CC, DD
    vpMatrix AA(0, nX), CC(0, nX);
    vpColVector BB, DD;

    /* solve iterative QP's
 at each iteration we solve :
 min || Ak.x - Bk || + ||w||
 st. AA.x = BB
  CC.x < DD
  Ck.x - w < Dk

 The problem is written canonical as:
  min  ||Q.X - r||^2
  st. A.X = b
   C.X < d
  with	X = (x,w)
  Q = Akt.Ak
  r = Akt.B
  */
    // init canonical QP matrices
    vpMatrix Q, A, C, Iw;
    vpColVector r, b, d, X;
    vpSubColVector x;
    unsigned int nW;
    for(h=0;h<hMax+1;++h)
    {
        nW = Ck[h].getRows();

        if(nW != 0 || Bk[h].getRows() != 0)		// if nothing at this level, skip to next one
        {
            // build canonical QP matrices Q, r, A, b, C, d
            Iw.eye(nW);
            Q = vpMatrix::stack(vpMatrix::juxtaposeMatrices(Ak[h],vpMatrix(Ak[h].getRows(),nW)),
                                vpMatrix::juxtaposeMatrices(vpMatrix(nW,Ak[h].getCols()), Iw));
            r = vpColVector::stack(Bk[h], vpColVector(nW));

            if(AA.getRows() > 0)
            {
                A = vpMatrix::stack(vpMatrix::juxtaposeMatrices(AA,vpMatrix(AA.getRows(),nW)),
                                    vpMatrix::juxtaposeMatrices(vpMatrix(nW,AA.getCols()), vpMatrix(nW,nW)));
                b = vpColVector::stack(BB, vpColVector(nW));
            }
            else
            {
                A.resize(0,nX+nW);
                b.resize(0);
            }
            if(CC.getRows() + nW > 0)
            {
                C = vpMatrix::stack(vpMatrix::juxtaposeMatrices(CC,vpMatrix(CC.getRows(),nW)),
                                    vpMatrix::juxtaposeMatrices(Ck[h], -Iw));
                d = vpColVector::stack(DD, Dk[h]);
            }
            else
            {
                C.resize(0,nX+nW);
                d.resize(0);
            }

            // solve program
            solveQP(Q, r, A, b, C, d, X, preactive_[h]);

            // extract solution
            x.init(X, 0, nX);

            // unless we are at the last iteration, update AA, BB, CC, DD and loop
            if(h != hMax)
            {
                // equalities stay equalities
                AA.stack(Ak[h]);
                BB.stack(Ak[h]*x);

                vpRowVector row;
                for(i=0;i<nW;++i)
                {
                    //cout << "Trying to extract row " << i << " from:" << endl << Ck[h] << endl;
                    row = Ck[h].getRow(i);
                    if(row*x < Dk[h][i] + thresh0)
                    {	// ensured inequalities stay inequalities
                        CC.stack(row);
                        DD.stack(Dk[h][i]);
                    }
                    else
                    {	// violated inequalities become equalities
                        AA.stack(row);
                        BB.stack(row*x);
                    }
                }
            }
        }
    }

    solution = xPrev = x;
}



/* Solves a quadratic minimization under equality constraint with projection approach
 * min_x ||Q.x - r|||^2
 * st. A.x = b
 */
void okSolveQP::solveQPe ( const vpMatrix &_Q, const vpColVector &_r, const vpMatrix &_A, const vpColVector &_b, vpColVector &_x)
{
    vpMatrix _Ap = _A.pseudoInverse();
    vpColVector x1 = _Ap * _b;
    vpMatrix P;P.eye(x1.getRows());
    P -= _Ap * _A;
    _x = (_Q*P).pseudoInverse() * (_r - _Q*x1);
}


/*
 * QR with Eigen is too long because of copy time even for big matrices
void okSolveQP::solveQPe_qr ( const vpMatrix &_Q, const vpColVector &_r, const vpMatrix &_A, const vpColVector &_b, vpColVector &_x, const double &_lambda)
{

    // to do: trivial solution through projection, this sould be the core function
    // Change matrices to Eigen
    MatrixXd A1t;   VispToEigen(_A.t(), A1t);
    VectorXd b1;    VispToEigen(_b, b1);
    MatrixXd A2;    VispToEigen(_Q, A2);
    VectorXd b2;    VispToEigen(_r, b2);

    Eigen::HouseholderQR<Eigen::MatrixXd> QR(A1t);
    MatrixXd Q = QR.householderQ();
    MatrixXd R0 = QR.matrixQR().triangularView<Upper>();

    // off-the-hat solution
    const unsigned int n = A1t.rows();
    const unsigned int m = A1t.cols();

    MatrixXd Z,Rt;
    Z = Q.block(0,m, n, n-m);
    Rt = R0.block(0,0, m, m);
    Rt.transposeInPlace();
    VectorXd x1 = Q.block(0,0, n, m) * Rt.triangularView<Eigen::Lower>().solve(b1);
    VectorXd z = (A2*Z).colPivHouseholderQr().solve((b2-A2*x1));
    EigenToVisp(x1+Z*z, _x);
}
*/

/* Solves a quadratic minimization under inequality constraint
 * Just a particular case of general problem
 * min_x ||Q.x - r|||^2
 * st. C.x <= d
 */
void okSolveQP::solveQPi ( const vpMatrix &Q, const vpColVector &r, vpMatrix C, const vpColVector &d, vpColVector &x, std::vector<bool> &active)
{
    vpMatrix A ( 0,Q.getCols() );
    vpColVector b ( 0 );
    solveQP ( Q, r, A, b, C, d, x, active);
}


/* Solves a quadratic minimization under equality and inequality constraint, uses projection
 * min_x ||Q.x - r|||^2
 * st. A.x = b
 * st. C.x <= d
 */
void okSolveQP::solveQP ( const vpMatrix &_Q, const vpColVector &_r, vpMatrix _A, vpColVector _b, const vpMatrix &_C, const vpColVector &_d, vpColVector &_x, std::vector<bool> &active)
{
    // check data coherence
    const unsigned int n = _Q.getCols();
    if (    n != _A.getCols() ||
            n != _C.getCols() ||
            _A.getRows() != _b.getRows() ||
            _C.getRows() != _d.getRows() ||
            _Q.getRows() != _r.getRows())
    {
        cout << "solveQP: wrong dimension" << endl <<
                "Q: " << _Q.getRows() << "x" << _Q.getCols() << " - r: " << _r.getRows() << endl <<
                "A: " << _A.getRows() << "x" << _A.getCols() << " - b: " << _b.getRows() << endl <<
                "C: " << _C.getRows() << "x" << _C.getCols() << " - d: " << _d.getRows() << endl;
        return;
    }

   /* cout << "solveQP: dimensions" << endl;
    cout << "Q: " << _Q.getRows() << "x" << _Q.getCols() << " - r: " << _r.getRows() << endl <<
            "A: " << _A.getRows() << "x" << _A.getCols() << " - b: " << _b.getRows() << endl <<
            "C: " << _C.getRows() << "x" << _C.getCols() << " - d: " << _d.getRows() << endl;
   */ /*  printvar("Q", Q);
    printvar("r", r);
    printvar("A", A);
    printvar("b", b);
    printvar("C", C);
    printvar("d", d);*/

    unsigned int i,j;
    const unsigned int nA = _A.getRows();
    const unsigned int nC = _C.getRows();

    /*cout << "constraints:";
    for(unsigned int i=0;i<nC;++i)
    {
        if(active[i])
            cout << " true";
        else
            cout << " false";
    }
    cout << endl;*/

    // vector<bool> active(nC, false);
    if(active.size() != nC)
        active.resize(nC, false);

    // look for trivial solution (case of only inequalities, often the highest priority for robot constraints)
    if(_r.euclideanNorm() == 0 &&
            (_d.getRows() == 0 || _d.getMinValue() >= 0) &&
            (_b.getRows() == 0 || _b.euclideanNorm() == 0))
    {
        cout << "okSolveQP::solveQP: trivial solution"	 << endl;
        _x.resize(n);
        return;
    }

    // no trivial solution, go for solver
    vector< vector<bool> > activePast;
    vector<bool> activeBest = active;
    activePast.reserve(5);
    unsigned int nAct = count ( active.begin(), active.end(),true );

    double ineqMax, errCur, errBest = -1;
    unsigned int ineqInd, ineqCount;
    vpMatrix In;In.eye(n);
    vpMatrix P = In;
    vpMatrix Ap;
    vpColVector x(n), l, e;
    vpSubMatrix ApC;

    // solve at one iteration
    while ( true )
    {
        activePast.push_back ( active );
        //   cout << "M: " << M.getRows() << "x" << M.getCols() << endl;
        //  cout << "v: " << v.getRows() << endl;
       /* cout << "active: ";
        for(i=0;i<active.size();++i)
            cout << active[i] << ", ";
        cout << endl;*/

        // update A and b (equality constraints)
        _A.resize(nA + nAct, n, false );
        _b.resize ( nA + nAct, false );

        // active set from C and d
        ineqCount = 0;
        for ( i=0;i<nC;++i )
        {
            if ( active[i] )
            {
                for ( j=0;j<n;++j )
                {
                    _A[nA+ineqCount][j] = _C[i][j];
                    _b[nA+ineqCount] = _d[i];

                }
                ineqCount++;
            }
        }
        // end init A and b

        // solve with projection
        if(_A.getRows() != 0)
        {
            Ap = _A.pseudoInverse();
            x = Ap * _b;
            P = In - Ap * _A;
            x += P*(_Q*P).pseudoInverse() * (_r - _Q*x);
        }
        else
        {
            x = _Q.pseudoInverse() * _r;
        }

        //cout << "temporary solution: " << x.t() << endl;

        // find strongest violated inequality in Cx > d
        ineqMax = 0;
        for ( i=0;i<nC;++i )
        {
            if ( _C.getRow ( i ) * x - _d[i] > ineqMax + thresh0 )
            {
                ineqMax = _C.getRow ( i ) * x - _d[i];
                ineqInd = i;
            }
        }

        if ( ineqMax != 0 )			// active worst violated equality
        {
            nAct++;
            active[ineqInd] = true;
           // cout << "activating ineq: " << ineqInd << endl;
        }
        else						// all inequalities ensured, ineqMax==0
        {
            // this solution is feasible, store it if it is the best found up to now
            e = (_Q*x - _r);
            errCur = e.euclideanNorm();
            if ( errBest == -1 || errCur < errBest )
            {
                errBest = errCur;
                activeBest = active;
                _x = x;
            }

            // try to deactivate a constraint, compute Lagrange multiplier
            if(_A.getRows())
            {
                ApC.init(Ap, 0, nA, n, nAct);
                l = -ApC.transpose() * _Q.transpose() * e;

                ineqCount = 0;
                ineqMax = 0;
                for ( i=0;i<nC;++i )
                    if ( active[i] )
                    {
                        if ( l[ineqCount] < ineqMax )
                        {
                            ineqMax = l[ineqCount];
                            ineqInd = i;
                        }
                        ineqCount++;
                    }

                // deactive most useless inequality if any
                if ( ineqMax != 0 )
                {
                    active[ineqInd] = false;
                    nAct--;
                   // cout << "deactivating ineq: " << ineqInd << endl;

                }
                else	// no useless equality, this has to be the optimal solution
                    break;
            }
        }

        // before looping again, check whether the new active set candidate has already been tested or not
        ineqInd = 0;
        ineqCount = 0;
        for ( i=0;i<activePast.size() && ineqCount != nC;++i )
        {
            ineqCount = 0;
            for ( j=0;j<nC;++j )
            {
                if ( activePast[i][j] == active[j] )
                    ineqCount++;
            }
        }
        // if nC same values: new active set has already been tested, we're beginning a cycle
        // leave the loop
        if ( ineqCount == nC )
            break;
    }
    active = activeBest;
}
