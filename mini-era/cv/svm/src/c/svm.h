/********************************
Author: Sravanthi Kota Venkata
********************************/

#ifndef _SCRIPT_SVM_
#define _SCRIPT_SVM_

#include "sdvbs_common.h"

typedef struct
{
    F2D* a;
    float b;
    float C;
    int d;
    int dim;
    F2D* e;
    float eps;
    F2D* a_result;
    F2D* b_result;
    F2D* X;
    F2D *Y;
    float tolerance;
    int ret;

}alphaRet;

alphaRet* getAlphaFromTrainSet(int N, F2D* trn1, F2D* trn2, int iterations);
float polynomial(int d, F2D* a, F2D* b, int dim);
float cal_learned_func(int k, F2D* a, float* b, int N, F2D* Y, F2D* X, int dim);
int examineExample(int i, F2D* a, float* b, float C, F2D* e, F2D* X, F2D* Y, float tolerance, int N, float eps, int dim);
int takeStep(int i, int j, F2D* a, float C, F2D* e, F2D* Y, F2D* X, float eps, float* b, int N, int dim);
F2D* usps_read_partial(F2D* dcell1, F2D* dcell2, int idx, int opt, int dim, int iterations);
int script_svm();

#endif


