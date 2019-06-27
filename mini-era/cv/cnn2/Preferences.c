// Preferences.cpp: implementation of the CPreferences class.
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h> 
#include "Preferences.h"
void InitPreferences(Preferences *p)
{

	p->m_nItemsTestingLabels = 10000;
	p->m_nItemsTestingImages = 10000;
	p->g_cImageSize = 28;
	p->m_nRowsImages = p->g_cImageSize;
	p->m_nColsImages = p->g_cImageSize;
	p->m_nItemsTrainingLabels = 60000;
	p->m_nItemsTrainingImages = 60000;

	p->m_nMagWindowSize = 5;
	p->m_nMagWindowMagnification = 8;

	p->m_dInitialEtaLearningRate = 0.001;
	p->m_dLearningRateDecay = 0.794328235;  // 0.794328235 = 0.001 down to 0.00001 in 20 epochs
	p->m_dMinimumEtaLearningRate = 0.00001;
	p->m_nAfterEveryNBackprops = 60000;
	p->m_dMaxScaling = 15.0;  // like 20.0 for 20%
	p->m_dMaxRotation = 15.0;  // like 20.0 for 20 degrees
	p->m_dElasticSigma = 8.0;  // higher numbers are more smooth and less distorted; Simard uses 4.0
	p->m_dElasticScaling = 0.5;  // higher numbers amplify the distortions; Simard uses 34 (sic, maybe 0.34 ??)

	p->m_dMicronLimitParameter = 0.10;  // since we divide by this, update can never be more than 10x current eta
	p->m_nNumHessianPatterns = 500;  // number of patterns used to calculate the diagonal Hessian
}
