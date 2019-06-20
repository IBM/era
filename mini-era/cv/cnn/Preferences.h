// Preferences.h: interface for the CPreferences class.
#ifndef PREFERENCES_H
#define PREFERENCES_H

typedef struct Preferences{
	unsigned int m_nItemsTrainingLabels;
	unsigned int m_nItemsTrainingImages;

	
	unsigned int m_nItemsTestingLabels;
	unsigned int m_nItemsTestingImages;
        int g_cImageSize;
	int m_nRowsImages;
	int m_nColsImages;
        int m_nMagWindowSize;
	int m_nMagWindowMagnification;

	double m_dInitialEtaLearningRate;
	double m_dLearningRateDecay;
	double m_dMinimumEtaLearningRate;
	unsigned int m_nAfterEveryNBackprops;

	double m_dMicronLimitParameter;
	unsigned int m_nNumHessianPatterns;
	double m_dMaxScaling;  // as a percentage, such as 20.0 for plus/minus 20%
	double m_dMaxRotation;  // in degrees, such as 20.0 for plus/minus rotations of 20 degrees
	double m_dElasticSigma;  // one sigma value for randomness in Simard's elastic distortions
	double m_dElasticScaling;  // after-smoohting scale factor for Simard's elastic distortions
}Preferences;

void InitPreferences(Preferences *p);

#endif
