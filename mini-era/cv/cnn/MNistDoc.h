// MNistDoc.h : interface of the CMNistDoc class
#ifndef MNISTDOC_H
#define MNISTDOC_H

#include "NeuralNetwork.h"	// Added by ClassView
#include "vector.h"
#include "Preferences.h"
//#define NULL ( (void *) 0)

#define GAUSSIAN_FIELD_SIZE ( 21 )  // strictly odd number
#include <stdbool.h>


typedef struct{
	double* m_DispH;  // horiz distortion map array
	double* m_DispV;  // vert distortion map array
	double m_GaussianKernel[ GAUSSIAN_FIELD_SIZE ] [ GAUSSIAN_FIELD_SIZE ];

	int m_cCols;  // size of the distortion maps
	int m_cRows;
	int m_cCount;
	volatile unsigned int m_cBackprops;
	volatile bool m_bNeedHessian;
	unsigned int m_nAfterEveryNBackprops;
	double m_dEtaDecay;
	double m_dMinimumEta;
	volatile double m_dEstimatedCurrentMSE;  // this number will be changed by one thread and used by others

	FILE *m_fileTrainingLabels;
	FILE *m_fileTrainingImages;
	FILE *m_fileTestingLabels;
	FILE *m_fileTestingImages;

	bool m_bDistortTrainingPatterns;

	volatile unsigned int m_iNextTrainingPattern;
	volatile unsigned int m_iRandomizedTrainingPatternSequence[ 60000 ];
        int g_cImageSize;
       	bool m_bDistortTestingPatterns;
	unsigned int m_iWhichImageSet;  // 0 == training set; 1 == testing set (which is the default
	unsigned int m_nItemsTrainingImages;
	unsigned int m_nItemsTestImages;
	volatile unsigned int m_iNextTestingPattern;
       NeuralNetwork *m_NN;

}CMNistDoc;

void InitializeCMNistDoc(CMNistDoc *cmnistdoc);
bool OnNewDocument(CMNistDoc *cmnistdoc);
double GetCurrentEta(CMNistDoc *cmnistdoc);
double GetPreviousEta(CMNistDoc *cmnistdoc);
unsigned int GetCurrentTrainingPatternNumber( CMNistDoc *cmnistdoc,bool bFromRandomizedPatternSequence);
void RandomizeTrainingPatternSequence(CMNistDoc *cmnistdoc);
unsigned int GetNextTrainingPattern(CMNistDoc *cmnistdoc,unsigned char* pArray , int* pLabel, bool bFlipGrayscale ,bool bFromRandomizedPatternSequence , unsigned int* iSequenceNum );
void GetTrainingPatternArrayValues( CMNistDoc *cmnistdoc,int iNumImage , unsigned char* pArray, int* pLabel , bool bFlipGrayscale);

unsigned int GetRandomTrainingPattern(CMNistDoc *cmnistdoc,unsigned char* pArray, int* pLabel, bool bFlipGrayscale);
unsigned int GetNextTestingPatternNumber(CMNistDoc *cmnistdoc);
void GetTestingPatternArrayValues(CMNistDoc *cmnistdoc,int iNumImage, unsigned char* pArray, int* pLabel ,bool bFlipGrayscale  );

unsigned int GetNextTestingPattern(CMNistDoc *cmnistdoc,unsigned char* pArray, int* pLabel,bool bFlipGrayscale);  // returns TRUE to signify roll-over back to zero-th pattern
double *At( CMNistDoc *cmnistdoc, double* p, int row, int col );
void GenerateDistortionMap(CMNistDoc *cmnistdoc, double severityFactor , Preferences *preferences );

void ApplyDistortionMap( CMNistDoc *cmnistdoc,double* inputVector );

void CalculateNeuralNet(CMNistDoc *cmnistdoc,double* inputVector, int count, double* outputVector,int oCount, vector** pNeuronOutputs, bool bDistort  );
void CalculateHessian(CMNistDoc *cmnistdoc , Preferences *p);

void BackpropagateNeuralNet(CMNistDoc *cmnistdoc,double *inputVector, int iCount, double* targetOutputVector, double* actualOutputVector, int oCount, vector** pMemorizedNeuronOutputs, bool bDistort , Preferences *preferences );
unsigned int BackpropagationThread(CMNistDoc *cmnistdoc , Preferences *p);
bool StartBackpropagation(CMNistDoc *cmnistdoc,unsigned int iStartPattern, unsigned int iNumThreads, double initialEta, double minimumEta, double etaDecay , unsigned int nAfterEvery , bool bDistortPatterns, double estimatedCurrentMSE , Preferences *preferences );
unsigned int TestingThread( CMNistDoc *cmnistdoc,Preferences *p );
bool StartTesting(CMNistDoc *cmnistdoc,unsigned int iStartingPattern, unsigned int iNumThreads,  bool bDistortPatterns, unsigned int iWhichImageSet );




#endif 
