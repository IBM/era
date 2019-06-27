// MNistDoc.cpp : implementation of the CMNistDoc class
//
#include <malloc.h>
#include <stdlib.h>
//#define NULL ( (void *) 0)
#include <stdbool.h>
#include "vector.h"
#include "NeuralNetwork.h"
//include "mnist_parse.h"
#include "Preferences.h"
#include <stdio.h>
#include "MNistDoc.h"

#define UNIFORM_PLUS_MINUS_ONE ( (double)(2.0 * rand())/RAND_MAX - 1.0 )
// UNIFORM_ZERO_THRU_ONE gives a uniformly-distributed number between zero (inclusive) and one (exclusive)
#define UNIFORM_ZERO_THRU_ONE ( (double)(rand())/(RAND_MAX ) )

// void createConvolutionLayer(NNLayer *pLayer, int outputHeight, int filterSize,
//                             int numFilters);  //Default stride is 2

void createInputLayer(NNLayer *pLayer, int inputHeight, int inputDepth) {
	int icNeurons = 0;
	int ii, jj, kk;
	unsigned int iNumWeight;

	int fm;
	for ( ii = 0; ii < inputHeight * inputHeight * inputDepth; ++ii ) {
		NNNeuron *x = (NNNeuron *) malloc(sizeof(NNNeuron));
		InitializeNNNeuron(x, 0.0);
		vector_add( (vector *) pLayer->m_Neurons->Neurons, (void *) x );
		icNeurons++;
	}
}

void createConvolutionLayer(NNLayer *pLayer, int inputHeight, int inputDepth, int outputHeight,
                            int filterSize,
                            int numFilters) {
	int ii, jj, kk, i, j, fm;
	int icNeurons = 0;
	double initWeight;
	unsigned int iNumWeight;

	for ( ii = 0; ii < outputHeight * outputHeight * numFilters; ++ii ) {
		NNNeuron *x = (NNNeuron *) malloc(sizeof(NNNeuron));
		InitializeNNNeuron(x, 0.0);
		vector_add( (vector *) pLayer->m_Neurons->Neurons, (void *) x );
		icNeurons++;
	}

	for ( ii = 0; ii < ((filterSize * filterSize + 1) * numFilters)*inputDepth; ++ii ) {
		initWeight = 0.05 * UNIFORM_PLUS_MINUS_ONE;
		NNWeight *nnw = (NNWeight *) malloc(sizeof(NNWeight));
		InitializeNNWeight(nnw, initWeight);
		vector_add((vector *) pLayer->m_Weights->Weights, (void *) nnw );
	}

	int kernelTemplate[filterSize * filterSize];

	for ( i = 0; i < filterSize; ++i) {
		for ( j = 0; j < filterSize; ++j)
			kernelTemplate[filterSize * i + j] = inputHeight * i + j;
	}

	for ( fm = 0; fm < numFilters; ++fm) {
		for ( ii = 0; ii < outputHeight; ++ii ) {
			for ( jj = 0; jj < outputHeight; ++jj ) {
				iNumWeight = fm * (filterSize * filterSize + 1) ;  // 26 is the number of weights per feature map
				NNNeuron *n = (NNNeuron *) vector_get( (vector *)pLayer->m_Neurons->Neurons ,
				                                       jj + ii * outputHeight + fm * outputHeight * outputHeight );
				AddConnectionNNNeuron(n, ULONG_MAX, iNumWeight++); // bias weight

				for ( kk = 0; kk < 25; ++kk )
					for ( i = 0; i < inputDepth; ++i)
						AddConnectionNNNeuron( n, i * inputHeight * inputHeight + 2 * jj + 2 * inputHeight * ii +
						                       kernelTemplate[kk], iNumWeight++);
			}
		}
	}
}

void createFCLayer(NNLayer *pLayer, int inputSize, int outputSize) {
	int ii, jj, kk, i, j, fm;
	int icNeurons = 0;
	double initWeight;
	unsigned int iNumWeight;

	for ( ii = 0; ii < outputSize; ++ii ) {
		NNNeuron *x = (NNNeuron *) malloc(sizeof(NNNeuron));
		InitializeNNNeuron(x, 0.0);
		vector_add( (vector *) pLayer->m_Neurons->Neurons, (void *) x );
		icNeurons++;
	}

	for ( ii = 0; ii < (inputSize + 1)*outputSize; ++ii ) {
		initWeight = 0.05 * UNIFORM_PLUS_MINUS_ONE;
		NNWeight *nnw = (NNWeight *) malloc(sizeof(NNWeight));
		InitializeNNWeight(nnw, initWeight);
		vector_add((vector *) pLayer->m_Weights->Weights, (void *) nnw );

	}

	iNumWeight = 0;  // weights are not shared in this layer

	for ( fm = 0; fm < outputSize; ++fm ) {
		NNNeuron *n2 = (NNNeuron *) vector_get( pLayer->m_Neurons->Neurons , fm );
		AddConnectionNNNeuron( n2, ULONG_MAX, iNumWeight++ );  // bias weight

		for ( ii = 0; ii < inputSize; ++ii )
			AddConnectionNNNeuron( n2, ii, iNumWeight++ );
	}
}

void InitializeCMNistDoc(CMNistDoc *cmnistdoc)
{
	cmnistdoc->m_cBackprops = 0;
	cmnistdoc->m_nAfterEveryNBackprops = 1;
	cmnistdoc->m_nItemsTrainingImages = 60000;
	cmnistdoc->m_nItemsTestImages = 10000;
	cmnistdoc->m_iNextTestingPattern = 0;
	cmnistdoc->m_iNextTrainingPattern = 0;
	cmnistdoc->g_cImageSize = 28;
	cmnistdoc->m_cCols = cmnistdoc->g_cImageSize + 1;
	cmnistdoc->m_cRows = cmnistdoc->g_cImageSize + 1;
	cmnistdoc->m_iWhichImageSet = 1;

	cmnistdoc->m_cCount = cmnistdoc->m_cCols * cmnistdoc->m_cRows;
    cmnistdoc->m_DispH = (double *) malloc(sizeof(double) * cmnistdoc->m_cCount);
    cmnistdoc->m_DispV = (double *) malloc(sizeof(double) * cmnistdoc->m_cCount);
	int iiMid = GAUSSIAN_FIELD_SIZE/2;  // GAUSSIAN_FIELD_SIZE is strictly odd
	Preferences *cp = (Preferences *) malloc(sizeof(Preferences));
        InitPreferences(cp);
	double twoSigmaSquared = 2.0 * (cp->m_dElasticSigma) * (cp->m_dElasticSigma);
	twoSigmaSquared = 1.0 /  twoSigmaSquared;
	double twoPiSigma = 1.0 / (cp->m_dElasticSigma) * sqrt( 2.0 * 3.1415926535897932384626433832795 );
	int row, col;
	for ( col=0; col<GAUSSIAN_FIELD_SIZE; ++col )
	{
		for ( row=0; row<GAUSSIAN_FIELD_SIZE; ++row )
		{
			cmnistdoc->m_GaussianKernel[ row ][ col ] = twoPiSigma *
				( exp(- ( ((row-iiMid)*(row-iiMid) + (col-iiMid)*(col-iiMid)) * twoSigmaSquared ) ) );
		}
	}


	cmnistdoc->m_fileTrainingImages = fopen("train-images.idx3-ubyte", "r");
	cmnistdoc->m_fileTrainingLabels = fopen("train-labels.idx1-ubyte", "r");
	cmnistdoc->m_fileTestingImages = fopen("t10k-images.idx3-ubyte", "r");
	cmnistdoc->m_fileTestingLabels = fopen("t10k-labels.idx1-ubyte", "r");

	cmnistdoc->m_NN = (NeuralNetwork *) malloc(sizeof(NeuralNetwork));
}

bool OnNewDocument(CMNistDoc *cmnistdoc)
{
	NeuralNetwork *NN = (NeuralNetwork *) cmnistdoc->m_NN;  // for easier nomenclature
	InitializeNN(NN);

	//Malloc and Initiallize layers

	NNLayer *pLayer = (NNLayer *) malloc(sizeof(NNLayer));
	InitializeNNLayer(pLayer);
	vector_add((vector *) NN->m_Layers->Layers, (void *) pLayer );

	NNLayer *pLayer1 = (NNLayer *) malloc(sizeof(NNLayer)) ;
	pLayer1 = InitializeNNLayerWithPrevLayer(pLayer1, pLayer);
	vector_add((vector *) NN->m_Layers->Layers, (void *) pLayer1 );

	NNLayer *pLayer2 = (NNLayer *) malloc(sizeof(NNLayer)) ;
	pLayer2 = InitializeNNLayerWithPrevLayer(pLayer2, pLayer1);
	vector_add((vector *) NN->m_Layers->Layers, (void *) pLayer2);

	NNLayer *pLayer3 = (NNLayer *) malloc(sizeof(NNLayer)) ;
	pLayer3 = InitializeNNLayerWithPrevLayer(pLayer3, pLayer2);
	vector_add((vector *) NN->m_Layers->Layers, (void *) pLayer3);

	NNLayer *pLayer4 = (NNLayer *) malloc(sizeof(NNLayer)) ;
	pLayer4 = InitializeNNLayerWithPrevLayer(pLayer4, pLayer3);
	vector_add((vector *) NN->m_Layers->Layers, (void *) pLayer4);

	// NNLayer *pLayer5 = (NNLayer *) malloc(sizeof(NNLayer)) ;
	// pLayer5 = InitializeNNLayerWithPrevLayer(pLayer5, pLayer4);
	// vector_add((vector *) NN->m_Layers->Layers, (void *) pLayer5);


	// layer zero, the input layer.
	// Create neurons: exactly the same number of neurons as the input
	// vector of 29x29=841 pixels, and no weights/connections
	createInputLayer(pLayer, 29, 1);
	printf("layer in\n");
	// layer one:
	// This layer is a convolutional layer that has 6 feature maps.  Each feature
	// map is 13x13, and each unit in the feature maps is a 5x5 convolutional kernel
	// of the input layer.
	// So, there are 13x13x6 = 1014 neurons, (5x5+1)x6 = 156 weights
	createConvolutionLayer(pLayer1, 29, 1, 13, 5, 6);
	printf("layer c1\n");
	// layer two:
	// This layer is a convolutional layer that has 50 feature maps.  Each feature
	// map is 5x5, and each unit in the feature maps is a 5x5 convolutional kernel
	// of corresponding areas of all 6 of the previous layers, each of which is a 13x13 feature map
	// So, there are 5x5x50 = 1250 neurons, (5x5+1)x6x50 = 7800 weights
	createConvolutionLayer(pLayer2, 13, 6, 5, 5, 50);
	printf("layer c2\n");
	// layer three:
	// This layer is a fully-connected layer with 100 units.  Since it is fully-connected,
	// each of the 100 neurons in the layer is connected to all 1250 neurons in
	// the previous layer.
	// So, there are 100 neurons and 100*(1250+1)=125100 weights
	createFCLayer(pLayer3, 1250, 100);
	printf("layer fc1\n");
	// layer four, the final (output) layer:
	// This layer is a fully-connected layer with 10 units.  Since it is fully-connected,
	// each of the 10 neurons in the layer is connected to all 100 neurons in
	// the previous layer.
	// So, there are 10 neurons and 10*(100+1)=1010 weights
	createFCLayer(pLayer4, 100, 10);
	printf("layer fc2\n");

	// createFCLayer(pLayer5, 100, 10);
	// printf("layer fc3\n");
	/*  printf("%d\n", vector_total(NN->m_Layers->Layers));
	    for (fm  = 0 ; fm < vector_total(NN->m_Layers->Layers) ; ++fm )
	    {
		NNLayer *nnl = (NNLayer *) vector_get((vector *) NN->m_Layers->Layers , fm);
		printf("%d\n", vector_total(nnl->m_Neurons->Neurons));
	    }*/
	//SetModifiedFlag( true );
	cmnistdoc->m_NN = NN;
	return true;
}
double GetCurrentEta(CMNistDoc *cmnistdoc)
{
	return cmnistdoc->m_NN->m_etaLearningRate;
}


double GetPreviousEta(CMNistDoc *cmnistdoc)
{
return cmnistdoc->m_NN->m_etaLearningRatePrevious;
}


unsigned int GetCurrentTrainingPatternNumber( CMNistDoc *cmnistdoc, bool bFromRandomizedPatternSequence )
{
unsigned int iRet;

	if ( bFromRandomizedPatternSequence == false )
	{
		iRet = cmnistdoc->m_iNextTrainingPattern;
	}
	else
	{
		iRet = cmnistdoc->m_iRandomizedTrainingPatternSequence[ cmnistdoc->m_iNextTrainingPattern ];
	}

	return iRet;
}






void RandomizeTrainingPatternSequence(CMNistDoc* cmnistdoc)
{
unsigned int ii, jj, iiMax, iiTemp;

	iiMax = cmnistdoc->m_nItemsTrainingImages;

	for ( ii=0; ii<iiMax; ++ii )
	{
		cmnistdoc->m_iRandomizedTrainingPatternSequence[ ii ] = ii;
	}


	for ( ii = iiMax-1; ii > 0; ii--)
    	{
        	jj = rand() % (ii+1);
         	iiTemp = cmnistdoc->m_iRandomizedTrainingPatternSequence[ ii ];
		cmnistdoc->m_iRandomizedTrainingPatternSequence[ ii ] = cmnistdoc->m_iRandomizedTrainingPatternSequence[ jj ];
		cmnistdoc->m_iRandomizedTrainingPatternSequence[ jj ] = iiTemp;
    	}	

}
unsigned int GetNextTrainingPattern(CMNistDoc *cmnistdoc, unsigned char *pArray , int *pLabel ,
									   bool bFlipGrayscale, bool bFromRandomizedPatternSequence , unsigned int* iSequenceNum )
{
	unsigned int iPatternNum;

	if ( bFromRandomizedPatternSequence == false )
	{
		iPatternNum = cmnistdoc->m_iNextTrainingPattern;
	}
	else
	{
		iPatternNum = cmnistdoc->m_iRandomizedTrainingPatternSequence[ cmnistdoc->m_iNextTrainingPattern ];
	}
	GetTrainingPatternArrayValues(cmnistdoc, iPatternNum, pArray, pLabel, bFlipGrayscale );

	if ( iSequenceNum != NULL )
	{
		*iSequenceNum = cmnistdoc->m_iNextTrainingPattern;
	}

	cmnistdoc->m_iNextTrainingPattern++;

	if ( cmnistdoc->m_iNextTrainingPattern >= cmnistdoc->m_nItemsTrainingImages )
	{
		cmnistdoc->m_iNextTrainingPattern = 0;
	}

	return iPatternNum;
}

void GetTrainingPatternArrayValues(CMNistDoc* cmnistdoc, int iNumImage , unsigned char *pArray , int *pLabel ,  bool bFlipGrayscale)
{
	int cCount = (cmnistdoc->g_cImageSize)*(cmnistdoc->g_cImageSize);
	int fPos, ii;


	if ( pArray != NULL )
	{
		fPos = 16 + iNumImage*cCount;  // 16 compensates for file header info
		fseek( cmnistdoc->m_fileTrainingImages , fPos, SEEK_SET );
		fread( pArray , sizeof(unsigned char) , cCount , cmnistdoc->m_fileTrainingImages );

		if ( bFlipGrayscale != false )
		{
			for (ii=0; ii<cCount; ++ii )
			{
				pArray[ ii ] = 255 - pArray[ ii ];
			}
		}
	}

	if ( pLabel != NULL )
	{
		fPos = 8 + iNumImage;
		char r;
		fseek( cmnistdoc->m_fileTrainingLabels , fPos, SEEK_SET );
		fread( &r , sizeof(unsigned char) , 1 ,cmnistdoc->m_fileTrainingLabels );
		// single byte
		*pLabel = r;
	}
}

unsigned int GetRandomTrainingPattern(CMNistDoc* cmnistdoc, unsigned char *pArray , int *pLabel, bool bFlipGrayscale )
{
	unsigned int patternNum = (unsigned int)( UNIFORM_ZERO_THRU_ONE * (cmnistdoc->m_nItemsTrainingImages - 1) );

	GetTrainingPatternArrayValues( cmnistdoc, patternNum, pArray, pLabel, bFlipGrayscale );

	return patternNum;
}

unsigned int GetNextTestingPatternNumber(CMNistDoc *cmnistdoc)
{
	return cmnistdoc->m_iNextTestingPattern;
}

void GetTestingPatternArrayValues(CMNistDoc *cmnistdoc, int iNumImage, unsigned char *pArray , int *pLabel , bool bFlipGrayscale )
{
	int cCount = (cmnistdoc->g_cImageSize)*(cmnistdoc->g_cImageSize);
	int fPos, ii;
	if ( pArray != NULL )
	{
		fPos = 16 + iNumImage*cCount;  // 16 compensates for file header info
		fseek( cmnistdoc->m_fileTestingImages , fPos, SEEK_SET );
		fread( pArray , sizeof(unsigned char) , cCount , cmnistdoc->m_fileTestingImages );

		if ( bFlipGrayscale != false )
		{
			for (  ii=0; ii<cCount; ++ii )
			{
				pArray[ ii ] = 255 - pArray[ ii ];
			}
		}
	}

	if ( pLabel != NULL )
	{
		fPos = 8 + iNumImage;
		char r;
		fseek( cmnistdoc->m_fileTestingLabels , fPos, SEEK_SET );
		fread( &r , sizeof(unsigned char) , 1 , cmnistdoc->m_fileTestingLabels );
		// single byte
		*pLabel = r;
	}
}

unsigned int GetNextTestingPattern(CMNistDoc *cmnistdoc, unsigned char *pArray, int *pLabel, bool bFlipGrayscale )
{
	GetTestingPatternArrayValues( cmnistdoc, cmnistdoc->m_iNextTestingPattern, pArray, pLabel, bFlipGrayscale );

	unsigned int iRet = cmnistdoc->m_iNextTestingPattern;
	cmnistdoc->m_iNextTestingPattern++;

	if ( cmnistdoc->m_iNextTestingPattern >= cmnistdoc->m_nItemsTestImages )
	{
		cmnistdoc->m_iNextTestingPattern = 0;
	}

	return iRet ;
}

double *At( CMNistDoc *cmnistdoc, double* p, int row, int col )  // zero-based indices, starting at bottom-left
{
    int location = row * cmnistdoc->m_cCols + col;
    double ans;
    ans = p[ location ] ;
    double *pt;
    pt = &ans;
    return pt;
}


void GenerateDistortionMap( CMNistDoc* cmnistdoc, double severityFactor , Preferences *preferences )
{
	int row, col;
	double* uniformH = (double *) malloc(sizeof(double) * cmnistdoc->m_cCount);
	double* uniformV = (double *) malloc(sizeof(double) * cmnistdoc->m_cCount);
	InitPreferences(preferences);

	for ( col=0; col<cmnistdoc->m_cCols; ++col )
	{
		for ( row=0; row<cmnistdoc->m_cRows; ++row )
		{
			int location = row * cmnistdoc->m_cCols + col;
			uniformH[location] = UNIFORM_PLUS_MINUS_ONE;
			uniformV[location] = UNIFORM_PLUS_MINUS_ONE;

		}
	}

	double fConvolvedH, fConvolvedV;
	double fSampleH, fSampleV;
	double elasticScale = severityFactor * preferences->m_dElasticScaling;
	int xxx, yyy, xxxDisp, yyyDisp;
	int iiMid = GAUSSIAN_FIELD_SIZE/2;  // GAUSSIAN_FIELD_SIZE is strictly odd

	for ( col=0; col<cmnistdoc->m_cCols; ++col )
	{
		for ( row=0; row<cmnistdoc->m_cRows; ++row )
		{
			fConvolvedH = 0.0;
			fConvolvedV = 0.0;

			for ( xxx=0; xxx<GAUSSIAN_FIELD_SIZE; ++xxx )
			{
				for ( yyy=0; yyy<GAUSSIAN_FIELD_SIZE; ++yyy )
				{
					xxxDisp = col - iiMid + xxx;
					yyyDisp = row - iiMid + yyy;

					if ( xxxDisp<0 || xxxDisp>=cmnistdoc->m_cCols || yyyDisp<0 || yyyDisp>=cmnistdoc->m_cRows )
					{
						fSampleH = 0.0;
						fSampleV = 0.0;
					}
					else
					{
						int location = yyyDisp * cmnistdoc->m_cCols + xxxDisp;
						fSampleH = uniformH[ location ];
						fSampleV = uniformV[ location ]; 
					}

					fConvolvedH += fSampleH * cmnistdoc->m_GaussianKernel[ yyy ][ xxx ];
					fConvolvedV += fSampleV * cmnistdoc->m_GaussianKernel[ yyy ][ xxx ];
				}
			}
			int location = row * cmnistdoc->m_cCols + col;
			cmnistdoc->m_DispH[ location ] = elasticScale * fConvolvedH;
			cmnistdoc->m_DispV[ location ] = elasticScale * fConvolvedV;
		}
	}
	double dSFHoriz = severityFactor * (preferences->m_dMaxScaling) / 100.0 * UNIFORM_PLUS_MINUS_ONE;  // m_dMaxScaling is a percentage
	double dSFVert = severityFactor * (preferences->m_dMaxScaling) / 100.0 * UNIFORM_PLUS_MINUS_ONE;  // m_dMaxScaling is a percentage


	int iMid = (cmnistdoc->m_cRows)/2;

	for ( row=0; row<cmnistdoc->m_cRows; ++row )
	{
		for ( col=0; col<cmnistdoc->m_cCols; ++col )
		{
			int location = row * cmnistdoc->m_cCols + col;
			cmnistdoc->m_DispH[ location ] += dSFHoriz * ( col-iMid );
			cmnistdoc->m_DispV[ location ] -= dSFVert * ( iMid-row );  // negative because of top-down bitmap
		}
	}

	double angle = severityFactor * preferences->m_dMaxRotation * UNIFORM_PLUS_MINUS_ONE;
	angle = angle * 3.1415926535897932384626433832795 / 180.0;  // convert from degrees to radians

	double cosAngle = cos( angle );
	double sinAngle = sin( angle );

	for ( row=0; row<cmnistdoc->m_cRows; ++row )
	{
		for ( col=0; col<cmnistdoc->m_cCols; ++col )
		{
			int location = row * cmnistdoc->m_cCols + col;
			cmnistdoc->m_DispH[ location ] += ( col-iMid ) * ( cosAngle - 1 ) - ( iMid-row ) * sinAngle;
			cmnistdoc->m_DispV[ location ] -= ( iMid-row ) * ( cosAngle - 1 ) + ( col-iMid ) * sinAngle;  // negative because of top-down bitmap

		}
	}

}
void ApplyDistortionMap(CMNistDoc* cmnistdoc, double *inputVector)
{
	double **mappedVector = (double **) malloc(sizeof(double *) * cmnistdoc->m_cRows);
	int i;
	for (i = 0; i < cmnistdoc->m_cRows ; i++)
	{
		mappedVector[i] = (double *) malloc(sizeof(double) * cmnistdoc->m_cCols);
	}
	double sourceRow, sourceCol;
	double fracRow, fracCol;
	double w1, w2, w3, w4;
	double sourceValue;
	int row, col;
	int sRow, sCol, sRowp1, sColp1;
	bool bSkipOutOfBounds;

	for ( row=0; row<cmnistdoc->m_cRows; ++row )
	{
		for ( col=0; col<cmnistdoc->m_cCols; ++col )
		{
			int location = row * cmnistdoc->m_cCols + col;
			sourceRow = (double)row - cmnistdoc->m_DispV[ location]; 
			sourceCol = (double)col - cmnistdoc->m_DispH[ location ];

			fracRow = sourceRow - (int)sourceRow;
			fracCol = sourceCol - (int)sourceCol;
			w1 = ( 1.0 - fracRow ) * ( 1.0 - fracCol );
			w2 = ( 1.0 - fracRow ) * fracCol;
			w3 = fracRow * ( 1 - fracCol );
			w4 = fracRow * fracCol;

			bSkipOutOfBounds = false;

			if ( (sourceRow + 1.0) >= cmnistdoc->m_cRows )	bSkipOutOfBounds = true;
			if ( sourceRow < 0 )				bSkipOutOfBounds = true;

			if ( (sourceCol + 1.0) >= cmnistdoc->m_cCols )	bSkipOutOfBounds = true;
			if ( sourceCol < 0 )				bSkipOutOfBounds = true;

			if ( bSkipOutOfBounds == false )
			{
				sRow = (int)sourceRow;
				sCol = (int)sourceCol;

				sRowp1 = sRow + 1;
				sColp1 = sCol + 1;

				while (sRowp1 >= cmnistdoc->m_cRows ) sRowp1 -= cmnistdoc->m_cRows;
				while (sRowp1 < 0 ) sRowp1 += cmnistdoc->m_cRows;

				while (sColp1 >= cmnistdoc->m_cCols ) sColp1 -= cmnistdoc->m_cCols;
				while (sColp1 < 0 ) sColp1 += cmnistdoc->m_cCols;

				int loc1 = sRow * cmnistdoc->m_cCols + sCol ; 
				int loc2 = sRow * cmnistdoc->m_cCols + sColp1 ;
				int loc3 = sRowp1 * cmnistdoc->m_cCols + sCol ; 
				int loc4 = sRowp1 * cmnistdoc->m_cCols + sColp1 ;
 				sourceValue = w1 * inputVector[ loc1 ] + w2 * inputVector[ loc2 ] + w3 * inputVector[ loc3 ] + w4 * inputVector[ loc4 ];
			}
			else
			{
				sourceValue = 1.0;  // "background" color in the -1 -> +1 range of inputVector
			}

			mappedVector[ row ][ col ] = 0.5 * ( 1.0 - sourceValue );  // conversion to 0->1 range we are using for mappedVector

		}
	}
	for ( row=0; row<cmnistdoc->m_cRows; ++row )
	{
		for ( col=0; col<cmnistdoc->m_cCols; ++col )
		{
			int location = row * cmnistdoc->m_cCols + col;
			inputVector[ location ] = 1.0 - 2.0 * mappedVector[ row ][ col ];
		}
	}

}
void CalculateNeuralNet(CMNistDoc* cmnistdoc, double *inputVector, int count,
								   double* outputVector , int oCount ,
								   vector** pNeuronOutputs,
								   bool bDistort )
{
	if ( bDistort != false )
	{
		Preferences p;
		GenerateDistortionMap(cmnistdoc, 1.0, &p );
		ApplyDistortionMap( cmnistdoc,inputVector );
	}


	CalculateNN( cmnistdoc->m_NN,inputVector, count, outputVector, oCount, pNeuronOutputs );

}

void CalculateHessian(CMNistDoc* cmnistdoc , Preferences *p)
{
        InitPreferences(p);
	double inputVector[841] = {0.0};  // note: 29x29, not 28x28
	double targetOutputVector[10] = {0.0};
	double actualOutputVector[10] = {0.0};

	unsigned char grayLevels[784] = { 0 };
	int label = 0;
	int ii, jj;
	unsigned int kk;

	EraseHessianInformationNN(cmnistdoc->m_NN);

	unsigned int numPatternsSampled = p->m_nNumHessianPatterns ;

	for ( kk=0; kk<numPatternsSampled; ++kk )
	{
		GetRandomTrainingPattern( cmnistdoc, grayLevels, &label, true );

		if ( label < 0 ) label = 0;
		if ( label > 9 ) label = 9;



		for ( ii=0; ii<841; ++ii )
		{
			inputVector[ ii ] = 1.0;  // one is white, -one is black
		}
		for ( ii=0; ii<p->g_cImageSize; ++ii )
		{
			for ( jj=0; jj<p->g_cImageSize; ++jj )
			{
				inputVector[ 1 + jj + 29*(ii+1) ] = (double)((int)(unsigned char)grayLevels[ jj + (p->g_cImageSize)*ii ])/128.0 - 1.0;  // one is white, -one is black
			}
		}

		for ( ii=0; ii<10; ++ii )
		{
			targetOutputVector[ ii ] = -1.0;
		}
		targetOutputVector[ label ] = 1.0;

		GenerateDistortionMap( cmnistdoc, 0.65 , p);
		ApplyDistortionMap( cmnistdoc, inputVector );

		CalculateNN( cmnistdoc->m_NN, inputVector, 841, actualOutputVector, 10, NULL );
		BackpropagateSecondDervativesNN( cmnistdoc->m_NN, actualOutputVector, targetOutputVector, 10 );

	}

	DivideHessianInformationByNN( cmnistdoc->m_NN, (double) numPatternsSampled );
}


void BackpropagateNeuralNet(CMNistDoc *cmnistdoc, double *inputVector, int iCount, double* targetOutputVector, double* actualOutputVector, int oCount, vector** pMemorizedNeuronOutputs, bool bDistort , Preferences *preferences)
{
	bool bWorthwhileToBackpropagate;  /////// part of code review
	InitPreferences(preferences);
	if ( ((( cmnistdoc->m_cBackprops) % (cmnistdoc->m_nAfterEveryNBackprops )) == 0) && (cmnistdoc->m_cBackprops != 0) )
		{
			double eta = cmnistdoc->m_NN->m_etaLearningRate;
			eta *= cmnistdoc->m_dEtaDecay;
			if ( eta < cmnistdoc->m_dMinimumEta )
				eta = cmnistdoc->m_dMinimumEta;
			cmnistdoc->m_NN->m_etaLearningRatePrevious = cmnistdoc->m_NN->m_etaLearningRate;
			cmnistdoc->m_NN->m_etaLearningRate = eta;
		}

		if ( (cmnistdoc->m_bNeedHessian != false) || (( cmnistdoc->m_cBackprops % preferences->m_nItemsTrainingImages ) == 0) )
		{
			CalculateHessian(cmnistdoc , preferences);

			cmnistdoc->m_bNeedHessian = false;
		}


		if ( ( cmnistdoc->m_cBackprops % preferences->m_nItemsTrainingImages ) == 0 )
		{
			RandomizeTrainingPatternSequence(cmnistdoc );
		}

		cmnistdoc->m_cBackprops++;
		CalculateNeuralNet( cmnistdoc ,inputVector, iCount, actualOutputVector, oCount, pMemorizedNeuronOutputs, bDistort );
		int ii;
		double dMSE = 0.0;
		for (  ii=0; ii<10; ++ii )
		{
			dMSE += ( actualOutputVector[ii]-targetOutputVector[ii] ) * ( actualOutputVector[ii]-targetOutputVector[ii] );
		}
		dMSE /= 2.0;

		if ( dMSE <= ( 0.10 * cmnistdoc->m_dEstimatedCurrentMSE ) )
		{
			bWorthwhileToBackpropagate = false;
		}
		else
		{
			bWorthwhileToBackpropagate = true;
		}


		if ( (bWorthwhileToBackpropagate != false) && (pMemorizedNeuronOutputs == NULL) )
		{
			BackpropagateNN( cmnistdoc->m_NN, actualOutputVector, targetOutputVector, oCount, NULL );
			return;
		}
	if (bWorthwhileToBackpropagate != false )
	{
		BackpropagateNN( cmnistdoc->m_NN, actualOutputVector, targetOutputVector, oCount, pMemorizedNeuronOutputs );
	}

}

unsigned int BackpropagationThread(CMNistDoc* cmnistdoc , Preferences *p)
{
	double inputVector[841] = {0.0};  // note: 29x29, not 28x28
	double targetOutputVector[10] = {0.0};
	double actualOutputVector[10] = {0.0};
	double dMSE;
	unsigned int scaledMSE;
	//unsigned char grayLevels[(p->g_cImageSize) * (p->g_cImageSize)] = { 0 };
	unsigned char grayLevels[784] = { 0 };
	int label = 0;
	int ii, jj, rSize;
	unsigned int iSequentialNum;
	int iSize = vector_total((vector *) cmnistdoc->m_NN->m_Layers->Layers);
	vector memorizedNeuronOutputs;
	vector_init(&memorizedNeuronOutputs);
	vector_resize(&memorizedNeuronOutputs, iSize);
	for( ii = 0 ; ii< iSize ; ii++ ){
		NNLayer *nnl = (NNLayer *) vector_get((vector *) cmnistdoc->m_NN->m_Layers->Layers, ii);
		rSize = vector_total((vector *) nnl->m_Neurons->Neurons);
		vector vlayer;
		vector_init(&vlayer);
		vector_resize(&vlayer, rSize);
		vector_add(&memorizedNeuronOutputs , (void *) &vlayer);
        }
        int count = 0;
    
	printf ("Training: %d images\n", cmnistdoc->m_nItemsTrainingImages);
	
	while ( count<cmnistdoc->m_nItemsTrainingImages)
	{
	    count++;
		int iRet = GetNextTrainingPattern( cmnistdoc, grayLevels, &label, true, true, &iSequentialNum );

		if ( label < 0 ) label = 0;
		if ( label > 9 ) label = 9;
		for ( ii=0; ii<841; ++ii )
		{
			inputVector[ ii ] = 1.0;  // one is white, -one is black
		}

		for ( ii=0; ii<p->g_cImageSize; ++ii )
		{
			for ( jj=0; jj<p->g_cImageSize; ++jj )
			{
				inputVector[ 1 + jj + 29*(ii+1) ] = (double)((int)(unsigned char)grayLevels[ jj + (p->g_cImageSize)*ii ])/128.0 - 1.0;  // one is white, -one is black
			}
		}
		for ( ii=0; ii<10; ++ii )
		{
			targetOutputVector[ ii ] = -1.0;
		}
		targetOutputVector[ label ] = 1.0;
		BackpropagateNeuralNet( cmnistdoc, inputVector, 841, targetOutputVector, actualOutputVector, 10, &memorizedNeuronOutputs, false, p );
		dMSE = 0.0;
		for ( ii=0; ii<10; ++ii )
		{
			dMSE += ( actualOutputVector[ii]-targetOutputVector[ii] ) * ( actualOutputVector[ii]-targetOutputVector[ii] );
		}
		dMSE /= 2.0;

		scaledMSE = (unsigned int)( sqrt( dMSE ) * 2.0e8 );  // arbitrary large pre-agreed upon scale factor; taking sqrt is simply to improve the scaling
		int iBestIndex = 0;
		double maxValue = -99.0;

		for ( ii=0; ii<10; ++ii )
		{
			if ( actualOutputVector[ ii ] > maxValue )
			{
				iBestIndex = ii;
				maxValue = actualOutputVector[ ii ];
			}
		}

	}  // end of main "while not abort flag" loop

	return 0L;
}




bool StartBackpropagation(CMNistDoc *cmnistdoc, unsigned int iStartPattern , unsigned int iNumThreads ,	 double initialEta, double minimumEta  , double etaDecay, unsigned int nAfterEvery , bool bDistortPatterns , double estimatedCurrentMSE , Preferences *preferences)
{
	cmnistdoc->m_cBackprops = iStartPattern;
	cmnistdoc->m_bNeedHessian = true;
        InitPreferences(preferences);
	cmnistdoc->m_iNextTrainingPattern = iStartPattern;
	
	if ( cmnistdoc->m_iNextTrainingPattern < 0 )
		cmnistdoc->m_iNextTrainingPattern = 0;
	if ( cmnistdoc->m_iNextTrainingPattern >= preferences->m_nItemsTrainingImages )
		cmnistdoc->m_iNextTrainingPattern = preferences->m_nItemsTrainingImages - 1;

	cmnistdoc->m_NN->m_etaLearningRate = initialEta;
	cmnistdoc->m_NN->m_etaLearningRatePrevious = initialEta;
	cmnistdoc->m_dMinimumEta = minimumEta;
	cmnistdoc->m_dEtaDecay = etaDecay;
	cmnistdoc->m_nAfterEveryNBackprops = nAfterEvery;
	cmnistdoc->m_bDistortTrainingPatterns = bDistortPatterns;

	cmnistdoc->m_dEstimatedCurrentMSE = estimatedCurrentMSE;  // estimated number that will define whether a forward calculation's error is significant enough to warrant backpropagation

	RandomizeTrainingPatternSequence(cmnistdoc );
	BackpropagationThread(cmnistdoc , preferences);


	return true;

}
unsigned int TestingThread(CMNistDoc *cmnistdoc , Preferences *p)
{
	double inputVector[841] = {0.0};  // note: 29x29, not 28x28
	double targetOutputVector[10] = {0.0};
	double actualOutputVector[10] = {0.0};

	double dPatternMSE = 0.0;
	double dTotalMSE = 0.0;
	unsigned int scaledMSE = 0;
	unsigned int iPatternsProcessed = 0;
	unsigned char grayLevels[784] = { 0 };
	int label = 0;
	int ii, jj;
	unsigned int iPatNum, iSequentialNum;
	int count = 0;
    int nCorrect = 0;
	
	printf ("Testing: %d images\n", cmnistdoc->m_nItemsTestImages);
	// printf ("iPatNum, Prediction, Label, Correctness\n");
	while ( 1 )
	{
       		if ( cmnistdoc->m_iWhichImageSet == 1 )
		{
			if ( count >= cmnistdoc->m_nItemsTestImages ) break;

			if (count<cmnistdoc->m_nItemsTestImages)
            		{
                		iPatNum = GetNextTestingPattern( cmnistdoc, grayLevels, &label, true );
                		count++;
			}

		}
		else
		{
			if (count > cmnistdoc->m_nItemsTrainingImages ) break;
			// training set
			if (count<cmnistdoc->m_nItemsTrainingImages)
            		{
                		iPatNum = GetNextTrainingPattern( cmnistdoc, grayLevels, &label, true, false, &iSequentialNum );
                		count++;
			}

		}


		if ( label < 0 ) label = 0;
		if ( label > 9 ) label = 9;
		for ( ii=0; ii<841; ++ii )
		{
			inputVector[ ii ] = 1.0;  // one is white, -one is black
		}
		for ( ii=0; ii<p->g_cImageSize; ++ii )
		{
			for ( jj=0; jj<p->g_cImageSize; ++jj )
			{
				inputVector[ 1 + jj + 29*(ii+1) ] = (double)((int)(unsigned char)grayLevels[ jj + (p->g_cImageSize)*ii ])/128.0 - 1.0;  // one is white, -one is black
			}
		}

		for ( ii=0; ii<10; ++ii )
		{
			targetOutputVector[ ii ] = -1.0;
		}
		targetOutputVector[ label ] = 1.0;

		CalculateNeuralNet( cmnistdoc, inputVector, 841, actualOutputVector, 10, NULL, false );
       		dPatternMSE = 0.0;
		for ( ii=0; ii<10; ++ii )
		{
			dPatternMSE += ( actualOutputVector[ii]-targetOutputVector[ii] ) * ( actualOutputVector[ii]-targetOutputVector[ii] );
		}
		dPatternMSE /= 2.0;

		dTotalMSE += dPatternMSE;
		++iPatternsProcessed;
		int iBestIndex = 0;
		double maxValue = -99.0;
		unsigned int code;

		for ( ii=0; ii<10; ++ii )
		{
			if ( actualOutputVector[ ii ] > maxValue )
			{
				iBestIndex = ii;
				maxValue = actualOutputVector[ ii ];
			}
		}
		if ( iBestIndex != label )
		{
			code  = ( iPatNum    & 0x0003FFFF );
			code |= ( label      & 0x0000007F ) << 18;
			code |= ( iBestIndex & 0x0000007F ) << 25;
		}

        nCorrect += (iBestIndex == label) ? 1 : 0;

        // printf("%d:%d, %d, %d, %s\n", count, iPatNum, iBestIndex, label, (iBestIndex == label) ? "true" : "false");

	}
	double divisor = (double)( (iPatternsProcessed>1) ? iPatternsProcessed : 1 );
	dTotalMSE /= divisor;
	scaledMSE = (unsigned int)( sqrt( dTotalMSE ) * 2.0e8 );  // arbitrary large pre-agreed upon scale factor; taking sqrt is simply to improve the scaling

    printf("Test Accuracy: %.3lf\%\n", ((float)nCorrect*100) / ((float)cmnistdoc->m_nItemsTestImages));
	return 0L;

}



bool StartTesting(CMNistDoc *cmnistdoc, unsigned int iStartingPattern, unsigned int iNumThreads, bool bDistortPatterns, unsigned int iWhichImageSet)
{
	cmnistdoc->m_iNextTestingPattern = iStartingPattern;
	cmnistdoc->m_iWhichImageSet = iWhichImageSet;

	if ( cmnistdoc->m_iWhichImageSet > 1 )
		cmnistdoc->m_iWhichImageSet = 1;
	
	if ( cmnistdoc->m_iNextTestingPattern < 0 )
		cmnistdoc->m_iNextTestingPattern = 0;
	if ( cmnistdoc->m_iNextTestingPattern >= cmnistdoc->m_nItemsTestImages )
		cmnistdoc->m_iNextTestingPattern = cmnistdoc->m_nItemsTestImages - 1;

	cmnistdoc->m_bDistortTestingPatterns = bDistortPatterns;
    	Preferences p;
    	InitPreferences(&p);
	TestingThread(cmnistdoc , &p);
	return true;
}



#undef UNIFORM_ZERO_THRU_ONE
