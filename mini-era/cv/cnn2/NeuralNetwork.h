// NeuralNetwork.h: interface for the NeuralNetwork class.
#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H
//#define NULL ( (void *) 0)
#include <limits.h>
#include <math.h>
#include "vector.h"
#include <stdbool.h>
#include "Preferences.h"
#define SIGMOID(x) (1.7159*tanh(0.66666667*x))
#define DSIGMOID(S) (0.66666667/1.7159*(1.7159+(S))*(1.7159-(S)))  // derivative of the sigmoid as a function of the sigmoid's output

typedef struct VectorLayers{
	vector *Layers;
}VectorLayers;

typedef struct VectorWeights{
	vector *Weights;
}VectorWeights;

typedef struct VectorNeurons{
	vector *Neurons;
}VectorNeurons;

typedef struct VectorConnections{
	vector *Connections;
}VectorConnections;


//Struct Definition
typedef struct NeuralNetwork{
	volatile double m_etaLearningRatePrevious;
	volatile double m_etaLearningRate;
    volatile unsigned int m_cBackprops;
    // counter used in connection with Weight sanity check
    struct VectorLayers *m_Layers;
}NeuralNetwork;

void PeriodicWeightSanityCheckNN(NeuralNetwork *nn);
void CalculateNN(NeuralNetwork *nn, double* inputVector, unsigned int icount, double* outputVector, unsigned int oCount,vector **pNeuronOutputs );
void BackpropagateNN(NeuralNetwork *nn, double *actualOutput, double *desiredOutput, unsigned int count, vector **pMemorizedNeuronOutputs );
void EraseHessianInformationNN(NeuralNetwork *nn);
void DivideHessianInformationByNN(NeuralNetwork *nn, double divisor );
void BackpropagateSecondDervativesNN(NeuralNetwork *nn, double* actualOutputVector, double* targetOutputVector, unsigned int count );
void InitializeNN(NeuralNetwork *nn);
void DestroyNN(NeuralNetwork *nn);

typedef struct NNLayer{
	struct VectorWeights *m_Weights;
	struct VectorNeurons *m_Neurons;
	struct NNLayer *m_pPrevLayer;
	bool m_bFloatingPointWarning; }NNLayer;

void PeriodicWeightSanityCheckNNLayer(NNLayer *nnlayer);  // check if weights are "reasonable"
void CalculateNNLayer(NNLayer *nnlayer);
void BackpropagateNNLayer( NNLayer *nnlayer, vector *dErr_wrt_dXn , vector *dErr_wrt_dXnm1 , vector *thisLayerOutput, vector *prevLayerOutput, double etaLearningRate, Preferences *preferences );
void EraseHessianInformationNNLayer(NNLayer *nnlayer);
void DivideHessianInformationByNNLayer( NNLayer *nnlayer, double divisor );
void BackpropagateSecondDerivativesNNLayer( NNLayer *nnlayer, vector *dErr_wrt_dXn , vector *dErr_wrt_dXnm1 );
void InitializeNNLayer(NNLayer* nnlayer);
NNLayer* InitializeNNLayerWithPrevLayer(NNLayer* nnlayer, NNLayer *prevLayer);
void DestroyNNLayer(NNLayer* nnlayer);
typedef struct{
    unsigned int NeuronIndex, WeightIndex;
}NNConnection;
void InitializeNNConnection(NNConnection* nnc, unsigned int neuron , unsigned int weight );
void DestroyNNConnection(NNConnection* nnc);

typedef struct{
	double value;
	double diagHessian;
}NNWeight;

void InitializeNNWeight( NNWeight* nnw, double val );
void DestroyNNWeight(NNWeight* nnw);

typedef struct{
	double output;
	struct VectorConnections *m_Connections;
}NNNeuron;
void InitializeNNNeuron(NNNeuron *nnn , double out);
void InitializeNNNeuronClr(NNNeuron *nnn);
void AddConnectionNNNeuron( NNNeuron *nnn, unsigned int iNeuron, unsigned int iWeight );
void AddConnectionNNNeuronWithConn( NNNeuron *nnn, NNConnection *conn );
void DestroyNNNeuron(NNNeuron* nnn);
#endif
