// NeuralNetwork.cpp: implementation of the NeuralNetwork class.

#include "vector.h"
#include "NeuralNetwork.h"
#include <malloc.h>  // for the _alloca function
//#define NULL ( (void *) 0)
#include <stdbool.h>
#include <math.h>
#include "Preferences.h"
#include <limits.h>
void InitializeNN(NeuralNetwork *nn)
{
	int i;
	nn->m_etaLearningRate = .001;  // arbitrary, so that brand-new NNs can be serialized with a non-ridiculous number
	nn->m_Layers = (VectorLayers *) malloc(sizeof(VectorLayers));
	nn->m_Layers->Layers = (vector *) malloc(sizeof(vector));
	vector_init(nn->m_Layers->Layers);

	nn->m_cBackprops = 0;
}

void DestroyNN(NeuralNetwork *nn)
{
		InitializeNN(nn);
}

void CalculateNN(NeuralNetwork *nn, double* inputVector, unsigned int iCount, double* outputVector , unsigned int oCount , vector **pNeuronOutputs  )
{
    int lit,nit;
    lit = 0;
	if ( lit< vector_total((vector *) nn->m_Layers->Layers))
	{
		int count = 0;
		nit = 0;
		NNLayer *nnlay = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers,0);
		int sMax = vector_total((vector *) nnlay->m_Neurons->Neurons);
		while( nit < (sMax) && ( count < iCount ) )
		{
			NNNeuron *n = (NNNeuron *) vector_get((vector *)nnlay->m_Neurons->Neurons, nit);
			n->output = inputVector[ count ];
			nit++;
			count++;
		}
	}
	for( lit++; lit<vector_total((vector *) nn->m_Layers->Layers); lit++ )
	{
		NNLayer *nl = (NNLayer *) vector_get((vector *)nn->m_Layers->Layers,lit);
		//printf("%d",lit);
		CalculateNNLayer(nl);
	}
	int itcount;
	if ( outputVector != NULL )
	{
		lit = vector_total((vector *) nn->m_Layers->Layers);
		lit--;
        NNLayer *nll = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers, lit);
        itcount = 0;
	int ii;
		for ( ii=0; ii<oCount; ++ii )
		{
		    NNNeuron *nnr = (NNNeuron *) vector_get((vector *)nll->m_Neurons->Neurons, itcount);
			outputVector[ ii ] = nnr->output;
			itcount++;
		}
	}


	if ( pNeuronOutputs != NULL )
	{
		if ( vector_total((vector *) pNeuronOutputs) < 1 )
		{
			int ii = 0;
			for( lit= 0; lit<vector_total((vector *)nn->m_Layers->Layers); lit++ )
			{
				vector layerOut;
				vector_init(&layerOut);
                		NNLayer *n2 = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers, lit);
                		int s = vector_total((vector *) n2->m_Neurons->Neurons);
				for ( ii=0; ii<s; ++ii )
				{
				    NNNeuron *out = (NNNeuron *) vector_get((vector *) n2->m_Neurons->Neurons, ii);
				    double res = out->output;
				    double *p1;
				    p1 = &res;
				    vector_add(&layerOut, (void *) p1);
				}

				vector_add((vector *) pNeuronOutputs, (void *) &layerOut);
			}
		}
		else
		{
		int ii, jj = 0;
			for( lit=0; lit<vector_total((vector *) nn->m_Layers->Layers); lit++ )
			{
				vector layerOut;
				vector_init(&layerOut);
                		NNLayer *n3 = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers, lit);
				for ( ii=0; ii<vector_total((vector *)n3->m_Neurons->Neurons); ++ii )
				{
				    NNNeuron *nn2 = (NNNeuron *) vector_get(n3->m_Neurons->Neurons, ii);
				    //vector *row1 = vector_get((vector *) pNeuronOutputs, jj);
				    double *p;
				    p = &(nn2->output);
				    vector_add(&layerOut, (void *) p);
					//(*pNeuronOutputs)[ jj ][ ii ] = nn2->output;
				}
				vector_add((vector *)pNeuronOutputs, (void *) &layerOut);
				++jj;
			}

		}

	}

}


void BackpropagateNN(NeuralNetwork *nn, double *actualOutput, double *desiredOutput, unsigned int count,
								  vector** pMemorizedNeuronOutputs )
{
	if ( ( actualOutput == NULL ) || ( desiredOutput == NULL ) || ( count >= 256 ) )
		return;
	nn->m_cBackprops++;

	if ( (nn->m_cBackprops % 10000) == 0 )
	{
	PeriodicWeightSanityCheckNN(nn);
	}
	vector dErr_wrt_dXlast;
	vector_init(&dErr_wrt_dXlast);

	int iSize = vector_total((vector *)nn->m_Layers->Layers);
     
	NNLayer *bit = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers, iSize - 1);
	double **differentials=  (double **) malloc(sizeof(double *) * iSize);
	int ii;
	for ( ii=0; ii<iSize-1; ++ii )
	{
	    NNLayer *nnl1 = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers, ii);
	    differentials[ii] = (double *) malloc(sizeof(double) * vector_total((vector *) nnl1->m_Neurons->Neurons));
	}
	 
	for(ii=0; ii<vector_total((vector *) bit->m_Neurons->Neurons);++ii){
	    double r = actualOutput[ ii ] - desiredOutput[ ii ] ;
	    vector_set(&dErr_wrt_dXlast, ii,(void *) &r);
	}
   
    differentials[iSize - 1] = (double *) malloc(sizeof(double) *  vector_total((vector *) bit->m_Neurons->Neurons));
    int k;
    for (k = 0; k < vector_total(&dErr_wrt_dXlast);k++)
    {
        double *v1 = (double *) vector_get(&dErr_wrt_dXlast , k);
	differentials[iSize - 1][k] = (*v1);
    }
	bool bMemorized = ( pMemorizedNeuronOutputs != NULL );
        Preferences p;
        InitPreferences(&p);
	int lit;
	lit = vector_total((vector *)nn->m_Layers->Layers) - 1;  // re-initialized to last layer for clarity, although it should already be this value

	ii = iSize - 1;
	for ( lit; lit>0; lit--)
	{
	    NNLayer* nl = (NNLayer *) vector_get((vector *)nn->m_Layers->Layers, lit);
	    vector prev,cur;
            vector_init(&prev);
	    vector_init(&cur);
            NNLayer *nnl2 = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers, ii);
	    int maxS =  vector_total((vector *) nnl2->m_Neurons->Neurons);
            int ij;
	    for (ij = 0;ij<maxS;ij++)
	    {
 		double dl = differentials[ii][ij];
		double *pd;
    		pd = &dl;
		vector_add(&cur, (void *) pd); 
            }
	    NNLayer *nnl3 = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers, ii - 1);
	    maxS =  vector_total((vector *) nnl3->m_Neurons->Neurons);
	    for (ij = 0;ij<maxS;ij++)
	    {
 		double dl = differentials[ii - 1][ij];
		double *pd;
    		pd = &dl;
		vector_add(&prev, (void *) pd); 
            } 
            
		if ( bMemorized != false )
		{

		    BackpropagateNNLayer(nl, &cur, &prev, (vector *) (vector_get((vector *) pMemorizedNeuronOutputs,ii)), (vector *) ((vector *) vector_get((vector *) pMemorizedNeuronOutputs,ii-1)), nn->m_etaLearningRate, &p );
        }
		else
		{
			BackpropagateNNLayer( nl, &cur, &prev,NULL, NULL, nn->m_etaLearningRate,&p );
		}

		--ii;
	}
  free(differentials);
}


void PeriodicWeightSanityCheckNN(NeuralNetwork *nn)
{
    int lit;
	for ( lit=0; lit<vector_total((vector *)nn->m_Layers->Layers); lit++)
	{
        NNLayer *nnl = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers, lit);
		PeriodicWeightSanityCheckNNLayer(nnl);
	}

}


void EraseHessianInformationNN(NeuralNetwork* nn)
{
    int lit;
	for ( lit=0; lit<vector_total((vector *)nn->m_Layers->Layers); lit++ )
	{
        NNLayer *nnl = (NNLayer *) vector_get((vector *)nn->m_Layers->Layers, lit);
		EraseHessianInformationNNLayer(nnl);
	}

}


void DivideHessianInformationByNN(NeuralNetwork *nn, double divisor )
{
	int lit;
	for ( lit=0; lit<vector_total((vector *)nn->m_Layers->Layers); lit++ )
	{
	    NNLayer *nnl = (NNLayer *) vector_get((vector *)nn->m_Layers->Layers, lit);
		DivideHessianInformationByNNLayer( nnl, divisor );
	}

}


void BackpropagateSecondDervativesNN( NeuralNetwork *nn, double* actualOutputVector, double* targetOutputVector, unsigned int count )
{
	if ( ( actualOutputVector == NULL) || ( targetOutputVector == NULL) || (count >= 256 ) || (nn == NULL)) return;
	int lit;
	lit = vector_total((vector *) nn->m_Layers->Layers) - 1;
	NNLayer *nnlast = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers , lit);
	vector *d2Err_wrt_dXLast = (vector *) malloc(sizeof(double) * vector_total((vector *) nnlast->m_Neurons->Neurons));
	double **differentials = (double **) malloc(sizeof(double *) * (lit+1));
	int ii , jj , iSize, maxCol, curCol;
	iSize = lit+1;
	maxCol = 0;
	for ( ii = 0; ii < iSize;++ii)
	{
		NNLayer *nncur = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers , ii);
		curCol = vector_total((vector *) nncur->m_Neurons->Neurons);
		if (curCol > maxCol) maxCol = curCol;
	}
	for ( ii = 0 ; ii< iSize ; ++ii)
	{
		differentials[ ii ] = (double *) malloc(sizeof(double) * maxCol);
	}
	for ( ii = 0 ; ii< iSize ; ++ii)
	{
		for ( jj = 0; jj < maxCol ; ++jj )
		{
			differentials[ ii ][ jj ] = 0.0;
		}
	}
	for ( ii = 0 ; ii < vector_total((vector *) nnlast->m_Neurons->Neurons) ; ++ii )
	{
		double one = 1.0;
		double *onep;
		onep = &one;
		vector_set(d2Err_wrt_dXLast , ii , (void *) onep);
		differentials[ iSize - 1 ][ ii ] = 1.0;
	}
	ii = iSize - 1;
	lit = iSize - 1;
	for ( lit; lit > 0 ; lit-- )
	{
		NNLayer *nncur = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers , ii);
		curCol = vector_total((vector *) nncur->m_Neurons->Neurons);

		vector cur;
		vector_init(&cur);
		double one;
		double *onep;
		for ( jj = 0 ; jj < curCol ; jj++ )
		{
			one = differentials[ ii ][ jj ];
			onep = &one;
			vector_add(&cur , (void *) onep); 
		}
		NNLayer *nnprev = (NNLayer *) vector_get((vector *) nn->m_Layers->Layers , ii - 1);
		curCol = vector_total((vector *) nnprev->m_Neurons->Neurons);
		
		vector prev;
		vector_init(&prev);
		for ( jj = 0 ; jj < curCol ; ++jj )
		{
			one = differentials[ ii - 1 ][ jj ];
			onep = &one;
			vector_add(&prev , (void *) onep); 
		}
		BackpropagateSecondDerivativesNNLayer( nncur , &cur, &prev );
		--ii;
	}	




}

void InitializeNNLayer(NNLayer *nnlayer)
{
    nnlayer->m_Weights = (VectorWeights *) malloc(sizeof(VectorWeights));
    nnlayer->m_Neurons = (VectorNeurons *) malloc(sizeof(VectorNeurons));
    nnlayer->m_pPrevLayer = (NNLayer *) malloc(sizeof(NNLayer));
    nnlayer->m_Weights->Weights = (vector *) malloc(sizeof(vector));
    nnlayer->m_Neurons->Neurons = (vector *) malloc(sizeof(vector)); 
    vector_init(nnlayer->m_Weights->Weights);
    vector_init(nnlayer->m_Neurons->Neurons);
    int wit,nit;

	nnlayer->m_bFloatingPointWarning = false;

}

NNLayer* InitializeNNLayerWithPrevLayer(NNLayer* nnlayer, NNLayer* prevLayer){
    InitializeNNLayer(nnlayer);
    nnlayer->m_pPrevLayer = prevLayer;
        return nnlayer;
}

void DestroyNNLayer(NNLayer* nnlayer)
{

	InitializeNNLayer(nnlayer);
}

void CalculateNNLayer(NNLayer *nnlayer)
{
        int nit,cit;
	double dSum;
	for( nit=0; nit<vector_total((vector *) nnlayer->m_Neurons->Neurons); nit++ )
	{
        NNNeuron* n = (NNNeuron *) vector_get((vector *) nnlayer->m_Neurons->Neurons, nit);
		cit = 0;
        NNConnection* nnc = (NNConnection *) vector_get((vector *) n->m_Connections->Connections,0);
        dSum =  ((NNWeight *) vector_get((vector *)nnlayer->m_Weights->Weights, nnc->WeightIndex))->value;
		for ( cit++ ; cit<vector_total((vector *) n->m_Connections->Connections); cit++ )
		{
		    NNConnection *nnc1 = (NNConnection *) vector_get((vector *) n->m_Connections->Connections,cit);
		    double dsum2 = ((NNWeight *) vector_get((vector *) nnlayer->m_Weights->Weights, nnc1->WeightIndex))->value;
		    double dsum1 = ((NNNeuron *) vector_get((vector *) nnlayer->m_pPrevLayer->m_Neurons->Neurons, nnc1->NeuronIndex))->output;
		    dSum+=dsum2*dsum1;
		}

		n->output = SIGMOID( dSum );

	}

}



void BackpropagateNNLayer( NNLayer *nnlayer, vector *dErr_wrt_dXn ,vector *dErr_wrt_dXnm1 , vector* thisLayerOutput, 
                          vector* prevLayerOutput,  
                          double etaLearningRate, Preferences *preferences )
{
	int ii, jj;
	unsigned int kk;
	int nIndex;
	double output;

	vector dErr_wrt_dYn;
	vector_init(&dErr_wrt_dYn);
	vector_resize(&dErr_wrt_dYn, vector_total((vector *)nnlayer->m_Neurons->Neurons));

	double* dErr_wrt_dWn = malloc( sizeof(double) *  vector_total((vector *) nnlayer->m_Weights->Weights) );

	for ( ii=0; ii<vector_total((vector *) nnlayer->m_Weights->Weights); ++ii )
	{
		dErr_wrt_dWn[ ii ] =0.0;
	}
    int nit,cit;

	bool bMemorized = ( (vector_total(thisLayerOutput) != 0 ) && ( vector_total(prevLayerOutput) != 0 ));
	for ( ii=0; ii<vector_total((vector *) nnlayer->m_Neurons->Neurons); ++ii )
	{
		if ( bMemorized != false )
		{
			double *op = (double *) vector_get(thisLayerOutput, ii );
                        output = *op;
		}
		else
		{
			output = ((NNNeuron *) vector_get((vector *) nnlayer->m_Neurons->Neurons, ii ))->output;
		}

                double *op1 = (double *) vector_get(dErr_wrt_dXn,ii);
		double op2 = (*op1) * DSIGMOID(output);
		double *op3;
		op3 = &op2;
		vector_add(&dErr_wrt_dYn, (void *) op3);
	}
	ii = 0;
	for ( nit=0; nit<vector_total((vector *) nnlayer->m_Neurons->Neurons); nit++ )
	{
		NNNeuron *n = (NNNeuron *) vector_get((vector *) nnlayer->m_Neurons->Neurons, nit);  // for simplifying the terminology

		for ( cit=0; cit<vector_total((vector *) n->m_Connections->Connections); cit++ )
		{
		    kk = ((int)((NNConnection *) vector_get((vector *) n->m_Connections->Connections,cit))->NeuronIndex);
			if ( kk >=4294967200 )
			{
				output = 1.0;  // this is the bias weight
			}
			else
			{
				if ( bMemorized != false )
				{
					double *op2 = (double *) vector_get(prevLayerOutput , kk);
					output = *op2;
				}
				else
				{
					output = (double) ((NNNeuron *) vector_get((vector *) nnlayer->m_pPrevLayer->m_Neurons->Neurons, kk ))->output;
				}
			}

                        double *op4 = (double *) vector_get(&dErr_wrt_dYn , ii );
			dErr_wrt_dWn[ (int)(((NNConnection *) vector_get((vector *) n->m_Connections->Connections,cit))->WeightIndex )] += (*op4) * output;
		}

		ii++;
	}
	ii = 0;
	for ( nit=0; nit<vector_total((vector *) nnlayer->m_Neurons->Neurons); nit++ )
	{
		NNNeuron *n = (NNNeuron *) vector_get((vector *) nnlayer->m_Neurons->Neurons, nit);  // for simplifying the terminology

		for ( cit=0; cit<vector_total((vector *) n->m_Connections->Connections); cit++ )
		{
            kk = (int) (((NNConnection *) vector_get(n->m_Connections->Connections,cit))->NeuronIndex);
			if ( kk <=4294967200 )
			{
				nIndex = kk;
				int ind = (int) (((NNConnection *) vector_get((vector *) n->m_Connections->Connections,cit))->WeightIndex);
				double *r1 = (double *) vector_get(dErr_wrt_dXnm1, nIndex);
				double *r2 = (double *) vector_get(&dErr_wrt_dYn, ii);
				double r3 = ((double)((NNWeight *) vector_get((vector *) nnlayer->m_Weights->Weights , ind))->value);
				double result = (*r1) + ((*r2) * r3);
				double *r4;
				r4 = &result;
				vector_set(dErr_wrt_dXnm1 , nIndex ,(void *) r4);
            }
		}
		ii++;  // ii tracks the neuron iterator
	}

	
	InitPreferences(preferences);
	double dMicron = preferences->m_dMicronLimitParameter;
	double epsilon, divisor, oldValue, newValue;

	for ( jj=0; jj<vector_total((vector *) nnlayer->m_Weights->Weights); ++jj )
	{
		divisor = ((double) ((NNWeight *) vector_get((vector *) nnlayer->m_Weights->Weights , jj))->diagHessian) + dMicron ;
		epsilon = etaLearningRate / divisor;
		oldValue = ((double) ((NNWeight *) vector_get((vector *)nnlayer->m_Weights->Weights , jj))->value);
		newValue = oldValue - epsilon * dErr_wrt_dWn[ jj ];
		double *newp;
		newp = &newValue;
        	vector_set((vector *) nnlayer->m_Weights->Weights , jj , (void *) newp); 
	}

}


void PeriodicWeightSanityCheckNNLayer(NNLayer *nnlayer)
{
	int wit;

	for ( wit=0; wit<vector_total((vector *) nnlayer->m_Weights->Weights); wit++ )
	{
	    NNWeight *ww = (NNWeight *) vector_get((vector *) nnlayer->m_Weights->Weights, wit);
		double val = fabs( ww->value );

		if ( (val>100.0) && (nnlayer->m_bFloatingPointWarning == false) )
		{
			nnlayer->m_bFloatingPointWarning = true;
		}
	}
}



void EraseHessianInformationNNLayer(NNLayer *nnlayer)
{
    int wit;
	for ( wit=0; wit<vector_total((vector *) nnlayer->m_Weights->Weights); wit++ )
	{
	    NNWeight *nnw = (NNWeight *) vector_get((vector *) nnlayer->m_Weights->Weights, wit);
	    nnw->diagHessian = 0.0;
	}

}

void DivideHessianInformationByNNLayer(NNLayer *nnlayer, double divisor)
{
    int wit;
	double dTemp;

	for ( wit=0; wit<vector_total((vector *) nnlayer->m_Weights->Weights); wit++ )
	{
	    NNWeight *nw = (NNWeight *) vector_get((vector *) nnlayer->m_Weights->Weights, wit);
		dTemp = nw->diagHessian;

		if ( dTemp < 0.0 )
		{
		dTemp = 0.0;
		}

		nw->diagHessian = dTemp / divisor ;
	}
}


void BackpropagateSecondDerivativesNNLayer( NNLayer *nnlayer, vector *d2Err_wrt_dXn , vector *d2Err_wrt_dXnm1 )
{
	int ii, jj;
	unsigned int kk;
	int nIndex;
	double output;
	double dTemp;

	vector d2Err_wrt_dYn;
	vector_init(&d2Err_wrt_dYn);
	vector_resize(&d2Err_wrt_dYn, vector_total((vector *) nnlayer->m_Neurons->Neurons));
	double* d2Err_wrt_dWn = (double *) malloc( sizeof(double) *  vector_total((vector *) nnlayer->m_Weights->Weights));

	for ( ii=0; ii<vector_total((vector *) nnlayer->m_Weights->Weights); ++ii )
	{
		d2Err_wrt_dWn[ ii ] =0.0;
	}
    int nit,cit;
    double r;
    for ( ii=0; ii<vector_total((vector *) nnlayer->m_Neurons->Neurons); ++ii )
	{
  	        output = (double) ((NNNeuron *) vector_get((vector *) nnlayer->m_Neurons->Neurons ,  ii ))->output;
		//printf("%f\n", output);
		dTemp = DSIGMOID( output );
		double *pt1 = (double *) vector_get(d2Err_wrt_dXn , ii);
		r = dTemp * dTemp * (*pt1);
		vector_add(&d2Err_wrt_dYn ,  (void *) &r);
	}
	ii = 0;
	for ( nit=0; nit<vector_total((vector *) nnlayer->m_Neurons->Neurons); nit++ )
	{
		NNNeuron *n = (NNNeuron *) vector_get((vector *) nnlayer->m_Neurons->Neurons, nit);  // for simplifying the terminology

		for ( cit=0; cit<vector_total((vector *) n->m_Connections->Connections); cit++ )
		{
            NNConnection *nnc = (NNConnection *) vector_get((vector *) n->m_Connections->Connections, cit);
			kk = nnc->NeuronIndex;
			if ( kk >= 4294967200)
			{
				output = 1.0;  // this is the bias connection; implied neuron output of "1"
			}
			else
			{
			    output = ((NNNeuron *) vector_get((vector *) nnlayer->m_pPrevLayer->m_Neurons->Neurons , kk))->output;
			}
			double *pt3 = (double *) vector_get(&d2Err_wrt_dYn , ii);
			d2Err_wrt_dWn[ nnc->WeightIndex ] += (*pt3) * output * output ;
		}

		ii++;
	}
	ii = 0;
	for ( nit=0; nit<vector_total((vector *) nnlayer->m_Neurons->Neurons); nit++ )
	{
		NNNeuron *n = (NNNeuron *) vector_get((vector *) nnlayer->m_Neurons->Neurons, nit); // for simplifying the terminology

		for ( cit=0; cit<vector_total((vector *) n->m_Connections->Connections); cit++  )
		{
		    NNConnection *nnc = (NNConnection *) vector_get((vector *) n->m_Connections->Connections, cit);
			kk=nnc->NeuronIndex;
			if ( kk <= 4294967200 )
			{
				nIndex = kk;
				dTemp = ((NNWeight *)(vector_get(nnlayer->m_Weights->Weights, nnc->WeightIndex )))->value ;
                		double *pt5 = (double *) vector_get(d2Err_wrt_dXnm1 , nIndex);
				double *pt6 = (double *) vector_get(&d2Err_wrt_dYn , ii);
				r = (*pt5) + ((*pt6) * dTemp * dTemp);
				double *pt7;
				pt7 = &r;
                		vector_set(d2Err_wrt_dXnm1, nIndex, (void *)  pt7);
			}

		}

		ii++;  // ii tracks the neuron iterator

	}

	double oldValue, newValue;
	for ( jj=0; jj<vector_total((vector *) nnlayer->m_Weights->Weights); ++jj )
	{
		oldValue = ((NNWeight *)(vector_get((vector *) nnlayer->m_Weights->Weights,  jj )))->diagHessian;
		newValue = oldValue + d2Err_wrt_dWn[ jj ];
		double *newp;
		newp = &newValue;
		vector_set((vector*) nnlayer->m_Weights->Weights , jj , (void *) newp);
	}

}



void InitializeNNConnection(NNConnection* nnc, unsigned int neuron, unsigned int weight)
{
    nnc->NeuronIndex = neuron;
    nnc->WeightIndex = weight;
};
void InitializeNNWeight(NNWeight* nnw, double val)
{
    nnw->value = val;
    nnw->diagHessian = 0.0;
}
void DestroyNNWeight(NNWeight* nnw)
{

}

void InitializeNNNeuron(NNNeuron* nnn , double out)
{
    InitializeNNNeuronClr(nnn);
    nnn->output = out;
   }

void InitializeNNNeuronClr(NNNeuron* nnn)
{
	nnn->m_Connections = (VectorConnections *) malloc(sizeof(VectorConnections));
	nnn->m_Connections->Connections = (vector *) malloc(sizeof(vector));
	vector_init((vector *) nnn->m_Connections->Connections);
}

void DestroyNNNeuron(NNNeuron* nnn)
{
	InitializeNNNeuronClr(nnn);
}


void AddConnectionNNNeuron( NNNeuron *nnn,unsigned int iNeuron, unsigned int iWeight )
{
        NNConnection *nnc = (NNConnection *) malloc(sizeof(NNConnection));
        InitializeNNConnection(nnc,iNeuron,iWeight); 
        vector_add((vector *) nnn->m_Connections->Connections, (void *) nnc);
}

void AddConnectionNNNeuronWithConn( NNNeuron *nnn, NNConnection  *conn )
{
        vector_add((vector *) nnn->m_Connections->Connections,  (void *) conn );
}
