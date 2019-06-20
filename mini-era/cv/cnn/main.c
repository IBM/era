#include <stdio.h>
#include <stdlib.h>
#include "vector.h"
#include "NeuralNetwork.h"
#include "Preferences.h"
#include "MNistDoc.h"
#include <stdbool.h>
int main(void)
{
	printf("Initialize the Neural Network... \n");
	CMNistDoc *c = (CMNistDoc *) malloc(sizeof(CMNistDoc));
	InitializeCMNistDoc(c);
	OnNewDocument(c);
	Preferences p;
	InitPreferences(&p);
	printf("Start training...\n");
	StartBackpropagation(c , 0 , 1 , 0.001 , 0.00005 , 0.7941833 , 120000 , 1 , 0.1 , &p );
	printf("Start testing...\n");	
	StartTesting(c , 0 , 1 , false , 1); 	
}
