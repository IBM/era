/*
 * Copyright 2022 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include "/usr/include/python3.6m/Python.h"
//#include <opencv2/opencv.hpp>

#include "cv_toolset.h"
#include "globals.h"
#include "debug.h"

//using namespace cv;

PyObject *pName, *pModule, *pFunc, *pFunc_load;
PyObject *pArgs, *pValue, *pretValue;
//#define PY_SSIZE_T_CLEAN

char *python_module	= "yolo";
char *python_func	= "predict";
char *python_func_load	= "loadmodel";

#define INPUTS_PER_LABEL 20
char cv_inputs[num_object_labels][INPUTS_PER_LABEL][32]; // the names of input files by object type


status_t cv_toolset_init() {

  DBGOUT(printf("In the cv_toolset_init routine\n"));

  /** The CV kernel uses a different method to select appropriate inputs; dictionary not needed**/
  // Set up the CV/CNN input data file names (by object type label)
  for (int obj = 0; obj < num_object_labels; obj++) {
    for (int fn = 0; fn < INPUTS_PER_LABEL; fn++) {
      sprintf(cv_inputs[obj][fn], "cv_data/%04u_%u.jpg", 1000 * obj + fn, obj);
    }
  }

  Py_Initialize();
  pName = PyUnicode_DecodeFSDefault(python_module);
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);

  if (pModule == NULL) {
    PyErr_Print();
    printf("Failed to load Python program, perhaps pythonpath needs to be set; export PYTHONPATH=<your_era_dir>/src/cv/yolo\n");
    return 1;
  } else {
    pFunc_load = PyObject_GetAttrString(pModule, python_func_load /* loadmodel */);

    if (pFunc_load && PyCallable_Check(pFunc_load)) {
      PyObject_CallObject(pFunc_load, NULL);
    } else {
      if (PyErr_Occurred())
        PyErr_Print();
      printf("Cannot find python function - loadmodel\n");
    }
    Py_XDECREF(pFunc_load);
  }
  DEBUG(printf("CV Kernel Init done\n"));

#ifdef ENABLE_NVDLA
  // Initialize NVDLA
  initNVDLA();
#endif

  return success;

}


//void execute_cv_kernel(/* 0 */ label_t *in_tr_val, size_t in_tr_val_size, /* 1 */
//                               /* 2 */ label_t *out_label, size_t out_label_size /* 3 */) {
//
//#ifdef ENABLE_NVDLA
//  int obj_id = (int) * in_tr_val;
//  int num = (rand() % (INPUTS_PER_LABEL)); // Return a value from [0,INPUTS_PER_LABEL)
//  //printf("   NVDLA: runImageonNVDLA for \"%s\"\n", cv_inputs[obj_id][num]);
//  runImageonNVDLAWrapper(cv_inputs[obj_id][num]);
//  //runImageonNVDLAWrapper("0003_0.jpg");//"class_busimage_5489.jpg");
//  //system("echo -n \"  > NVDLA: \"; ./nvdla_runtime --loadable hpvm-mod.nvdla --image 2004_2.jpg --rawdump | grep execution");
//  //printf("\n");
//  *out_label = parse_output_dimg();
//  printf("    NVDLA Prediction: %d vs %d\n", *out_label, obj_id);
//#endif
//  *out_label = run_object_classification((unsigned)in_tr_val);
//}


label_t run_object_classification(unsigned tr_val) {

  DBGOUT(printf("Entered run_object_classification...\n"));
  label_t object = (label_t)tr_val;

  if (pModule != NULL) {
    pFunc = PyObject_GetAttrString(pModule, python_func /* predict */);

    if (pFunc && PyCallable_Check(pFunc)) {
      pArgs = PyTuple_New(1);
      // int rand_obj = rand() % 5;
      pValue = PyLong_FromLong(tr_val);
      if (!pValue) {
        Py_DECREF(pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        fprintf(stderr, "Trying to run CNN kernel: Cannot convert C argument into python\n");
        return 1;
      }
      PyTuple_SetItem(pArgs, 0, pValue);
      pretValue = PyObject_CallObject(pFunc, pArgs);
      Py_DECREF(pArgs);
      if (pretValue != NULL) {
	DBGOUT(printf("Predicted label from Python program: %ld\n", PyLong_AsLong(pretValue)));
        int val = PyLong_AsLong(pretValue);
        object = (label_t)val;
        DBGOUT(printf("run_object_classification returning %u = %u\n", val, object));
        Py_DECREF(pretValue);
      } else {
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        PyErr_Print();
        printf("Trying to run CNN kernel : Python function call failed\n");
        return 1;
      }
    } else {
      if (PyErr_Occurred())
        PyErr_Print();
      printf("Cannot find python function");
    }
    Py_XDECREF(pFunc);
    //Py_DECREF(pModule);
  }

  return object;
}

//label_t run_object_classification_2(const cv::Mat& image) {
//
//  return 0;
//}
