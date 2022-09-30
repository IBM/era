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
#include <Python.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "cv_toolset.h"
#include "globals.h"
#include "debug.h"


#ifdef USE_OLD_MODEL

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

#else

/*****************************************************************************/
/* NEW: PyTorch TinyYOLOv2 support (May 2022)                                */

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>

PyObject *python_yolo_model;


int recv_all(int sock, char *buf, int len)
{
  ssize_t n;

  while (len > 0) {
      n = recv(sock, buf, len, 0);

      if (n <= 0)
	return n;
      buf += n;
      len -= n;
  }

  return 1;
}


int cv_toolset_init(char *python_module, char *model_weights) {

  PyObject *module_name, *module, *dict, *python_class;

  printf("In the cv_toolset_init routine\n");

  // Initialize the Python interpreter. In an application embedding Python,
  // this should be called before using any other Python/C API functions
  Py_Initialize();
  import_array();

  // Returns new reference
  module_name = PyUnicode_FromString(python_module);

  // Returns new reference
  module = PyImport_Import(module_name);
  Py_DECREF(module_name);
  if (module == NULL) {
      PyErr_Print();
      printf("Fails to import the module. Perhaps PYTHONPATH needs to be set: export PYTHONPATH=<your_era_dir>/src/cv/yolo\n");
      return 1;
  }

  // Returns borrowed reference
  dict = PyModule_GetDict(module);
  Py_DECREF(module);
  if (dict == NULL) {
      PyErr_Print();
      printf("Fails to get the dictionary.\n");
      return 1;
  }

  // Returns borrowed reference
  python_class = PyDict_GetItemString(dict, "LITinyYolo");
  //Py_DECREF(dict);
  if (python_class == NULL) {
      PyErr_Print();
      printf("Fails to get the Python class.\n");
      return 1;
  }

  // Creates an instance of the class
  if (PyCallable_Check(python_class)) {
      // Returns new reference
      python_yolo_model = PyObject_CallObject(python_class, NULL);
      if (python_yolo_model == NULL) {
          PyErr_Print();
          printf("Fails to create Python object.\n");
          return 1;
      }
      //Py_DECREF(python_class);
  } else {
      printf("Cannot instantiate the Python class.\n");
      //Py_DECREF(python_class);
      return 1;
  }

  // Call a method of the class instance; in this case, the load() method to load the model weights
  // Returns new reference
  //PyObject *value = PyObject_CallMethod(python_yolo_model, "load", "(s)", "yolo/yolov2-tiny.weights");
  PyObject *value = PyObject_CallMethod(python_yolo_model, "load", "(s)", model_weights);
  if (value == NULL) {
      PyErr_Print();
      printf("Fails to call load() method.\n");
      return 1;
  }
  Py_DECREF(value);

#ifdef ENABLE_NVDLA
  // Initialize NVDLA
  initNVDLA();
#endif

  return 0;

}


detection_t *run_object_classification(unsigned char *data, dim_t dimensions, char *filename, int *nboxes) {

  detection_t *detections = NULL;

  if (python_yolo_model != NULL) {

      npy_intp dims[] = {dimensions.height, dimensions.width, dimensions.c};
      // Returns new or borrowed reference?
      PyObject *pValue = PyArray_SimpleNewFromData(3, dims, NPY_UINT8, data);

      if (pValue) {

	  // Returns new reference
	  PyObject *list = PyObject_CallMethod(python_yolo_model, "predict", "Os", pValue, filename);
	  Py_XDECREF(pValue);

	  if (list) {

	      // Here we process the list of dictionaries returned by the Python predict() function,
	      // and convert it into an array of detection_t structs. Each detection_t struct in the
	      // array corresponds to a detected object (and its bounding box) in the image.

	      *nboxes    = (int)PyList_Size(list);
	      detections = (detection_t *)malloc(*nboxes * sizeof(detection_t));

	      for (Py_ssize_t i = 0; i < *nboxes; i++) {

		  // Returns borrowed reference
		  PyObject* dict = PyList_GetItem(list, i);

		  if (!PyDict_Check(dict)) {
		      PyErr_SetString(PyExc_TypeError, "List must contain dictionaries");
		      PyErr_Print();
		      Py_XDECREF(list);
		      return NULL;
		  }

		  PyObject *key, *item;

		  // Returns new reference
		  key = PyUnicode_FromString("class_label");
		  // Returns borrowed reference
		  item = PyDict_GetItem(dict, key);
		  const char* class_label = PyUnicode_AsUTF8(item);
		  Py_XDECREF(key);

		  // Returns new reference
		  key = PyUnicode_FromString("id");
		  // Returns borrowed reference
		  item = PyDict_GetItem(dict, key);
		  long id = PyLong_AsLong(item);
		  Py_XDECREF(key);

		  // Returns new reference
		  key = PyUnicode_FromString("x_top_left");
		  // Returns borrowed reference
		  item = PyDict_GetItem(dict, key);
		  double x_top_left = PyFloat_AsDouble(item);
		  Py_XDECREF(key);

		  // Returns new reference
		  key = PyUnicode_FromString("y_top_left");
		  // Returns borrowed reference
		  item = PyDict_GetItem(dict, key);
		  double y_top_left = PyFloat_AsDouble(item);
		  Py_XDECREF(key);

		  // Returns new reference
		  key = PyUnicode_FromString("width");
		  // Returns borrowed reference
		  item = PyDict_GetItem(dict, key);
		  double width = PyFloat_AsDouble(item);
		  Py_XDECREF(key);

		  // Returns new reference
		  key = PyUnicode_FromString("height");
		  // Returns borrowed reference
		  item = PyDict_GetItem(dict, key);
		  double height = PyFloat_AsDouble(item);
		  Py_XDECREF(key);

		  // Returns new reference
		  key = PyUnicode_FromString("confidence");
		  // Returns borrowed reference
		  item = PyDict_GetItem(dict, key);
		  double confidence = PyFloat_AsDouble(item);
		  Py_XDECREF(key);

		  snprintf(detections[i].class_label, 255, "%s", class_label);
		  detections[i].id          = id;
		  detections[i].x_top_left  = x_top_left;
		  detections[i].y_top_left  = y_top_left;
		  detections[i].width       = width;
		  detections[i].height      = height;
		  detections[i].confidence  = confidence;

	      }
	      Py_XDECREF(list);

	  } else {
	      PyErr_Print();
	      return NULL;
	  }

      } else {
	  PyErr_Print();
	  return NULL;
      }
  }

  return detections;
}
/*****************************************************************************/

#endif
