/*  
 *  Oct. 21 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  toPython transfers data to python 
 *
 * */

#include <iostream>
#include <sstream>
#include "toPython.h"

ToPython::ToPython(string dir, string module): 
  mModulePath(dir),
  mModuleName(module)
{
  init();
}

ToPython::~ToPython()
{
  uninit();
}

void ToPython::init()
{
    //初始化python
    Py_Initialize();
    
    //引入当前路径,否则下面模块不能正常导入
    PyRun_SimpleString("import sys");  
  
    stringstream ss; 
    ss <<"sys.path.append('"<<mModulePath<<"')";

    // PyRun_SimpleString("sys.path.append('./')");
    PyRun_SimpleString(ss.str().c_str());
    
    cout <<"toPython.cpp: init() load module: "<<mModulePath<<"/"<<mModuleName<<".py"<<endl;

    //引入模块
    mpModule = PyImport_ImportModule(mModuleName.c_str());

    //获取模块字典属性
    mpDict = PyModule_GetDict(mpModule);
}

void ToPython::uninit()
{
  //释放python
  Py_Finalize(); 
}

float ToPython::List2F1(string func_name, float *l1, float* l2, int N)
{
    PyObject *PyList  = PyList_New(N);
    PyObject *PyList2 = PyList_New(N);
    PyObject *ArgList = PyTuple_New(2);
    for(int Index_i = 0; Index_i < PyList_Size(PyList); Index_i++)
    {
      PyList_SetItem(PyList, Index_i, PyFloat_FromDouble(l1[Index_i]));
      PyList_SetItem(PyList2, Index_i, PyFloat_FromDouble(l2[Index_i]));
    }
    
    PyObject *pFuncFour = PyDict_GetItemString(mpDict, func_name.c_str());
    PyTuple_SetItem(ArgList, 0, PyList);
    PyTuple_SetItem(ArgList, 1, PyList2); 
    PyObject *r = PyObject_CallObject(pFuncFour, ArgList);
    float ret;  
    PyArg_Parse(r, "f", &ret);
    return ret; 
}
