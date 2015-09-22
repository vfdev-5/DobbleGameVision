#ifndef DGV_H
#define DGV_H

// Python
// Hack to disable python integrated python27_d.lib linking
// http://stackoverflow.com/questions/19716859/puzzling-dependency-of-boost-python-1-54-debug-build-to-python27-lib-on-window
#ifdef _DEBUG
#undef _DEBUG
#endif
#include <Python.h>

// Project
#include "Core/ImageCommon.h"
#include "Export.h"


#ifdef __cplusplus
extern "C" {
#endif
//******************************************************************************


/*
 * Function to be called from Python
 */
static PyObject* py_hello(PyObject* self, PyObject* args)
{
    char *s = "Hello from Dobble Game Vision!";
    return Py_BuildValue("s", s);
}

/*
 * Bind Python function names to our C functions
 */
static PyMethodDef dgv_methods[] = {
    {"hello", py_hello, METH_VARARGS},
    {NULL, NULL}
};

/*
 * Python calls this to let us initialize our module
 */
void DGV_PYD_EXPORT initdgv()
{
    (void) Py_InitModule("dgv", dgv_methods);
}




//******************************************************************************
#ifdef __cplusplus
}
#endif

#endif // DGV_H
