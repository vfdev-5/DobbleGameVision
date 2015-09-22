#ifndef EXPORT_H
#define EXPORT_H

//******************************************************************************
// DLL Export definitions
//******************************************************************************
#if (defined WIN32 || defined _WIN32 || defined WINCE) && defined PYTHON_EXPORT
#  define DGV_PYD_EXPORT __declspec(dllexport)
#elif (defined WIN32 || defined _WIN32 || defined WINCE)
#  define DGV_PYD_EXPORT __declspec(dllimport) // This helps to resolve the problem with plugins link
#else
#  define DGV_PYD_EXPORT
#endif


#endif // EXPORT_H
