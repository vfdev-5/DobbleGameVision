#ifndef LIBEXPORT_H
#define LIBEXPORT_H

//******************************************************************************
// DLL Export definitions
//******************************************************************************
#if (defined WIN32 || defined _WIN32 || defined WINCE) && defined LIB_EXPORT
#  define DGV_DLL_EXPORT __declspec(dllexport)
#elif (defined WIN32 || defined _WIN32 || defined WINCE)
#  define DGV_DLL_EXPORT __declspec(dllimport) // This helps to resolve the problem with plugins link
#else
#  define DGV_DLL_EXPORT
#endif


#endif // LIBEXPORT_H
