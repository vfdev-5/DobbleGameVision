#ifndef GLOBAL_H
#define GLOBAL_H

// STD
#include <iostream>

// Qt
#include <QString>
#include <QMessageBox>

//******************************************************************************
// STD OUT, STD ERR
//******************************************************************************

#if 0
#   define SD_TRACE(msg) std::cout << QString(msg).toStdString() << std::endl;
#   define SD_WARN(msg)  std::cout << QString(msg).toStdString() << std::endl;
#   define SD_ERR(msg) std::cerr << QString(msg).toStdString() << std::endl;
#else
#   define SD_TRACE(msg) std::cout << QString(msg).toStdString() << std::endl;
#   define SD_TRACE1(msg, arg1) std::cout << QString(msg).arg(arg1).toStdString() << std::endl;
#   define SD_TRACE2(msg, arg1, arg2) std::cout << QString(msg).arg(arg1).arg(arg2).toStdString() << std::endl;
#   define SD_TRACE3(msg, arg1, arg2, arg3) std::cout << QString(msg).arg(arg1).arg(arg2).arg(arg3).toStdString() << std::endl;
#   define SD_TRACE4(msg, arg1, arg2, arg3, arg4) std::cout << QString(msg).arg(arg1).arg(arg2).arg(arg3).arg(arg4).toStdString() << std::endl;
#   define SD_TRACE_PTR(msg, ptr) std::cout << QString(msg + QString(" : 0x%1").arg((quintptr)ptr, QT_POINTER_SIZE, 16, QChar('0'))).toStdString() << std::endl;
#   define SD_WARN(msg) std::cout << QString(msg).toStdString() << std::endl; \
                        QMessageBox::warning(0, QObject::tr("Warning"), msg);
#   define SD_ERR(msg) std::cerr << QString(msg).toStdString() << std::endl; \
                       QMessageBox::critical(0, QObject::tr("Error"), msg);
#   define SD_INFO(msg) std::cout << QString(msg).toStdString() << std::endl; \
                       QMessageBox::information(0, QObject::tr("Information"), msg);

#endif

//******************************************************************************
// SOME VERBOSE PARAMETERS
//******************************************************************************
#define TIME_PROFILER_ON

#ifdef TIME_PROFILER_ON
void StartTimer(const QString & message);
double StopTimer();
#endif



//******************************************************************************
// CLASS PROPERTY MACROS
//******************************************************************************

#ifndef Q_PROPERTY_WITH_ACCESSORS

#define Q_PROPERTY_WITH_ACCESSORS(type, name, getter, setter) \
protected:                                                    \
    type _##name;                                             \
public:                                                       \
    Q_PROPERTY(type name READ getter WRITE setter)            \
    const type & getter () const                               \
    { return _##name; }                                       \
    void setter (const type &v)                               \
    { _##name = v; }                                          \

#endif


#ifndef Q_PROPERTY_WITH_GETACCESSOR

#define Q_PROPERTY_WITH_GETACCESSOR(type, name, getter)       \
protected:                                                    \
    type _##name;                                             \
public:                                                       \
    Q_PROPERTY(type name READ getter)                         \
    const type & getter () const                              \
    { return _##name; }                                       \

#endif


#ifndef PROPERTY_ACCESSORS

#define PROPERTY_ACCESSORS(type, name, getter, setter) \
protected:                                                    \
    type _##name;                                             \
public:                                                       \
    const type & getter () const                               \
    { return _##name; }                                       \
    void setter (const type &v)                               \
    { _##name = v; }                                          \

#endif


#ifndef PROPERTY_GETACCESSOR

#define PROPERTY_GETACCESSOR(type, name, getter) \
protected:                                                    \
    type _##name;                                             \
public:                                                       \
    const type & getter () const                               \
    { return _##name; }                                       \

#endif


#ifndef PTR_PROPERTY_ACCESSORS

#define PTR_PROPERTY_ACCESSORS(type, name, getter, setter)  \
protected:                                                    \
    type * _##name;                                           \
public:                                                       \
    type * getter ()                                          \
    { return _##name; }                                       \
    void setter ( type * v)                                   \
    { _##name = v; }                                          \

#endif

#ifndef PTR_PROPERTY_GETACCESSOR

#define PTR_PROPERTY_GETACCESSOR(type, name, getter)         \
protected:                                                    \
    type * _##name;                                           \
public:                                                       \
    type * getter ()                                          \
    { return _##name; }                                       \

#endif

//******************************************************************************

#endif // GLOBAL_H
