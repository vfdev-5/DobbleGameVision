#ifndef ImageCommonTest_H
#define ImageCommonTest_H

// Qt
#include <QObject>
#include <QtTest>

// Project

namespace Tests
{

//*************************************************************************

class ImageCommonTest : public QObject
{
    Q_OBJECT
private slots:
    void isEllipseLikeTest();
    void isEllipseLike2Test1();
    void isEllipseLike2Test2();
//    void isCircleLikeTest();

    void intersectWithEllipseTest();

private:

};

//*************************************************************************

} 

#endif // ImageCommonTest_H
