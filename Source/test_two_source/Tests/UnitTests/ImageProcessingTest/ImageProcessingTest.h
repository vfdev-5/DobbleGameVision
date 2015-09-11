#ifndef ImageProcessingTest_H
#define ImageProcessingTest_H

// Qt
#include <QObject>
#include <QtTest>

// Project

namespace Tests
{

//*************************************************************************

class ImageProcessingTest : public QObject
{
    Q_OBJECT
private slots:

    void detectObjectsTest();

private:

};

//*************************************************************************

} 

#endif // ImageProcessingTest_H
