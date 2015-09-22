#ifndef AppTest_H
#define AppTest_H

// Qt
#include <QObject>
#include <QtTest>

// Project

namespace Tests
{

//*************************************************************************

class AppTest : public QObject
{
    Q_OBJECT
private slots:

    void initTestCase();
    void detectCardsTest();

private:

    QStringList _filesToOpen;
    QString _path;
};

//*************************************************************************

} 

#endif // AppTest_H
