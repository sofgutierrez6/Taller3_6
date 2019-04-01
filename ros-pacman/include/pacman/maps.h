#ifndef MAPS_H
#define MAPS_H

#include <iostream>

#include <QWidget>
#include <QObject>
#include <QImage>
#include <QFile>
#include <QLabel>
#include <QPixmap>
#include <QRgb>
#include <QComboBox>
#include <QVBoxLayout>
#include <QDir>
#include <QDirIterator>
#include <ros/package.h>

using namespace std;

class Maps : public QWidget
{
    Q_OBJECT

public:
    Maps(QWidget *parent = 0);
    int getWidth();
    int getHeight();
    
private:
    QByteArray File2ArrayMap(QString fileName, int &colsText, int &rowsText);
    void CreateObstaclesArray(QByteArray text, int colsText, int rowsText);
    void PrintObstaclesArray();
    void CreateImageFromObstaclesArray();
    void SaveImage(QString fileName);

   
private:
    int rows, cols;
    int BLOCK_SIZE = 50;
    bool *mObstacles;
    
    QVector<int> *pPacman;
    QVector<int> *pGhosts;
    QVector<int> *pCookies;
    QVector<int> *pBonus;
    QVector<int> *pObstacles;
    QImage* image;
    
public slots:
    void CreateMap(QString nameMap);
    
signals:
    void SendMapData(int blockWidth, int blockHeight, QImage *mapImage, bool *mObstacles, QVector<int> *pPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus, QVector<int> *pObstacles, int maxIndexRow, int maxIndexCol);
};

#endif
