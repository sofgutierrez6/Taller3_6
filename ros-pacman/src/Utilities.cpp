#include "pacman/Utilities.h"
#include <QtCore/qfiledevice.h>

Utilities::Utilities()
{

}

int Utilities::GetIndexColFromCoord(QPoint coord, int mapWidth)
{
    return (int)(coord.x() + mapWidth*0.5);
}

int Utilities::GetIndexRowFromCoord(QPoint coord, int mapHeight)
{
    return (int)(abs(coord.y() - mapHeight*0.5));
}

QPoint* Utilities::GetCoordFromIndex(int blockWidth, int blockHeight, double ortho[4], int iRow, int iCol)
{
    QPoint *p = new QPoint;
    p->setX(iCol*blockWidth+ortho[0]+blockWidth*0.5);
    p->setY(ortho[3]-iRow*blockHeight-blockHeight*0.5);
    
    return p;
}

QVector<QPoint>* Utilities::ConvertImageCoordToLayoutCoord(QVector<QPoint>* coordsImage, int blockWidth, int blockHeight)
{
    QVector<QPoint>* coordsLayout = new QVector<QPoint>;
    QPoint p;
    for(int i = 0; i < coordsImage->size(); i++)
    {
	p.setX(coordsImage->at(i).x() / blockWidth); 
	p.setY(coordsImage->at(i).y() / blockHeight);
	coordsLayout->append(p);
    }
    
    return coordsLayout;
}

int Utilities::ComputeDistanceBetweenPoints(QPoint a, QPoint b)
{
    return (int)sqrt((a.x() - b.x())*(a.x() - b.x()) + (a.y() - b.y())*(a.y() - b.y()));
}