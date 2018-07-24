#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLTexture>
#include <QOpenGLWidget>
#include <GL/glu.h>
#include <QOpenGLFunctions>
#include <QCoreApplication>
#include <math.h>
#include <iostream>
#include <QPoint>
#include <QImage>
#include "pacman/Ghosts.h"
#include "pacman/Pacman.h"
#include "Utilities.h"

using namespace std;

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();
    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;
    void LoadTexture (QImage* img);
    void LoadNewTexture (QImage* img);
    void DrawMap();
    void DrawPacman();
    void DrawGhosts();
    void DrawCookies();
    void DrawBonus();
    void DrawCircle(float x, float y, float radius, float red, float green, float blue);
    void TogglePlaying();
    void UpdatePacmanPosition(int i);
    
protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    
private slots:
    void UpdateSimulationSlot();
    void ReceiveMapDataGL(int blockWidht, int blockHeight, QImage* mapImage, int *mObstacles, QVector<int> *pPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus);
    void SetPacmanCommand(int aPacmanCommand);
   
private:
    bool allowToPlay;
    QImage *_mapImage;  
    double ortho[4];
    QVector<GLuint> texIds;   
    int mapWidth, mapHeight;
    int *obstacles, firstTime;
    int nPacman;
    Pacman **pacmanArray;
    int nGhosts;
    Ghosts **ghostsArray;
    int sCookies;
    int sBonus;
    QPoint *cookiesCoord;
    QPoint *bonusCoord;
    Utilities utilities;

signals:
    void UpdatePacmanPos(QPoint coordPacman);
};

#endif
