#ifndef GHOSTS_H
#define GHOSTS_H

#include <iostream>
#include <QObject>
#include <QPoint>
#include <math.h>
#include "Utilities.h"
#include <QVector>

using namespace std;

class Ghosts: public QObject
{
    Q_OBJECT

public:
    enum class Mode {Chase, Scatter, Frightened, Initial};
    enum class Action {Up, Down, Right, Left, None};
    enum class Personality {Shadow, Speedy, Bashful, Pokey}; 
    Ghosts(QPoint initialPosition, Ghosts::Personality character, int aHeight, int aWidth, QPoint initialPacmanPosition, int cMapHeight, int cMapWidth, bool *cObstacles);
    int GetTexId();
    void UpdateGhostPosition(QPoint newPacmanPosition, double newPacmanOrientation, QPoint newBlinkysPosition);
    void CalculateTargetPosition();
    int GetPossibleActions(QVector<Ghosts::Action> *possibleActions);
    void PrintPossibleActions(QVector<Ghosts::Action>* possibleActions);
    bool IsPossibleAction(Ghosts::Action anAction, QVector<Ghosts::Action>* possibleActions);
    void DeleteReverseAction(Ghosts::Action anAction, QVector<Ghosts::Action>* possibleActions);
    void ToggleMode();
    void ComputeGhostDecision(QVector<Ghosts::Action> *possibleActions);
    void SetFrigthenedMode();
    void RecoverFromFrigthenedMode();
    bool isFrightened();
    void SetInitialMode();
    
    QPoint currentPosition;
    QPoint _initialPosition;
    double orientation;
    Mode mode;
    Mode previousMode;
    QPoint targetPosition;
    QPoint scatterTargetPosition;
    Action action;
    QString name;
    Personality character;
    int height;
    int width;
    int texId;
    bool changedMode;
    Utilities utilities;
    bool deadGhost;
    
    QPoint pacmanPosition;
    QPoint blinkysPosition;
    double pacmanOrientation;
    int mapWidth, mapHeight;
    bool *obstacles;
    
    int frightenedMode;
    int frightenedCount;
};

#endif