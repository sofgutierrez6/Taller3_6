#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QObject>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>
#include <QTimer>
#include <QTime>
#include <QDate>
#include <QList>
#include <QFile>
#include <QTextStream>
#include <QTextEdit>
#include <QLCDNumber>
#include <QSound>
#include <iostream>
#include "ros/package.h"
#include "ros/ros.h"
#include "maps.h"
#include "pacman/glwidget.h"
#include "pacman/listenmsgthread.h"
#include "pacman/pacmanPos.h"
#include "pacman/ghostsPos.h"
#include "pacman/cookiesPos.h"
#include "pacman/bonusPos.h"
#include "pacman/game.h"
#include "pacman/pos.h"
#include "pacman/mapService.h"
#include "pacman/performance.h"
#include <QSerialPort>

using namespace std;

class GLWidget;
class MainWindow;
class Window : public QWidget
{
    Q_OBJECT

public:
    Window(QStringList args);		//GED Jul-27: Se recibe QStringList args con  argumentos
    int getArguments(QStringList args);	//GED Jul-28
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;
    QString verifyMapArgument(QStringList args, QComboBox *mapsList, int pacmanMode);	//GED Jul-30
private:
    void ListArrayMap(QString path);
    bool ObsService(pacman::mapService::Request& req, pacman::mapService::Response &res);
   
protected:
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void PlaySlot();
    void UpdatePacmanPosSlot(QVector<QPoint>* pos);
    void UpdateGhostsPosSlot(QVector<QPoint>* pos, bool* ghostsMode);
    void UpdateCookiesPosSlot(QVector<QPoint>* pos);
    void UpdateBonusPosSlot(QVector<QPoint>* pos);
    void UpdateObstaclesPosSlot(QVector<QPoint>* pos, int xMin, int xMax, int yMin, int yMax);
    void UpdateSizeSlot();
    void DeadPacmanSlot();
    void EndOfDeadPacmanSlot();
    void UpdateGameStateSlot();
    void UpdateScoresSlot(int score, int lives);
    void EndGame(bool win);	//GED Ag-01
    void InitializeCounterTimerSlot();	//Jul-27
    void ReceiveMaxValues(int maxScore, int maxLives);
    void InitializeGameSlot();
    void restartReadySlot();
    void UpdateMapNameSlot(QString name);
    void PlayerNameChangedSlot();
    void UpdateNumberOfPacman(int blockWidth, int blockHeight, QImage *mapImage, bool *mObstacles, QVector<int> *pPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus, QVector<int> *pObstacles, int maxIndexRow, int maxIndexCol);
    
private:
    GLWidget *glWidget;
    QWidget *wScores;
    const int maxWidth = 1000;
    const int maxHeight = 680;
    const int scoreWidth = 750;
    const int scoreHeight = 60;
    const double wScore = 0.3;
    const double wLives = 0.6;
    const double wTime = 0.1;
    int windowWidth, windowHeight;
    int MAX_SCORE, MAX_LIVES, MAX_TIME_SEC;
    double performEval;
    QVBoxLayout *mainLayout;
    QPushButton *playBtn;
    QPushButton *counterBtn;	//GED Jul-27
    MainWindow *mainWindow;
    Maps *maps;
    QString mapName;
    int mode;
    bool gameState = false;
    QComboBox *mapsList;
    QHBoxLayout *container;
    bool allowPlay;
    QTimer *refreshTimer;
    QTimer *counterTimer; 	//GED Jul-27
    const int refreshTimeMs = 150;
    ListenMsgThread *listenMsg;
    ros::NodeHandle *node;
    ros::Subscriber subscriber0;
    ros::Subscriber subscriber1;
    ros::Publisher pacmanPublisher0;
    ros::Publisher pacmanPublisher1;
    ros::Publisher ghostPublisher;
    ros::Publisher cookiesPublisher;
    ros::Publisher bonusPublisher;
    ros::Publisher gameStatePublisher;
    ros::Publisher performancePublisher;
    ros::ServiceServer mapResponseServer;
    pacman::pacmanPos msgPacman0;
    pacman::pacmanPos msgPacman1;
    pacman::ghostsPos msgGhosts;
    pacman::cookiesPos msgCookies;
    pacman::bonusPos msgBonus;
    pacman::game msgState;
    pacman::performance msgPerformance;
    QVector<QPoint> *posObstacles;
    int minX, maxX, minY, maxY;
    QHBoxLayout *containerScores;
    QLabel *scoreName;
    QLabel *scoreLabel;
    QLabel *livesName;
    QLabel *livesLabel;
    QLCDNumber *gameTimeRemainingLCD;
    QLabel *performValName;
    QLabel *performValLabel;
    QLabel *playerNameLabel;
    QTextEdit *playerNameTextEdit;
    QTimer *remainingTimeTimer;
    QTime *gameTime;
    const int initialGameTimeMins = 3;
    const int initialGameTimeSecs = 0;
    const int oneSecondTimeMilisecs = 1000;
    QSound *initSound;
    QTimer *restartGameTimer;
    const int restartGameTime = 5000;
    int numberOfPacmans;
    QString playerName, playerDateAndTime;
    QFile *scoreBoardFile;
    int _score, _lives, gTime;
    bool mute;
    
signals:
    void ArrowKey(int key);
    void InitializeGame();
};

#endif
