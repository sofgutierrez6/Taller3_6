#include "pacman/window.h"


Window::Window(QStringList args)
{
    char **argv;
    int argc = 0;
    ros::init(argc, argv, "pacman_world");
    this->setWindowTitle(tr("Pacman World"));
    this->setWindowIcon(QIcon(":/resources/textures/pacman.jpeg"));
    //this->setMaximumSize(maxWidth, maxHeight);
    
    QWidget *w = new QWidget();
    wScores = new QWidget();
    maps = new Maps();
    scoreName = new QLabel("Score: ");
    scoreLabel = new QLabel();
    livesName = new QLabel("Lives: ");
    livesLabel = new QLabel();
    performValName = new QLabel("Performance: ");
    performValLabel = new QLabel();
    playerNameLabel = new QLabel("Player: ");
    playerNameTextEdit = new QTextEdit();
    glWidget = new GLWidget();
    refreshTimer = new QTimer();
    mainLayout = new QVBoxLayout();
    container = new QHBoxLayout();
    containerScores = new QHBoxLayout();
    listenMsg = new ListenMsgThread();
    mapsList = new QComboBox();
    counterTimer = new QTimer();
    playBtn = new QPushButton(tr("Play"));
    counterBtn = new QPushButton();
    gameTimeRemainingLCD = new QLCDNumber(4);
    gameTime = new QTime(0, initialGameTimeMins, initialGameTimeSecs);
    initSound = new QSound(tr(":/resources/audio/start.wav"));
    restartGameTimer = new QTimer();
    scoreBoardFile = new QFile(QString::fromStdString(ros::package::getPath("pacman")) + "/ScoreBoard");
    node = new ros::NodeHandle();    
    
    QFont fontBold;
    fontBold.setBold(true);
    scoreName->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    scoreName->setFont(fontBold);
    scoreLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    livesName->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    livesName->setFont(fontBold);
    livesLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    performValName->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    performValName->setFont(fontBold);
    performValLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    playerNameLabel->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    playerNameLabel->setFont(fontBold);
    playerNameTextEdit->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    playerNameTextEdit->setMaximumSize(90, 43);
    w->setLayout(container);
    wScores->setLayout(containerScores);
    wScores->setMaximumSize(maxWidth, scoreHeight);
    mainLayout->addWidget(wScores);
    mainLayout->addWidget(counterBtn);
    mainLayout->addWidget(w);
    container->addWidget(glWidget);
    containerScores->addWidget(scoreName);  
    containerScores->addWidget(scoreLabel);
    containerScores->addWidget(livesName);
    containerScores->addWidget(livesLabel);
    containerScores->addWidget(performValName);
    containerScores->addWidget(performValLabel);   
    containerScores->addWidget(gameTimeRemainingLCD);
    containerScores->addWidget(playerNameLabel);
    containerScores->addWidget(playerNameTextEdit);
    ListArrayMap(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/layouts/");
    mode = getArguments(args);
    allowPlay = false;
    gameState = false;
    gameTimeRemainingLCD->setSegmentStyle(QLCDNumber::Filled);
    QString time = gameTime->toString();
    gameTimeRemainingLCD->display(time);
    restartGameTimer->setSingleShot(true);
    counterBtn->setStyleSheet("background-color: black;"
                            "font: bold 28px;"
			    "color: yellow;"
			   );
    counterBtn->setEnabled(false);
    playerDateAndTime = QTime::currentTime().toString(Qt::DefaultLocaleLongDate) + " " + QDate::currentDate().toString("dMMMyy");    
    
    if(mode == 1)
    {
      counterBtn->setText("Ready");
      mainLayout->addWidget(playBtn);
      mainLayout->addWidget(mapsList);
      mapName = mapsList->currentText();
      playerName = "GameMode ";
      connect(this, SIGNAL(ArrowKey(int)), glWidget, SLOT(receiveArrowKey(int)));
    }
    else if(mode == 2)
    {
      counterBtn->setText("Waiting for initialization request");
      mapName = verifyMapArgument(args, mapsList, mode);
      playerName = "ChallengeMode ";
      playerNameTextEdit->setEnabled(false);
    }
    else
    {
      cout << "Error: Pacman argument missing" << endl;
      cout << "Usage: rosrun pacman pacman_world [pacmanMode] {options}" << endl;
      cout << "       pacmanMode: Parameter that specifies the mode" << endl;
      cout << "                   --c: Challenge mode" << endl;
      cout << "                   --g: Game mode" << endl;
      cout << "       options   : Optional arguments" << endl;
      cout << "                   mapName: Name of map to play. Only needed in Challenge mode " << endl;
      cout << "                   --m: If used, runs the game with without sound " << endl;
      exit(0);
    }
    
    if(scoreBoardFile->exists())
	scoreBoardFile->remove();
    
    if (scoreBoardFile->open(QIODevice::ReadWrite | QIODevice::Text))
    {
	QTextStream out(scoreBoardFile);
	out << "Session " + playerName + playerDateAndTime << "\n";
    }
    performancePublisher = node->advertise<pacman::performance>("performanceEval",100);
    
    
    connect(playBtn, &QPushButton::clicked, this, &Window::PlaySlot);
    connect(mapsList, SIGNAL(currentIndexChanged(QString)), maps, SLOT(CreateMap(QString)));
    connect(mapsList, SIGNAL(currentIndexChanged(QString)), this, SLOT(UpdateMapNameSlot(QString)));
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, int, int)), glWidget, SLOT(ReceiveMapDataGL(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, int, int)));
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, int, int)), this, SLOT(UpdateNumberOfPacman(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, int, int)));
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, int, int)), this, SLOT(UpdateSizeSlot()));
    connect(refreshTimer, SIGNAL(timeout()), glWidget, SLOT(UpdateSimulationSlot()));
    connect(listenMsg, SIGNAL(UpdatePacmanCommand(int, int)), glWidget, SLOT(SetPacmanCommand(int, int)));
    connect(glWidget, SIGNAL(UpdatePacmanPos(QVector<QPoint>*)), this, SLOT(UpdatePacmanPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateGhostsPos(QVector<QPoint>*, bool*)), this, SLOT(UpdateGhostsPosSlot(QVector<QPoint>*, bool*)));
    connect(glWidget, SIGNAL(UpdateCookiesPos(QVector<QPoint>*)), this, SLOT(UpdateCookiesPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateBonusPos(QVector<QPoint>*)), this, SLOT(UpdateBonusPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateObstaclesPos(QVector<QPoint>*, int, int, int, int)), this, SLOT(UpdateObstaclesPosSlot(QVector<QPoint>*, int, int, int, int)));
    connect(glWidget, SIGNAL(DeadPacmanSignal()), this, SLOT(DeadPacmanSlot()));
    connect(glWidget, SIGNAL(EndOfDeadPacmanSignal()), this, SLOT(EndOfDeadPacmanSlot()));
    connect(glWidget, SIGNAL(UpdateScores(int, int)), this, SLOT(UpdateScoresSlot(int, int)));
    connect(counterTimer, SIGNAL(timeout()), this, SLOT(InitializeCounterTimerSlot()));
    connect(glWidget, SIGNAL(updateGameState()), this, SLOT(UpdateGameStateSlot()));
    connect(glWidget, SIGNAL(EndGameSignal(bool)), this, SLOT(EndGame(bool)));
    connect(glWidget, SIGNAL(SendMaxScore(int, int)), this, SLOT(ReceiveMaxValues(int, int)));
    connect(this, SIGNAL(InitializeGame()),this,SLOT(InitializeGameSlot()));
    connect(restartGameTimer, SIGNAL(timeout()), this, SLOT(restartReadySlot()));
    connect(playerNameTextEdit, SIGNAL(textChanged()), this, SLOT(PlayerNameChangedSlot()));
    
    glWidget->setNumberOfPacmans(numberOfPacmans);
    
    setLayout(mainLayout);
    maps->CreateMap(mapName);
    
    if (numberOfPacmans == 1)
    {
      subscriber0 = node->subscribe("pacmanActions0", 100, &ListenMsgThread::callback0, listenMsg);
    }
    else if (numberOfPacmans == 2)
    {
      subscriber0 = node->subscribe("pacmanActions0", 100, &ListenMsgThread::callback0, listenMsg);
      subscriber1 = node->subscribe("pacmanActions1", 100, &ListenMsgThread::callback1, listenMsg);
    }
    pacmanPublisher0 = node->advertise<pacman::pacmanPos>("pacmanCoord0", 100);
    pacmanPublisher1 = node->advertise<pacman::pacmanPos>("pacmanCoord1", 100);
    ghostPublisher = node->advertise<pacman::ghostsPos>("ghostsCoord",100);
    cookiesPublisher = node->advertise<pacman::cookiesPos>("cookiesCoord",100);
    bonusPublisher = node->advertise<pacman::bonusPos>("bonusCoord",100); 
    gameStatePublisher = node->advertise<pacman::game>("gameState",100);
    mapResponseServer = node->advertiseService("pacman_world", &Window::ObsService, this);
    
    listenMsg->start();
}

QSize Window::sizeHint() const
{
    return QSize(maps->getWidth(), maps->getHeight() + scoreHeight + 100);
}

QSize Window::minimumSizeHint() const
{
    return QSize(60, 60);
}

int Window::getArguments(QStringList args)
{
    int pacmanMode = 0;
    int cont = 0;
    for (QStringList::iterator it = args.begin(); it != args.end(); ++it)
    {
	QString current = *it;
	if (current.contains("--g", Qt::CaseInsensitive) && cont == 1)
	    pacmanMode = 1;
	else if (current.contains("--c", Qt::CaseInsensitive) && cont == 1)
	    pacmanMode = 2;
	else if (cont == 1)
	    pacmanMode = 0;
	
	if (current.contains("--m", Qt::CaseInsensitive) && (cont == 2 || cont == 3))
	{
	    mute = true;
	    glWidget->setMute();
	}
	cont++;
    }
    
    return pacmanMode;
}

void Window::InitializeCounterTimerSlot()
{
  if(counterBtn->text() == "Waiting for initialization request" || counterBtn->text() == "Ready")
  {
      if (!mute)
	  initSound->play();
      counterBtn->setText("Playing in... 3");
  }
  else if(counterBtn->text() == "Playing in... 3")
      counterBtn->setText("Playing in... 2");    
  else if(counterBtn->text() == "Playing in... 2")
      counterBtn->setText("Playing in... 1");
  else if(counterBtn->text() == "Playing in... 1")
  {
      counterBtn->setText("Playing");
      if (mode == 2)
	  listenMsg->setWorkingThread(true);
      glWidget->TogglePlaying();
      gameState = true;
      allowPlay = true;
      refreshTimer->start(refreshTimeMs);
  }
  else if (counterBtn->text() == "Playing")
  {
      *gameTime = gameTime->addMSecs(-oneSecondTimeMilisecs);
      QString time = gameTime->toString();
      gameTimeRemainingLCD->display(time);
  }
    
  if (*gameTime == QTime(0,0,0))
      EndGame(false);
}

void Window::keyPressEvent(QKeyEvent *e)
{
    if (allowPlay)
      emit ArrowKey(e->key());
}

void Window::ListArrayMap(QString path)
{
    QDir dir(path);
    dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
    QStringList list = dir.entryList();
    for(int i = 0; i < list.size(); i++)
      mapsList->addItem(list.at(i).split(".")[0]);
}

void Window::PlaySlot()
{
    if(playBtn->text() == "Play")
    {
      playBtn->setText("Stop");
      mapsList->setEnabled(false);
      counterTimer->start(oneSecondTimeMilisecs);
      playerNameTextEdit->setEnabled(false);
    }
    else
    {
      if (glWidget->IsPlaying())
	  glWidget->TogglePlaying();
      playerNameTextEdit->setEnabled(true);
      playBtn->setText("Play");
      counterBtn->setText("Ready");
      maps->CreateMap(mapName);
      counterTimer->stop();
      mapsList->setEnabled(true);
      allowPlay = false;
      gameTime->setHMS(0, initialGameTimeMins, initialGameTimeSecs);
      gameTimeRemainingLCD->display(gameTime->toString());
      if (!initSound->isFinished() && !mute)
	  initSound->stop();
    }
}
void Window::UpdatePacmanPosSlot(QVector<QPoint>* pos)
{
  if(pos->size() == 1)
  {
    msgPacman0.pacmanPos.x = pos->at(0).x(); 
    msgPacman0.pacmanPos.y = pos->at(0).y();
    msgPacman0.nPacman = pos->size();
    pacmanPublisher0.publish(msgPacman0);
  }
  else if (pos->size() == 2)
  {
    msgPacman0.pacmanPos.x = pos->at(0).x(); 
    msgPacman0.pacmanPos.y = pos->at(0).y();
    msgPacman0.nPacman = pos->size();
    pacmanPublisher0.publish(msgPacman0);
    msgPacman1.pacmanPos.x = pos->at(1).x(); 
    msgPacman1.pacmanPos.y = pos->at(1).y();
    msgPacman1.nPacman = pos->size();
    pacmanPublisher1.publish(msgPacman1);
  }
}
void Window::UpdateGhostsPosSlot(QVector<QPoint>* pos, bool* ghostsMode)
{
    msgGhosts.ghostsPos.resize(pos->size());
    msgGhosts.mode.resize(pos->size());
    for(int i = 0; i < pos->size(); i++)
    {
      msgGhosts.ghostsPos[i].x = pos->at(i).x(); 
      msgGhosts.ghostsPos[i].y = pos->at(i).y();
      msgGhosts.mode[i] = (int)ghostsMode[i];
    }
    msgGhosts.nGhosts = pos->size();
    ghostPublisher.publish(msgGhosts);
}
void Window::UpdateCookiesPosSlot(QVector<QPoint>* pos)
{
    msgCookies.cookiesPos.resize(pos->size());
    for(int i = 0; i < pos->size(); i++)
    {
      msgCookies.cookiesPos[i].x = pos->at(i).x();
      msgCookies.cookiesPos[i].y = pos->at(i).y();
    }
    msgCookies.nCookies = pos->size();
    cookiesPublisher.publish(msgCookies);
}
void Window::UpdateBonusPosSlot(QVector<QPoint>* pos)
{
    msgBonus.bonusPos.resize(pos->size());
    for(int i = 0; i < pos->size(); i++)
    {
      msgBonus.bonusPos[i].x = pos->at(i).x();
      msgBonus.bonusPos[i].y = pos->at(i).y();
    }
    msgBonus.nBonus = pos->size();
    bonusPublisher.publish(msgBonus);
}

void Window::UpdateObstaclesPosSlot(QVector< QPoint >* pos, int xMin, int xMax, int yMin, int yMax)
{
    posObstacles = pos;
    minX = xMin;
    maxX = xMax;
    minY = yMin;
    maxY = yMax;
}

void Window::UpdateSizeSlot()
{
    this->resize(sizeHint());
}

void Window::DeadPacmanSlot()
{
    refreshTimer->stop();
    counterTimer->stop();
    gameState = false;
}

void Window::EndOfDeadPacmanSlot()
{
    refreshTimer->start(refreshTimeMs);
    counterTimer->start(oneSecondTimeMilisecs);
    gameState = true;
}

void Window::EndGame(bool win)
{
    if (mode == 1)
    {
	playBtn->setText("Play");
	playBtn->setEnabled(false);
    }
    
    if (win)
	counterBtn->setText("Win!");
    else
	counterBtn->setText("Game Over");
    
    refreshTimer->stop();
    counterTimer->stop();
    listenMsg->setWorkingThread(false);
    glWidget->TogglePlaying();
    allowPlay = false;
    gameState = false;
    restartGameTimer->start(restartGameTime);
}

QString Window::verifyMapArgument(QStringList args, QComboBox *mapsList, int pacmanMode)
{
  QString mapArgument = "", Listmaps;
  bool mapFound = false;
  if(pacmanMode == 2)
  {
    for (QStringList::iterator it = args.begin(); it != args.end(); ++it)
    {
      QString current = *it;
      if(mapsList->findText(current) == -1 && !mapFound)
      {
      }
      else
      {
	mapArgument = mapsList->itemText(mapsList->findText(current));
	mapFound = true;
	break;
      }
    }      
    
    if(mapFound == false)
    {
      QDir dir(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/layouts/");
      dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
      QStringList list = dir.entryList();
      cout<<"No valid argument for map"<<endl;
      cout << "Maps List:" << endl;
      for(int i = 0; i < list.size(); i++)
	cout << list.at(i).toLocal8Bit().constData() << " -- ";
      exit(0);
    }
  }
  return QString(mapArgument);
}

void Window::UpdateGameStateSlot()
{
    msgState.state = (int)gameState; 
    gameStatePublisher.publish(msgState);
    
}

void Window::UpdateScoresSlot(int score, int lives)
{
    _score = score;
    _lives = lives;
    scoreLabel->setText(QString::number(score));
    livesLabel->setText(QString::number(lives));
    
    gTime = QTime(0, 0, 0).secsTo(*gameTime);
    performEval =  (((double)score / (double)MAX_SCORE ) * wScore) + (((double)lives / (double)MAX_LIVES ) * wLives) + (((double)gTime / (double)MAX_TIME_SEC ) * wTime);
    performEval *= 100;
    performValLabel->setText(QString::number(performEval, 'g', 4));
    msgPerformance.lives = _lives;
    msgPerformance.score = _score;
    msgPerformance.gtime = gTime;
    msgPerformance.performEval = performEval;
    performancePublisher.publish(msgPerformance);
}

bool Window::ObsService(pacman::mapService::Request& req, pacman::mapService::Response& res)
{
    res.minX = minX;
    res.maxX = maxX;
    res.minY = minY;
    res.maxY = maxY;
    res.nObs = posObstacles->size();
    res.obs.resize(posObstacles->size());
    for(int i = 0; i < posObstacles->size(); i++)
    {
      res.obs[i].x = posObstacles->at(i).x();
      res.obs[i].y = posObstacles->at(i).y();
    }
    if (counterBtn->text() == "Waiting for initialization request")
	emit InitializeGame();
    playerName = QString::fromStdString(req.name);
        
    return true;
}

void Window::InitializeGameSlot()
{
    gameTime->setHMS(0, initialGameTimeMins, initialGameTimeSecs);
    maps->CreateMap(mapName);
    QString time = gameTime->toString();
    gameTimeRemainingLCD->display(time);
    playerNameTextEdit->setText(playerName);
    counterTimer->start(oneSecondTimeMilisecs);
}

void Window::ReceiveMaxValues(int maxScore, int maxLives)
{
    MAX_SCORE = maxScore;
    MAX_LIVES = maxLives;
    MAX_TIME_SEC = (initialGameTimeMins * 60) + initialGameTimeSecs;
}

void Window::restartReadySlot()
{
    QTextStream out(scoreBoardFile);
    out << "Name: " << playerName << "\t" << "Score: " << _score << "\t" << "Lives: " 
    << _lives << "\t" << "Performance: " << performEval << "\t" << "Time: " << gTime 
    << "\t" << "Map: " << mapName << "\t" << "Date: " 
    << QTime::currentTime().toString(Qt::DefaultLocaleLongDate) << endl;
    
    if (mode == 1)
    {
	playBtn->setEnabled(true);
	counterBtn->setText("Ready");
	gameTime->setHMS(0, initialGameTimeMins, initialGameTimeSecs);
	QString time = gameTime->toString();
	gameTimeRemainingLCD->display(time);
	maps->CreateMap(mapName);
	mapsList->setEnabled(true);
	playerNameTextEdit->setEnabled(true);
    }
    else if (mode == 2)
	counterBtn->setText("Waiting for initialization request");
}

void Window::UpdateMapNameSlot(QString name)
{
    mapName = name;
    if (mapName == "ALIANZA" || mapName == "GHOST" || mapName == "mediumCRAZY" || mapName == "mediumPACMAN" || mapName == "originalEMA")
	numberOfPacmans = 2;
    else
	numberOfPacmans = 1;
    glWidget->setNumberOfPacmans(numberOfPacmans);
}

void Window::PlayerNameChangedSlot()
{
    if (playerNameTextEdit->toPlainText() == "" && mode == 1)
	playerName = "GameMode ";
    else if (playerNameTextEdit->toPlainText() == "" && mode == 2)
	playerName = "ChallengeMode ";
    else
	playerName = playerNameTextEdit->toPlainText();
}

void Window::UpdateNumberOfPacman(int blockWidth, int blockHeight, QImage* mapImage, bool* mObstacles, QVector< int >* pPacman, QVector< int >* pGhosts, QVector< int >* pCookies, QVector< int >* pBonus, QVector< int >* pObstacles, int maxIndexRow, int maxIndexCol)
{
    numberOfPacmans = pPacman->size()/2;
}
