#include "pacman/glwidget.h"

GLWidget::GLWidget(QWidget *parent)
{
    firstTime = true;
    allowToPlay = false;
    nPacman = 0;
    nGhosts = 0;
    ghostModeTimer = new QTimer();
    frightenedGhostModeTimer = new QTimer();
    deadPacmanTimer = new QTimer();
    frightenedGhostAlmostModeTimer = new QTimer();
    cookiesSound = new QSound(tr(":/resources/audio/cookies.wav"));
    normalSound = new QSound(tr(":/resources/audio/normal.wav"));
    frightenedSound = new QSound(tr(":/resources/audio/frightened.wav"));
    ghostSound = new QSound(tr(":/resources/audio/ghosts.wav"));
    returnHomeSound = new QSound(tr(":/resources/audio/returnHome.wav"));
    missSound = new QSound(tr(":/resources/audio/miss.wav"));
    
    ghostModeTimer->setSingleShot(true);
    frightenedGhostModeTimer->setSingleShot(true);
    deadPacmanTimer->setSingleShot(true);
    frightenedGhostAlmostModeTimer->setSingleShot(true);
    connect(ghostModeTimer, SIGNAL(timeout()), this, SLOT(ToggleGhostModeSlot()));
    connect(frightenedGhostModeTimer, SIGNAL(timeout()), this, SLOT(EndOfFrightenedGhostModeSlot()));
    connect(deadPacmanTimer, SIGNAL(timeout()), this, SLOT(EndOfDeadPacmanSlot()));
    connect(frightenedGhostAlmostModeTimer, SIGNAL(timeout()), this, SLOT(ChangeFrightenedFigSlot()));
    contGhostModePhases = 0;
    ghostRemainingTime = 0;
    isInFrightenedMode = false;
    mute = false;
}

GLWidget::~GLWidget()
{

}

void GLWidget::LoadTexture (QImage* img)
{
    GLuint tex;
    glEnable(GL_TEXTURE_2D); // Enable texturing
    QImage t = (img->convertToFormat(QImage::Format_RGBA8888)).mirrored();
    glGenTextures(1, &tex); // Obtain an id for the texture
    glBindTexture(GL_TEXTURE_2D, tex); // Set as the current texture
    texIds.append(tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, t.width(), t.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, t.bits());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDisable(GL_TEXTURE_2D);
}

void GLWidget::LoadNewTexture (QImage* img)
{
    GLuint tex;
    glEnable(GL_TEXTURE_2D); // Enable texturing
    QImage t = (img->convertToFormat(QImage::Format_RGBA8888)).mirrored();
    glGenTextures(1, &tex); // Obtain an id for the texture
    glBindTexture(GL_TEXTURE_2D, tex); // Set as the current texture
    texIds[20] = tex;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, t.width(), t.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, t.bits());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDisable(GL_TEXTURE_2D);
}

void GLWidget::DrawMap()
{
    glColor3f(0.7, 0.7, 0.7);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texIds[20]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f  (-0.5*_mapWidth, -0.5*_mapHeight);
    glTexCoord2f(0.0f,  1.0f);
    glVertex2f  (-0.5*_mapWidth,  0.5*_mapHeight);
    glTexCoord2f(1.0f,  1.0f);
    glVertex2f  (0.5*_mapWidth,  0.5*_mapHeight);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f  (0.5*_mapWidth, -0.5*_mapHeight);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void GLWidget::DrawPacman()
{
    for(int i=0;i < nPacman;i++)
    {
	glShadeModel(GL_SMOOTH);
	glPushMatrix();
	glTranslated(pacmanArray[i]->currentPosition.x(),pacmanArray[i]->currentPosition.y(), 0);
	glRotated(pacmanArray[i]->orientation, 0, 0, 1);
	glColor3f(1.0, 1.0, 1.0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texIds[i]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f  (-0.5*pacmanArray[i]->width, -0.5*pacmanArray[i]->height);
	glTexCoord2f(0.0f,  1.0f);
	glVertex2f  (-0.5*pacmanArray[i]->width,  0.5*pacmanArray[i]->height);
	glTexCoord2f(1.0f,  1.0f);
	glVertex2f  (0.5*pacmanArray[i]->width,  0.5*pacmanArray[i]->height);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f  (0.5*pacmanArray[i]->width, -0.5*pacmanArray[i]->height);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	glColor3f(1.0, 1.0, 1.0);
    }
}

void GLWidget::DrawGhosts()
{
    for(int i=0;i < nGhosts;i++)
    {
	glShadeModel(GL_SMOOTH);
	glPushMatrix();
	glTranslated(ghostsArray[i]->currentPosition.x(), ghostsArray[i]->currentPosition.y(), 0);
	glRotated(0, 0, 0, 1);
	glColor3f(1.0, 1.0, 1.0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texIds[ghostsArray[i]->GetTexId()]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f  (-0.5*ghostsArray[i]->width , -0.5*ghostsArray[i]->height);
	glTexCoord2f(0.0f,  1.0f);
	glVertex2f  (-0.5*ghostsArray[i]->width,  0.5*ghostsArray[i]->height);
	glTexCoord2f(1.0f,  1.0f);
	glVertex2f  (0.5*ghostsArray[i]->width,  0.5*ghostsArray[i]->height);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f  (0.5*ghostsArray[i]->width, -0.5*ghostsArray[i]->height);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	glColor3f(1.0, 1.0, 1.0);
    }
}

void GLWidget::DrawCookies()
{
    for(int i = 0; i < cookiesCoord->size(); i++)
	DrawCircle(cookiesCoord->at(i).x(), cookiesCoord->at(i).y(), _blockWidth/11.6, 1.0, 1.0, 0.0);
    if(cookiesCoord->size() == 0 and bonusCoord->size() == 0)	//GED Ag-01
	emit EndGameSignal(true);
}

void GLWidget::DrawBonus()
{
    for(int i = 0; i < bonusCoord->size(); i++)
	DrawCircle(bonusCoord->at(i).x(), bonusCoord->at(i).y(), _blockWidth/6.4, 1.0, 0.8, 0.0);
}

void GLWidget::DrawCircle(float x, float y, float radius, float red, float green, float blue) 
{ 
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(x, y, 0.0f);
    glColor3f(red, green, blue);
    static const int circle_points = 100;
    static const float angle = 2.0f * 3.1416f / circle_points;

    glBegin(GL_POLYGON);
    double angle1=0.0;
    glVertex2d(radius * cos(0.0) , radius * sin(0.0));
    int i;
    for (i=0; i<circle_points; i++)
    {       
        glVertex2d(radius * cos(angle1), radius *sin(angle1));
        angle1 += angle;
    }
    glEnd();
    glPopMatrix();
}

void GLWidget::TogglePlaying()
{
    if (allowToPlay)
    {
      allowToPlay = false;
      ghostModeTimer->stop();
      contGhostModePhases = 0;
    }
    else
    {
      ghostModeTimer->start(ghostModeTimes[contGhostModePhases]);
      contGhostModePhases++;
      allowToPlay = true;
    }
}

void GLWidget::UpdatePacmanPosition(int i)
{
    int stepX = pacmanArray[i]->width, stepY = pacmanArray[i]->height;
    
    if(pacmanArray[i]->action == Pacman::Action::Right)
    {
      QPoint coord(pacmanArray[i]->currentPosition.x() + (int)(pacmanArray[i]->width*0.5)  + 1, pacmanArray[i]->currentPosition.y());
      if (!_obstacles[utilities.GetIndexRowFromCoord(coord, _mapHeight)*_mapWidth + utilities.GetIndexColFromCoord(coord, _mapWidth)])
	  pacmanArray[i]->currentPosition.setX(pacmanArray[i]->currentPosition.x() + stepX);
      pacmanArray[i]->orientation = 0.0;
    }
    else if(pacmanArray[i]->action == Pacman::Action::Left)
    {
      QPoint coord(pacmanArray[i]->currentPosition.x() - (int)(pacmanArray[i]->width*0.5) - 1, pacmanArray[i]->currentPosition.y());
      if (!_obstacles[utilities.GetIndexRowFromCoord(coord, _mapHeight)*_mapWidth + utilities.GetIndexColFromCoord(coord, _mapWidth)])
	  pacmanArray[i]->currentPosition.setX(pacmanArray[i]->currentPosition.x() - stepX);
      pacmanArray[i]->orientation = 180.0;
    }
    else if(pacmanArray[i]->action == Pacman::Action::Up)
    {
      QPoint coord(pacmanArray[i]->currentPosition.x(), pacmanArray[i]->currentPosition.y() + (int)(pacmanArray[i]->height*0.5) + 1);
      if (!_obstacles[utilities.GetIndexRowFromCoord(coord, _mapHeight)*_mapWidth + utilities.GetIndexColFromCoord(coord, _mapWidth)])
	  pacmanArray[i]->currentPosition.setY(pacmanArray[i]->currentPosition.y() + stepY);
      pacmanArray[i]->orientation = 90.0;
    }
    else if(pacmanArray[i]->action == Pacman::Action::Down)
    {
      QPoint coord(pacmanArray[i]->currentPosition.x(), pacmanArray[i]->currentPosition.y() - (int)(pacmanArray[i]->height*0.5) - 1);
      if (!_obstacles[utilities.GetIndexRowFromCoord(coord, _mapHeight)*_mapWidth + utilities.GetIndexColFromCoord(coord, _mapWidth)])
	  pacmanArray[i]->currentPosition.setY(pacmanArray[i]->currentPosition.y() - stepY);
      pacmanArray[i]->orientation = 270.0;
    }    
}

void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    LoadTexture(new QImage(tr(":/resources/textures/pacman.jpeg")));
    LoadTexture(new QImage(tr(":/resources/textures/pacgirl.jpg")));
    LoadTexture(new QImage(tr(":/resources/textures/redGhostUp.png")));
    LoadTexture(new QImage(tr(":/resources/textures/redGhostDown.png")));
    LoadTexture(new QImage(tr(":/resources/textures/redGhostRight.png")));
    LoadTexture(new QImage(tr(":/resources/textures/redGhostLeft.png")));
    LoadTexture(new QImage(tr(":/resources/textures/pinkGhostUp.png")));
    LoadTexture(new QImage(tr(":/resources/textures/pinkGhostDown.png")));
    LoadTexture(new QImage(tr(":/resources/textures/pinkGhostRight.png")));
    LoadTexture(new QImage(tr(":/resources/textures/pinkGhostLeft.png")));
    LoadTexture(new QImage(tr(":/resources/textures/blueGhostUp.png")));
    LoadTexture(new QImage(tr(":/resources/textures/blueGhostDown.png")));
    LoadTexture(new QImage(tr(":/resources/textures/blueGhostRight.png")));
    LoadTexture(new QImage(tr(":/resources/textures/blueGhostLeft.png")));
    LoadTexture(new QImage(tr(":/resources/textures/orangeGhostUp.png")));
    LoadTexture(new QImage(tr(":/resources/textures/orangeGhostDown.png")));
    LoadTexture(new QImage(tr(":/resources/textures/orangeGhostRight.png")));
    LoadTexture(new QImage(tr(":/resources/textures/orangeGhostLeft.png")));
    LoadTexture(new QImage(tr(":/resources/textures/frightenedGhost1.png")));
    LoadTexture(new QImage(tr(":/resources/textures/frightenedGhost2.png")));
    LoadTexture(_mapImage);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(ortho[0], ortho[1], ortho[2], ortho[3], -10, 10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
      
    DrawMap();
    DrawCookies();
    DrawBonus();
    DrawPacman();
    DrawGhosts();
}

void GLWidget::resizeGL(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(ortho[0], ortho[1], ortho[2], ortho[3], -10, 10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GLWidget::UpdateSimulationSlot()
{      
    bool scaredGhosts = false;
    //Update ghost dynamics (it has be done before changing pacman position)
    QVector<QPoint> previousGhostsCoord, previousPacmanCoord;
    QPoint pacmanCurrentPosition[4];
    double pacmanCurrentOrientation[4];
    if (nPacman == 1)
    {
	pacmanCurrentPosition[0] = pacmanArray[0]->currentPosition;
	pacmanCurrentPosition[1] = pacmanArray[0]->currentPosition;
	pacmanCurrentPosition[2] = pacmanArray[0]->currentPosition;
	pacmanCurrentPosition[3] = pacmanArray[0]->currentPosition;
	pacmanCurrentOrientation[0] = pacmanArray[0]->orientation;
	pacmanCurrentOrientation[1] = pacmanArray[0]->orientation;
	pacmanCurrentOrientation[2] = pacmanArray[0]->orientation;
	pacmanCurrentOrientation[3] = pacmanArray[0]->orientation;
    }
    else if (nPacman == 2)
    {
	pacmanCurrentPosition[0] = pacmanArray[0]->currentPosition;
	pacmanCurrentPosition[1] = pacmanArray[0]->currentPosition;
	pacmanCurrentPosition[2] = pacmanArray[1]->currentPosition;
	pacmanCurrentPosition[3] = pacmanArray[1]->currentPosition;
	pacmanCurrentOrientation[0] = pacmanArray[0]->orientation;
	pacmanCurrentOrientation[1] = pacmanArray[0]->orientation;
	pacmanCurrentOrientation[2] = pacmanArray[1]->orientation;
	pacmanCurrentOrientation[3] = pacmanArray[1]->orientation;
    }
    
    for(int i = 0;i < nGhosts; i++)
    {
	previousGhostsCoord.append(ghostsArray[i]->currentPosition);
	if(allowToPlay)
	   ghostsArray[i]->UpdateGhostPosition(pacmanCurrentPosition[i], pacmanCurrentOrientation[i], ghostsArray[0]->currentPosition);
	
	ghostsCoord->replace(i, ghostsArray[i]->currentPosition);
	scaredGhosts |= ghostsArray[i]->isFrightened();
    }
        
    if (normalSound->isFinished() && allowToPlay && !scaredGhosts && !mute)
	normalSound->play();
    else if (frightenedSound->isFinished() && scaredGhosts && !mute)
	frightenedSound->play();

    if ((frightenedGhostModeTimer->remainingTime() < almostFrightenedTimeMs) && (frightenedGhostModeTimer->remainingTime() > 0))
    {
	if (frightenedGhostAlmostModeTimer->remainingTime() <= 0)
	    frightenedGhostAlmostModeTimer->start();
    }
	
    //Update pacman dynamics
    for(int i = 0; i < nPacman; i++)
    {
	previousPacmanCoord.append(pacmanArray[i]->currentPosition);
	UpdatePacmanPosition(i);
	pacmanCoord->replace(i, pacmanArray[i]->currentPosition);
    }
    
    //Update cookies
    for(int j = 0; j < nPacman; j++)
    {
	for(int i = 0; i < cookiesCoord->size(); i++)
	{
	    if (pacmanCoord->at(j) == cookiesCoord->at(i))
	    {
		if (cookiesSound->isFinished() && !mute)
		    cookiesSound->play();
		cookiesCoord->remove(i);
		score += COOKIES_SCORE;
	    }
	}
    }
       
    //Update Bonuses and detect Frightened mode
    bool enterFrigthenedMode = false;
    for(int j = 0; j < nPacman; j++)
    {
	for(int i = 0; i < bonusCoord->size(); i++)
	{
	    if (pacmanCoord->at(j) == bonusCoord->at(i))
	    {
		enterFrigthenedMode |= true;
		bonusCoord->remove(i);
		score += BONUS_SCORE;
		scoreGhosts = GHOSTS_BASE_SCORE;
	    }
	}      
    }
    
    //If Frightened mode was detected, pause the current timer (if any) and start or restart the frigthened timer. Set ghosts to frigthened mode
    if (enterFrigthenedMode)
    {
	ghostRemainingTime = ghostModeTimer->remainingTime();
	
	if (ghostRemainingTime > 0)
	    ghostModeTimer->stop();
	
	if (frightenedGhostAlmostModeTimer->remainingTime() > 0)
	    frightenedGhostAlmostModeTimer->stop();
	
	frightenedGhostModeTimer->start(frightenedModeTimeMs);
	scoreGhosts = GHOSTS_BASE_SCORE;
	
	for(int i = 0;i < nGhosts;i++)
	    ghostsArray[i]->SetFrigthenedMode();
    }
    
    //Check for dead ghosts and dead pacman
    for(int j = 0;j < nPacman;j++)
    {
	for(int i = 0;i < ghostsCoord->size();i++)
	{
	    if ((pacmanCoord->at(j) == ghostsCoord->at(i)) || ((previousPacmanCoord.at(j) == ghostsCoord->at(i)) && (previousGhostsCoord.at(i) == pacmanCoord->at(j))))
	    {
		if (ghostsArray[i]->isFrightened())
		{
		    scoreGhosts *= 2;
		    score += scoreGhosts;
		    ghostsArray[i]->deadGhost = true;
		    ghostsCoord->replace(i, ghostsArray[i]->currentPosition);
		    deadGhostTimers[i]->start(deadGhostTimeMs);
		    if (!mute)
		    {
			ghostSound->play();
			returnHomeSound->play();
		    }
		}
		else
		{
		    lives--;
		    if (!mute)
			missSound->play();
		    emit DeadPacmanSignal();
		    if(lives == 0)	//GED Ago-01
		      emit EndGameSignal(false);
		    else
		      deadPacmanTimer->start(deadPacmanTimeMs);
		}
	    }
	}      
    }
    ghostsMode = (bool*)malloc(sizeof(bool)*nGhosts);
    for(int i = 0;i < nGhosts;i++)
    {
	if (ghostsArray[i]->isFrightened())
	    ghostsMode[i] = true;
	else
	    ghostsMode[i] = false;
    }
    
    emit UpdateGhostsPos(utilities.ConvertImageCoordToLayoutCoord(ghostsCoord, _blockWidth, _blockHeight), ghostsMode);
    emit UpdatePacmanPos(utilities.ConvertImageCoordToLayoutCoord(pacmanCoord, _blockWidth, _blockHeight));
    emit UpdateCookiesPos(utilities.ConvertImageCoordToLayoutCoord(cookiesCoord, _blockWidth, _blockHeight));
    emit UpdateBonusPos(utilities.ConvertImageCoordToLayoutCoord(bonusCoord, _blockWidth, _blockHeight));
    emit updateGameState();
    emit UpdateScores(score, lives);
    
    update();
}

void GLWidget::ReceiveMapDataGL(int blockWidth, int blockHeight, QImage* mapImage, bool *mObstacles, QVector<int> *pPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus, QVector<int> *pObstacles, int maxIndexRow, int maxIndexCol)
{
    if(!firstTime)
      LoadNewTexture(mapImage);
    else
      firstTime = false;

    //Set Map
    _mapImage = new QImage(*mapImage);
    _mapWidth = mapImage->width();
    _mapHeight = mapImage->height();
    _blockWidth = blockWidth;
    _blockHeight = blockHeight;
    _obstacles = new bool[(_mapHeight)*(_mapWidth)];
    memcpy(_obstacles, mObstacles, (_mapHeight)*(_mapWidth)*sizeof(bool));
    ortho[0] = -_mapWidth*0.5;
    ortho[1] = _mapWidth*0.5;
    ortho[2] = -_mapHeight*0.5;
    ortho[3] = _mapHeight*0.5;
        
    //Set Pacmans
    nPacman = pPacman->size()/2;
    pacmanArray = (Pacman**) malloc(sizeof(Pacman)*nPacman);
    pacmanCoord = new QVector<QPoint>;
    QPoint *pacmanPositions;
    for(int i = 0; i < nPacman; i++)
    {
	pacmanPositions = utilities.GetCoordFromIndex(_blockWidth, _blockHeight, ortho, pPacman->at(i*2), pPacman->at(i*2 + 1));
	pacmanArray[i] = new Pacman(*pacmanPositions, (double)0, blockHeight, blockWidth);
	pacmanCoord->append(pacmanArray[i]->currentPosition);
    }
        
    //Set Ghosts
    nGhosts = pGhosts->size()/2;
    ghostsArray = (Ghosts**) malloc(sizeof(Ghosts)*nGhosts);
    deadGhostTimers = (QTimer**) malloc(sizeof(QTimer)*nGhosts);
    ghostsCoord = new QVector<QPoint>;
    QPoint *ghostPositions;
    Ghosts::Personality ghostsPersonality[4];
    ghostsPersonality[0] = Ghosts::Personality::Shadow;
    ghostsPersonality[1] = Ghosts::Personality::Speedy;
    
    QPoint tempPacmanPositions[4];
    if (nPacman == 1)
    {
	ghostsPersonality[2] = Ghosts::Personality::Bashful;
	ghostsPersonality[3] = Ghosts::Personality::Pokey;
    }
    else if (nPacman == 2)
    {
	ghostsPersonality[2] = Ghosts::Personality::Shadow;
	ghostsPersonality[3] = Ghosts::Personality::Speedy;
	tempPacmanPositions[0] = pacmanArray[0]->currentPosition;
	tempPacmanPositions[1] = pacmanArray[0]->currentPosition;
	tempPacmanPositions[2] = pacmanArray[1]->currentPosition;
	tempPacmanPositions[3] = pacmanArray[1]->currentPosition;
    }
	
    for(int i = 0; i < nGhosts; i++)
    {
	ghostPositions = utilities.GetCoordFromIndex(_blockWidth, _blockHeight, ortho, pGhosts->at(i*2), pGhosts->at(i*2 + 1));
	if (nPacman == 1)
	    ghostsArray[i] = new Ghosts(*ghostPositions, ghostsPersonality[i], blockHeight, blockWidth, pacmanArray[0]->currentPosition, _mapHeight, _mapWidth, _obstacles);
	else if (nPacman == 2)
	    ghostsArray[i] = new Ghosts(*ghostPositions, ghostsPersonality[i], blockHeight, blockWidth, tempPacmanPositions[i], _mapHeight, _mapWidth, _obstacles);
	ghostsCoord->append(ghostsArray[i]->currentPosition);
	
	deadGhostTimers[i] = new QTimer();
	deadGhostTimers[i]->setSingleShot(true);
	if (i == 0)
	    connect(deadGhostTimers[i], SIGNAL(timeout()), this, SLOT(reviveGhost0Slot()));
	else if (i == 1)
	    connect(deadGhostTimers[i], SIGNAL(timeout()), this, SLOT(reviveGhost1Slot()));
	else if (i == 2)
	    connect(deadGhostTimers[i], SIGNAL(timeout()), this, SLOT(reviveGhost2Slot()));
	else if (i == 3)
	    connect(deadGhostTimers[i], SIGNAL(timeout()), this, SLOT(reviveGhost3Slot()));
    }
    scoreGhosts = GHOSTS_BASE_SCORE;
    
    //Set cookies
    cookiesCoord = new QVector<QPoint>;
    for(int i = 0; i < pCookies->size()/2; i++)
	cookiesCoord->append( *utilities.GetCoordFromIndex(_blockWidth, _blockHeight, ortho, pCookies->at(i*2), pCookies->at(i*2 + 1)) );
    
    //Set bonuses
    bonusCoord = new QVector<QPoint>;
    for(int i = 0; i < pBonus->size()/2; i++)
	bonusCoord->append( *utilities.GetCoordFromIndex(_blockWidth, _blockHeight, ortho, pBonus->at(i*2), pBonus->at(i*2 + 1)) );
    
    //Set obstacles
    obstaclesCoord = new QVector<QPoint>;
    for(int i = 0; i < pObstacles->size()/2; i++)
	obstaclesCoord->append( *utilities.GetCoordFromIndex(_blockWidth, _blockHeight, ortho, pObstacles->at(i*2), pObstacles->at(i*2 + 1)) );
   
    //Set boundaries
    QVector<QPoint>* boundaries = new QVector<QPoint>;
    boundaries->append( *utilities.GetCoordFromIndex(_blockWidth, _blockHeight, ortho, 0, 0) ); //Upper Left Boundary
    boundaries->append( *utilities.GetCoordFromIndex(_blockWidth, _blockHeight, ortho, maxIndexRow, maxIndexCol) ); //Lower Right Boundary
    boundaries = utilities.ConvertImageCoordToLayoutCoord(boundaries, _blockWidth, _blockHeight);
    emit UpdateObstaclesPos( utilities.ConvertImageCoordToLayoutCoord(obstaclesCoord, _blockWidth, _blockHeight), boundaries->at(0).x(), boundaries->at(1).x(), boundaries->at(1).y(), boundaries->at(0).y() );
    
    //Set Max Scores
    int maxScore = (cookiesCoord->size() * COOKIES_SCORE) + (bonusCoord->size() * BONUS_SCORE);
    int scoreGhostMax = GHOSTS_BASE_SCORE;
    for(int i = 0; i < ghostsCoord->size(); i++)
    {
	scoreGhostMax *= 2; 
	maxScore += bonusCoord->size() * scoreGhostMax;
    }
    emit SendMaxScore(maxScore, LIVES_BASE);
    
    //Set Scores
    score = SCORE_BASE;
    lives = LIVES_BASE;
    scoreGhosts = GHOSTS_BASE_SCORE;
    emit UpdateScores(score, lives);
    
    update();
}

void GLWidget::SetPacmanCommand(int aPacmanCommand, int id)
{
    //TODO :modify pacman message to be able to receive up to two pacman actions
    
    pacmanArray[id]->action = static_cast<Pacman::Action>(aPacmanCommand);
}

void GLWidget::ToggleGhostModeSlot()
{
    for(int i = 0;i < nGhosts;i++)
	ghostsArray[i]->ToggleMode();
    
    if (contGhostModePhases < 8)
    {
	ghostModeTimer->start(ghostModeTimes[contGhostModePhases]);
	contGhostModePhases++;
    }
    isInFrightenedMode = false;
}

void GLWidget::EndOfFrightenedGhostModeSlot()
{
    for(int i = 0;i < nGhosts;i++)
	ghostsArray[i]->RecoverFromFrigthenedMode();
    
    if ((contGhostModePhases < 8) && (ghostRemainingTime > 0))
	ghostModeTimer->start(ghostRemainingTime);
    
    isInFrightenedMode = false;
}

void GLWidget::reviveGhost0Slot()
{
    ghostsArray[0]->deadGhost = false;
}

void GLWidget::reviveGhost1Slot()
{
    ghostsArray[1]->deadGhost = false;
}

void GLWidget::reviveGhost2Slot()
{
    ghostsArray[2]->deadGhost = false;
}

void GLWidget::reviveGhost3Slot()
{
    ghostsArray[3]->deadGhost = false;
}

void GLWidget::EndOfDeadPacmanSlot()
{
    for(int j = 0;j < nPacman;j++)
    {
	pacmanArray[j]->currentPosition = pacmanArray[j]->_initialPosition;
	pacmanArray[j]->action = Pacman::Action::None;
    }
    
    for(int k = 0;k < ghostsCoord->size();k++)
    {
	ghostsArray[k]->currentPosition = ghostsArray[k]->_initialPosition;
	ghostsArray[k]->SetInitialMode();
    }
    
    contGhostModePhases = 0;
    ghostRemainingTime = 0;    
    ghostModeTimer->start(ghostModeTimes[contGhostModePhases]);
    
    emit EndOfDeadPacmanSignal();
}

void GLWidget::receiveArrowKey(int key)
{
    if(key == Qt::Key_D)
	pacmanArray[0]->action = static_cast<Pacman::Action>(2);
    else if(key == Qt::Key_A)
	pacmanArray[0]->action = static_cast<Pacman::Action>(3);
    else if(key == Qt::Key_W)
	pacmanArray[0]->action = static_cast<Pacman::Action>(0);
    else if(key == Qt::Key_S)
	pacmanArray[0]->action = static_cast<Pacman::Action>(1);
}

void GLWidget::ChangeFrightenedFigSlot()
{
    for(int i = 0;i < nGhosts;i++)
    {
	if((ghostsArray[i]->isFrightened()) && (ghostsArray[i]->frightenedMode == 1))
	    ghostsArray[i]->frightenedMode = 2;
	else if ((ghostsArray[i]->isFrightened()) && (ghostsArray[i]->frightenedMode == 2))
	    ghostsArray[i]->frightenedMode = 1;
    }
}

bool GLWidget::IsPlaying()
{
    return allowToPlay;
}

void GLWidget::setNumberOfPacmans(int numberOfPacmans)
{
    nPacman = numberOfPacmans;
}

void GLWidget::setMute()
{
    mute = true;
}
