#include "pacman/Ghosts.h"

Ghosts::Ghosts(QPoint initialPosition, Ghosts::Personality aCharacter, int aHeight, int aWidth, QPoint initialPacmanPosition, int cMapHeight, int cMapWidth, bool *cObstacles)
{
    mode = Mode::Initial;
    previousMode = Mode::Initial;
    currentPosition = initialPosition;
    _initialPosition = initialPosition;
    character = aCharacter;
    orientation = 180.0;
    height = aHeight;
    width = aWidth;
    action = Action::None;
    pacmanPosition = initialPacmanPosition;
    mapHeight = cMapHeight;
    mapWidth = cMapWidth;
    changedMode = false;
    obstacles = new bool[(mapHeight)*(mapWidth)];
    deadGhost = false;
    frightenedMode = 1;
    frightenedCount = 1;
    memcpy(obstacles, cObstacles, (mapHeight)*(mapWidth)*sizeof(bool));
        
    if (character == Ghosts::Personality::Shadow)
    {
	name = "Blinky";
	scatterTargetPosition = QPoint(0.5*mapWidth - width, 0.5*mapHeight + height);
    }
    else if (character == Ghosts::Personality::Speedy)
    {
	name = "Pinky";
	scatterTargetPosition = QPoint(-0.5*mapWidth + width, 0.5*mapHeight + height);
    }
    else if (character == Ghosts::Personality::Bashful)
    {
	name = "Inky";
	scatterTargetPosition = QPoint(0.5*mapWidth, -0.5*mapHeight - height);
    }
    else if (character == Ghosts::Personality::Pokey)
    {
	name = "Clyde";
	scatterTargetPosition = QPoint(-0.5*mapWidth, -0.5*mapHeight - height);
    }
    GetTexId();
}

int Ghosts::GetTexId()
{
    if (mode == Mode::Frightened)
    {
	if (frightenedMode == 1)
	    texId = 18;
	else
	    texId = 19;
    }
    else if (character == Ghosts::Personality::Shadow)
    { 
	if (orientation == 90)
	    texId = 2;
	else if (orientation == 270)
	    texId = 3;
	else if (orientation == 0)
	    texId = 4;
	else if (orientation == 180)
	    texId = 5;
    }
    else if (character == Ghosts::Personality::Speedy)
    { 
	if (orientation == 90)
	    texId = 6;
	else if (orientation == 270)
	    texId = 7;
	else if (orientation == 0)
	    texId = 8;
	else if (orientation == 180)
	    texId = 9;
    }
    else if (character == Ghosts::Personality::Bashful)
    { 
	if (orientation == 90)
	    texId = 10;
	else if (orientation == 270)
	    texId = 11;
	else if (orientation == 0)
	    texId = 12;
	else if (orientation == 180)
	    texId = 13;
    }
    else if (character == Ghosts::Personality::Pokey)
    { 
	if (orientation == 90)
	    texId = 14;
	else if (orientation == 270)
	    texId = 15;
	else if (orientation == 0)
	    texId = 16;
	else if (orientation == 180)
	    texId = 17;
    }
    else
	texId = -1;
    
    return texId;
}

void Ghosts::UpdateGhostPosition(QPoint newPacmanPosition, double newPacmanOrientation, QPoint newBlinkysPosition)
{
    int stepX = width;
    int stepY = height;
    pacmanPosition = newPacmanPosition;
    pacmanOrientation = newPacmanOrientation;
    blinkysPosition = newBlinkysPosition;
    
    if ((mode == Ghosts::Mode::Frightened) && (frightenedCount == 1))
	frightenedCount = 2;
    else
    {
	if (action == Ghosts::Action::None)
	{
	    QPoint coordLeft(currentPosition.x() - stepX, currentPosition.y());
	    QPoint coordRight(currentPosition.x() + stepX, currentPosition.y());
	    QPoint coordUp(currentPosition.x(), currentPosition.y() + stepY);
	    QPoint coordDown(currentPosition.x(), currentPosition.y() - stepY);
	    
	    if (!obstacles[utilities.GetIndexRowFromCoord(coordLeft, mapHeight)*mapWidth + utilities.GetIndexColFromCoord(coordLeft, mapWidth)])
		action = Ghosts::Action::Left;
	    else if (!obstacles[utilities.GetIndexRowFromCoord(coordDown, mapHeight)*mapWidth + utilities.GetIndexColFromCoord(coordDown, mapWidth)])
		action = Ghosts::Action::Down;
	    else if (!obstacles[utilities.GetIndexRowFromCoord(coordRight, mapHeight)*mapWidth + utilities.GetIndexColFromCoord(coordRight, mapWidth)])
		action = Ghosts::Action::Right;
	    else if (!obstacles[utilities.GetIndexRowFromCoord(coordUp, mapHeight)*mapWidth + utilities.GetIndexColFromCoord(coordUp, mapWidth)])
		action = Ghosts::Action::Up;
	}
	else
	{	
	    if (!changedMode)
	    {
		QVector<Ghosts::Action> *possibleActions;
		possibleActions = new QVector<Ghosts::Action>;
		GetPossibleActions(possibleActions);

		if (possibleActions->size() == 1)
		    action = possibleActions->at(0);
		else if (possibleActions->size() == 2)
		{
		    DeleteReverseAction(action, possibleActions);
		    action = possibleActions->at(0); 
		}
		else if (possibleActions->size() > 2)
		{
		    DeleteReverseAction(action, possibleActions);
		    if (mode == Mode::Frightened || mode == Mode::Initial)
			action = possibleActions->at(rand() % possibleActions->size());
		    else
		    {
			CalculateTargetPosition();
			ComputeGhostDecision(possibleActions);
		    }    
		}
	    }
	    else
	    {
		if(action == Ghosts::Action::Down)
		  action = Ghosts::Action::Up;
		else if(action == Ghosts::Action::Up)
		    action = Ghosts::Action::Down;
		else if(action == Ghosts::Action::Right)
		    action = Ghosts::Action::Left;
		else if(action == Ghosts::Action::Left)
		    action = Ghosts::Action::Right;
		changedMode = false;
	    }
	}
	
	if (deadGhost)
	{
	    mode = previousMode;
	    currentPosition = _initialPosition;
	    orientation = 180.0;
	}
	else
	{
	    if (action == Ghosts::Action::Left)
	    {
		currentPosition.setX(currentPosition.x() - stepX);
		if (mode == Mode::Frightened)
		    orientation = 0;
		else
		    orientation = 180.0;
	    }
	    else if (action == Ghosts::Action::Right)
	    {
		currentPosition.setX(currentPosition.x() + stepX);
		if (mode == Mode::Frightened)
		    orientation = 0;
		else
		    orientation = 0.0;
	    }
	    else if (action == Ghosts::Action::Up)
	    {
		currentPosition.setY(currentPosition.y() + stepY);
		if (mode == Mode::Frightened)
		    orientation = 0;
		else
		    orientation = 90;
	    }
	    else if (action == Ghosts::Action::Down)
	    {
		currentPosition.setY(currentPosition.y() - stepY);
		if (mode == Mode::Frightened)
		    orientation = 0;
		else
		    orientation = 270;
	    }
	}
	frightenedCount = 1;
    }
}

void Ghosts::CalculateTargetPosition()
{
    if (mode == Mode::Scatter)
	targetPosition = scatterTargetPosition;
    else if (mode == Mode::Chase)
    {
	if (character == Ghosts::Personality::Shadow)
	    targetPosition = pacmanPosition;
	else if (character == Ghosts::Personality::Speedy)
	{
	    if (pacmanOrientation == 0.0)
	    {
		targetPosition = pacmanPosition;
		targetPosition.setX(targetPosition.x() + 4*width); 
	    }
	    else if (pacmanOrientation == 90.0)
	    {
		targetPosition = pacmanPosition;
		targetPosition.setY(targetPosition.y() + 4*height); 
	    }
	    else if (pacmanOrientation == 180.0)
	    {
		targetPosition = pacmanPosition;
		targetPosition.setX(targetPosition.x() - 4*width); 
	    }
	    else if (pacmanOrientation == 270.0)
	    {
		targetPosition = pacmanPosition;
		targetPosition.setY(targetPosition.y() - 4*height); 
	    }
	}
	else if (character == Ghosts::Personality::Bashful)
	{
	    if (pacmanOrientation == 0.0)
	    {
		targetPosition = pacmanPosition;
		targetPosition.setX(targetPosition.x() + 2*width);
	    }
	    else if (pacmanOrientation == 90.0)
	    {
		targetPosition = pacmanPosition;
		targetPosition.setY(targetPosition.y() + 2*height); 
	    }
	    else if (pacmanOrientation == 180.0)
	    {
		targetPosition = pacmanPosition;
		targetPosition.setX(targetPosition.x() - 2*width); 
	    }
	    else if (pacmanOrientation == 270.0)
	    {
		targetPosition = pacmanPosition;
		targetPosition.setY(targetPosition.y() - 2*height); 
	    }
	    targetPosition += QPoint(targetPosition.x() - blinkysPosition.x(), targetPosition.y() - blinkysPosition.y());
	}
	else if (character == Ghosts::Personality::Pokey)
	{
	    int distanceToPacman = utilities.ComputeDistanceBetweenPoints(pacmanPosition, currentPosition);
	    if (distanceToPacman > 8*width)
		targetPosition = pacmanPosition;
	    else
		targetPosition = scatterTargetPosition;
	}
    }
}

int Ghosts::GetPossibleActions(QVector<Ghosts::Action>* possibleActions)
{
    QPoint coordLeft(currentPosition.x() - (int)(width*0.5) - 1, currentPosition.y());
    QPoint coordDown(currentPosition.x(), currentPosition.y() - (int)(height*0.5) - 1);
    QPoint coordRight(currentPosition.x() + (int)(width*0.5)  + 1, currentPosition.y());
    QPoint coordUp(currentPosition.x(), currentPosition.y() + (int)(height*0.5) + 1);

    if (!obstacles[utilities.GetIndexRowFromCoord(coordLeft, mapHeight)*mapWidth + utilities.GetIndexColFromCoord(coordLeft, mapWidth)])
	possibleActions->append(Ghosts::Action::Left);
    if (!obstacles[utilities.GetIndexRowFromCoord(coordDown, mapHeight)*mapWidth + utilities.GetIndexColFromCoord(coordDown, mapWidth)])
	possibleActions->append(Ghosts::Action::Down);
    if (!obstacles[utilities.GetIndexRowFromCoord(coordRight, mapHeight)*mapWidth + utilities.GetIndexColFromCoord(coordRight, mapWidth)])
	possibleActions->append(Ghosts::Action::Right);
    if (!obstacles[utilities.GetIndexRowFromCoord(coordUp, mapHeight)*mapWidth + utilities.GetIndexColFromCoord(coordUp, mapWidth)])
	possibleActions->append(Ghosts::Action::Up);
}

void Ghosts::PrintPossibleActions(QVector<Ghosts::Action>* possibleActions)
{
    for (int i = 0;i < possibleActions->size();i++)
    {
	if(possibleActions->at(i) == Ghosts::Action::Down)
	    cout << "   Down at " << i << endl;
	if(possibleActions->at(i) == Ghosts::Action::Left)
	    cout << "   Left " << i << endl;
	if(possibleActions->at(i) == Ghosts::Action::Up)
	    cout << "   Up " << i << endl;
	if(possibleActions->at(i) == Ghosts::Action::Right)
	    cout << "   Right " << i << endl;
    }
}

bool Ghosts::IsPossibleAction(Ghosts::Action anAction, QVector<Ghosts::Action>* possibleActions)
{
    bool isPossible = false;
    for (int i = 0;i < possibleActions->size();i++)
    {
	if (possibleActions->at(i) == anAction)
	    isPossible |= true;
    }
    return isPossible;
}

void Ghosts::DeleteReverseAction(Ghosts::Action anAction, QVector< Ghosts::Action >* possibleActions)
{
    Ghosts::Action reverseAction = Ghosts::Action::None;
    if(action == Ghosts::Action::Down)
	reverseAction = Ghosts::Action::Up;
    else if(action == Ghosts::Action::Up)
	reverseAction = Ghosts::Action::Down;
    else if(action == Ghosts::Action::Right)
	reverseAction = Ghosts::Action::Left;
    else if(action == Ghosts::Action::Left)
	reverseAction = Ghosts::Action::Right;
    
    for (int i = 0;i < possibleActions->size();i++)
    {
	if (possibleActions->at(i) == reverseAction)
	    possibleActions->remove(i);
    }
}

void Ghosts::ToggleMode()
{
    if (mode == Ghosts::Mode::Scatter)
	mode = Ghosts::Mode::Chase;
    else
	mode = Ghosts::Mode::Scatter;
    
    changedMode = true;
}

void Ghosts::ComputeGhostDecision(QVector<Ghosts::Action> *possibleActions)
{
    int minDist = 1000000000;
    QPoint futurePosition;
    for (int i = 0;i < possibleActions->size();i++)
    {
	if (possibleActions->at(i) == Ghosts::Action::Left)
	{
	    futurePosition.setX(currentPosition.x() - (int)(width*0.5) - 1);
	    futurePosition.setY(currentPosition.y());
	}
	else if (possibleActions->at(i) == Ghosts::Action::Right)
	{
	    futurePosition.setX(currentPosition.x() + (int)(width*0.5)  + 1);
	    futurePosition.setY(currentPosition.y());
	}
	else if (possibleActions->at(i) == Ghosts::Action::Up)
	{
	    futurePosition.setX(currentPosition.x());
	    futurePosition.setY(currentPosition.y() + (int)(height*0.5) + 1);
	}
	else if (possibleActions->at(i) == Ghosts::Action::Down)
	{
	    futurePosition.setX(currentPosition.x());
	    futurePosition.setY(currentPosition.y() - (int)(height*0.5) - 1);
	}
	  
	if (utilities.ComputeDistanceBetweenPoints(futurePosition, targetPosition) < minDist)
	{
	    action = possibleActions->at(i);
	    minDist = utilities.ComputeDistanceBetweenPoints(futurePosition, targetPosition);
	}
    }
}

void Ghosts::SetFrigthenedMode()
{
    if (mode != Mode::Frightened)
	previousMode = mode;
    mode = Mode::Frightened;
    frightenedCount = 1;
    frightenedMode = 1;
}

void Ghosts::RecoverFromFrigthenedMode()
{
    mode = previousMode;
}

bool Ghosts::isFrightened()
{
    if (mode == Mode::Frightened)
	return true;
    else
	return false;
}

void Ghosts::SetInitialMode()
{
    mode = Mode::Initial;
}