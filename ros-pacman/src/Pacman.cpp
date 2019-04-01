#include "pacman/Pacman.h"

Pacman::Pacman(QPoint initialPosition, double initialOrientation, int aHeight, int aWidth)
{
    currentPosition = initialPosition;
    orientation = initialOrientation;
    height = aHeight;
    width = aWidth;
    action = Pacman::Action::None;
    _initialPosition = initialPosition;
}