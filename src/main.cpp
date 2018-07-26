#include <QApplication>
#include "pacman/window.h"
#include "ros/ros.h"
#include "pacman/pacmanPos.h"
#include "pacman/pos.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Window mainWindow;
    //mainWindow.resize(mainWindow.sizeHint());
    mainWindow.show();
    return app.exec();
}
