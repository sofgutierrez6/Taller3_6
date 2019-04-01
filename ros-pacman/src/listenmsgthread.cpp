#include "pacman/listenmsgthread.h"

void ListenMsgThread::callback0(const pacman::actions::ConstPtr& msg)
{
    if(msg->action < 5 && work)
      emit UpdatePacmanCommand(msg->action, 0);
}
void ListenMsgThread::callback1(const pacman::actions::ConstPtr& msg)
{
    if(msg->action < 5 && work)
      emit UpdatePacmanCommand(msg->action, 1);
}
ListenMsgThread::ListenMsgThread()
{
    work = false;
}

void ListenMsgThread::setWorkingThread(bool aWork)
{
    work = aWork;
}

void ListenMsgThread::run()
{
    ros::spin();      
}