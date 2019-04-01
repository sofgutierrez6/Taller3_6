#include <string.h>
#include "ros/ros.h"
#include "pacman/actions.h"
#include "pacman/pacmanPos.h"
#include "pacman/ghostsPos.h"
#include "pacman/cookiesPos.h"
#include "pacman/bonusPos.h"
#include "pacman/game.h"
#include "pacman/pos.h"
#include "pacman/mapService.h"
#include "pacman/performance.h"

void performanceCallback(const pacman::performance::ConstPtr& msg)
{
    //ROS_INFO("Lives %d, score: %d, time: %d, performEval: %f",msg->lives, msg->score, msg->gtime, msg->performEval);
  
}
void pacmanPosCallback(const pacman::pacmanPos::ConstPtr& msg)
{
  //ROS_INFO("Num Pacman: %d",msg->nPacman);
  for(int i = 0; i < msg->nPacman; i++)
  {
    //ROS_INFO("Pos Pacman [%d] :  x = [%d]  y = [%d]",i ,msg->pacmanPos.x, msg->pacmanPos.y);
  }
}
void ghostsPosCallback(const pacman::ghostsPos::ConstPtr& msg)
{
  //ROS_INFO("Num Fantasmas: %d",msg->nGhosts);
  for(int i = 0; i < msg->nGhosts; i++)
  {
    //ROS_INFO("Pos Fantasma [%d] :  x = [%d]  y = [%d]",i ,msg->ghostsPos[i].x, msg->ghostsPos[i].y);
    //ROS_INFO("Modo Fantasma [%d] :  [%d]",i ,msg->mode[i]);
  }
}
void cookiesPosCallback(const pacman::cookiesPos::ConstPtr& msg)
{
  //ROS_INFO("Num Cookies: %d",msg->nCookies);
  for(int i = 0; i < msg->nCookies; i++)
  {
    //ROS_INFO("Pos cookies [%d] :  x = [%d]  y = [%d]",i ,msg->cookiesPos[i].x, msg->cookiesPos[i].y);
  }
}
void bonusPosCallback(const pacman::bonusPos::ConstPtr& msg)
{
  //ROS_INFO("Num Bonus: %d",msg->nBonus);
  for(int i = 0; i < msg->nBonus; i++)
  {
    //ROS_INFO("Pos bonus [%d] :  x = [%d]  y = [%d]",i ,msg->bonusPos[i].x, msg->bonusPos[i].y);
  }
}
void gameStateCallback(const pacman::game::ConstPtr& msg)
{
  //ROS_INFO("Game State: %d",msg->state);
}
int main(int argc, char **argv)
{
    bool inicio = false;
    char cadena[13];
    char cadenaAccion[15];
    srand (time(NULL));
    ros::init(argc, argv, "pacman_controller_phone");
    ros::NodeHandle n;
    strcpy(cadena, "pacmanCoord0");
    strcpy(cadenaAccion, "pacmanActions0");
    ros::Publisher chatter_pub = n.advertise<pacman::actions>(cadenaAccion, 100);
    ros::Subscriber posPacmanSub = n.subscribe(cadena, 100, pacmanPosCallback);
    ros::Subscriber posGhostsSub = n.subscribe("ghostsCoord", 100, ghostsPosCallback);
    ros::Subscriber posCookiesSub = n.subscribe("cookiesCoord", 100, cookiesPosCallback);
    ros::Subscriber posBonuSub = n.subscribe("bonusCoord", 100, bonusPosCallback);
    ros::Subscriber gameStateSub = n.subscribe("gameState", 100, gameStateCallback);
    ros::Subscriber performEvalSub = n.subscribe("performanceEval", 100, performanceCallback);
    ros::ServiceClient mapRequestClient = n.serviceClient<pacman::mapService>("pacman_world");
    pacman::mapService srv;
    srv.request.name = "Controller cpp";
    if(mapRequestClient.call(srv))
    {	
	printf("# obs: %d \n", srv.response.nObs);
	printf("minX: %d maxX: %d \n", srv.response.minX, srv.response.maxX);
	printf("minY: %d maxY: %d \n", srv.response.minY, srv.response.maxY);
	inicio = true;
    }
    else
    {
      printf("Error!! Make sure pacman_world node is running \n");
    }
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok() && inicio == true)
    {
      pacman::actions msg;
      msg.action = rand() % 5;
      chatter_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
  return 0;
}