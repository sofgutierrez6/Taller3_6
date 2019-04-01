^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pacman
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2018-09-22)
------------------
* Add maps
* Updated repository for final changes before launching
* Fixed performance update in between maps
* Updated README file with controller url
* Removed script to run controllers
* Merge pull request `#59 <https://github.com/carlosquinterop/ros-pacman/issues/59>`_ from eyberthrojas/master
  Se publica lives, score, gtime y performEval en topico performance ev…
* Updated README file
* Removed pacman_controller nodes
* Se publica lives, score, gtime y performEval en topico performance eval. Se actuailiza topico actions y todos los topicos se publican sincronicamente
* Merge pull request `#55 <https://github.com/carlosquinterop/ros-pacman/issues/55>`_ from Fabididi/master
  Se actualizan los Scripts para ejcución en red. Se incluye la creació…
* Merge branch 'master' into master
* Se actualizan los Scripts para ejcución en red. Se incluye la creación del workspace catkin. Se cambia la versión de ROS a Kinetic
* Updated Readme file
* Increased Player Name Text Edit size
* Merge pull request `#54 <https://github.com/carlosquinterop/ros-pacman/issues/54>`_ from eyberthrojas/master
  se incluye nombre del usuario en el servicio de inicio de juego
* Improved functionality of startPacmanServer.sh and updated README
* se incluye nombre del usuario en el servicio de inicio de juego
* Added pacgirl image
* Fixed scoreBoard file path
* Merge pull request `#53 <https://github.com/carlosquinterop/ros-pacman/issues/53>`_ from eyberthrojas/master
  Se publica en dos topicos diferentes (pacmanCoord0 y pacmanCoord1) la…
* Merge branch 'master' into master
* Se publica en dos topicos diferentes (pacmanCoord0 y pacmanCoord1) las coordenadas de los pacman, si hay dos pacman se publican en los dos y si solo hay un pacman solo se publica en pacmanCoor0, tambien el nodo pacman_world se suscribe a dos topicos para la accion de movimiento a realizar (pacmanActions0 y pacmanActions1), si hay dos pacman se suscribe a los dos topicos y si solo hay un pacman se suscribe a pacmanActions0
* Cleaned all figs
* Added --m (mute) option to mute all sounds
* Updated .gitignore
* Updated path to ScoreBoard file
* Deleted scoreBoard
* Updated .gitignore again
* Updated .ignore file to ignore score board file
* Added score board file to store the players statistics
* Merge pull request `#51 <https://github.com/carlosquinterop/ros-pacman/issues/51>`_ from Fabididi/master
  Include script for Instal for Requirements and two scrips for Network…
* Include script for Instal for Requirements and two scrips for Network running
* Added support for two pacmans to be assigned to 2 ghost
* Start centered window
* Changed cookies and bonus radius to depend on the block_size
* Added icon and window name
* Fixed the possibility of starting and pausing during game mode with play button if the game never started
* Identify the number of pacmans according to map name, included text for win and game over and added 3 seconds to start in game mode after play button is pressed
* Included restart after loosing in challenge option
* Merge pull request `#47 <https://github.com/carlosquinterop/ros-pacman/issues/47>`_ from eyberthrojas/master
  se incluye en servicio de solicitud del mapa reinicio del juego
* se incluye en servicio de solicitud del mapa reinicio del juego
* Added the possibility of loading a new pacman texture
* Merge pull request `#46 <https://github.com/carlosquinterop/ros-pacman/issues/46>`_ from oswaldoapr/master
  Creation maps with two pacmans and Implementation Performance Evaluator
* Implementation Performance Evaluator
* Creation new maps with two pacmans
* Updated and cleaned
* Cleaned
* Added init sound, almost no frightened animation and slower ghosts when frightened
* Added sounds
* Merge branch 'master' of https://github.com/carlosquinterop/ros-pacman
  It add support for playing sounds using QSound and Qt5Multimedia
* Added support for sound playing
* Added oneSecond constant and updated README file
* Added a game Timer in the window class and added a maximum time that make the player lose if reached
* Changed name of test mode to challenge mode
* Ghost positions are reset when pacman dies and reset ghosts mode toggle when pacman dies
* Merge pull request `#37 <https://github.com/carlosquinterop/ros-pacman/issues/37>`_ from Fabididi/master
  Se implementa el fin del juegos bajo las 2 posibles condiciones: muer…
* Se implementa el fin del juegos bajo las 2 posibles condiciones: muerte 3 veces de Pacman o Pacman se come todas las galletas y bonus
* Enabled pacman_controller to publish pacman actions
* Fixed size of maps and score
* Merge pull request `#36 <https://github.com/carlosquinterop/ros-pacman/issues/36>`_ from oswaldoapr/master
  Send Xmin, Xmax, Ymin, Ymax in Window Class and Add Score and Lives
* Merge branch 'master' into master
* Fix reset ghost score
* Send Xmin, Xmax, Ymin, Ymax in Window Class and Add Score and Lives
* Fixed errors from last merge
* Merge pull request `#34 <https://github.com/carlosquinterop/ros-pacman/issues/34>`_ from Fabididi/master
  Se realiza la verificación del argumento [Map_name], en caso de error…
* Merge branch 'master' into master
* Se realiza la verificación del argumento [Map_name], en caso de error de argumento, presenta mensaje de error y lista los nombres de mapas
* Merge pull request `#33 <https://github.com/carlosquinterop/ros-pacman/issues/33>`_ from eyberthrojas/master
  Se incluye estado de ghosts en el protocolo de mensajes, se crea serv…
* Se incluye estado de ghosts en el protocolo de mensajes, se crea servicio para solicitud de obstaculos y se añade mensaje con el estado de juego
* Fixed pacman and ghosts dead when interchanging tiles
* Fixed game start and added key controller in game mode
* Merge pull request `#31 <https://github.com/carlosquinterop/ros-pacman/issues/31>`_ from Fabididi/master
  Se implementa: protocolo inicio del juego emplenado dos argumentos en…
* Se implementa: protocolo inicio del juego emplenado dos argumentos en la clase window (game-test y nombre de mapa) y se  Emite señal StartedGame para inicio de juego en modo Test
* Implemented pacman and ghost deads
* Included new Ghost::Mode::Initial to avoid ghosts from starting in Frightened mode
* Fixed connect between sendMapData signal and resizeSlot
* Merge pull request `#21 <https://github.com/carlosquinterop/ros-pacman/issues/21>`_ from oswaldoapr/master
  Convert arrays of coordinates to QVector of coordinates for Pacman, Ghosts, Cookies and Bonuses
* Merge branch 'master' into master
* Add QVector for Obstacles
* Create Utility called ConvertImageCoordToLayoutCoord and Remove Cookies and Bonus when Pacman is in their positions
* Change Arrays to QVector in Coordinates of Pacman
* Change Arrays to QVector in Coordinates of Cookies, Bonus and Ghost
* Merge pull request `#20 <https://github.com/carlosquinterop/ros-pacman/issues/20>`_ from carlosquinterop/Add-Pacman-Lifes
  Add pacman lifes
* Updated .gitignore to ignore map images
* Test
* Cleaned maps
* Fixed resize of window and glWidget classes to fit the maps
* Cleaned maps
* Finished Frightened mode implementation for ghosts
* Completed ghosts search strategies
* Fixed glWidget to update ghostsCoord correctly before publishing
* Merge pull request `#15 <https://github.com/carlosquinterop/ros-pacman/issues/15>`_ from eyberthrojas/master
  se agregó la publicación de las posiciones de pacman, ghosts, cookies…
* se agregó la publicación de las posiciones de pacman, ghosts, cookies y bonus
* Changed .gitignore to upload ghost images
* Improved methods spell
* Added new ghosts images
* Added Utilities class and implemented a simple random algorithm for ghosts
* Added Pacman class and rewritten to support several ghosts and pacmans
* Merge pull request `#14 <https://github.com/carlosquinterop/ros-pacman/issues/14>`_ from eyberthrojas/master
  Se agregó mensage actualización de coordenadas de pacman
* Se agregó mensage actualización de coordenadas de pacman
* Included class Ghost to support ghosts
* Updated gitignore file
* Updated gitignore file
* Removed map images
* Changed Node initialization and subscriptions to window class
* Merge pull request `#12 <https://github.com/carlosquinterop/ros-pacman/issues/12>`_ from oswaldoapr/master
  Include cookies and bonus in mObstacles matrix and plot them
* Include cookies and bonus in mObstacles matrix and plot them
* Updated README file
* Included support for ROS messages and one simple controller node
* Included maps tools
* Fixed README typo
* Updated parameters t0 test new maps
* Updated README file to fix format
* Updated Qt5Widget_DIR variable so that catkin could find it
* Updated README file
* First upload
* Initial commit
* Contributors: CarlosQ, Eyberth Rojas, Fabian Pérez Gordillo, Oswaldo, carlosquinterop
