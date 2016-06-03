#include <QCoreApplication>
#include <robot.h>
#include <mcl.h>
#include <map.h>
#include <iostream>
#include <mainwindow.h>
#include <unistd.h>

using namespace std;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Map *myMap = new Map(580); //Size of map
    Robot robo(myMap);
    Mcl myMcl(1000,myMap); //Amount of particles
    MainWindow mw(myMap->world_size);
    thread_mcl tmcl(&robo,&myMcl,myMap,300,false); //TRUE = LOCAL - FALSE = GLOBAL

    tmcl.start(); //Thread starts
    mw.show(); //Show the image

    return a.exec();
}
