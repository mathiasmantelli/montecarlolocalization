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
    //--------- SETTINGS -----------
    int sizeMap = 580;
    int amountPartic = 2000;
    int timeThread = 300;
    bool kindLocalization = false; //TRUE = LOCAL - FALSE = GLOBAL
    bool kld = true;            //TRUE = KLD - FALSE = NON KLD
    //------------------------------

    QApplication a(argc, argv);
    Map *myMap = new Map(sizeMap); //Size of map
    Robot robo(myMap);
    Mcl myMcl(amountPartic, myMap); //Amount of particles
    MainWindow mw(myMap->world_size);
    thread_mcl tmcl(&robo, &myMcl, myMap, timeThread, kindLocalization, kld);

    tmcl.start(); //Thread starts
    mw.show(); //Show the image

    return a.exec();
}
