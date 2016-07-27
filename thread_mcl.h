#ifndef THREAD_MCL_H
#define THREAD_MCL_H
#include <QThread>
#include <bits/stdc++.h>
#include <QImage>
#include <mcl.h>
#include <robot.h>
#include <QApplication>
#include <QtGui>
#include <map.h>
#include <cstdlib>
#include <limits>

using namespace std;

class thread_mcl : public QThread
{
public:
    thread_mcl(Robot *robo, Mcl *myMcl, Map *myMap, int time, bool localization);
    void run();
    void run_kdl_sampling();
    void run_normal();
    bool bin_is_empty(particle oneP);
    void show_bin();
    int time;
    bool localization;

    Robot *robo;
    Mcl *myMcl;
    Map *myMap;
    static QString state;
    static QImage img;
    static QString neff;
    static bool kind;
    static int n_particles;


};

#endif // THREAD_MCL_H
