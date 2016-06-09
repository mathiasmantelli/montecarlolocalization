#ifndef MCL_H
#define MCL_H
#include "robot.h"
#include <algorithm>
#include <QApplication>
#include <QtGui>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <utility>
#include <math.h>
#include <set>

using namespace std;

typedef struct noises{
    double sense_noise, turn_noise, forward_noise;
}noises;

typedef struct particle{
    int x, y;
    float th,w;
    noises error;
}particle;

class Mcl
{
public:
    Mcl(int num_part, Map *MyMap);
    vector<particle> particles;
    int num_particles;

    random_device generator;

    void show_particles();
    void sampling(movement new_pose);

    void weight_particles(vector<float> measur);
    void show_weight_particles();
    void set_noise_particles(float foward, float turn, float sense);
    void resample();
    QImage Gera_Imagem_Pixmap(Robot *robo);
    QImage point(int x, int y, int tam, int r, int g, int b, QImage img);
    float max_element();
    float measurement_prob(vector<float> measurements);
    float gaussian(float dist, float sense_noise, float measurement);
    void set_position(position pos);
    void resample_Roleta();
    float mod(float a, float b);
    float number_effective();
private:
    Map *map;

};

#endif // MCL_H
