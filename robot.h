#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <iostream>
#include <random>
#include <time.h>
#include <math.h>
#include <map.h>


using namespace std;

typedef struct position{
    float x;
    float y;
    float orientation;
}position;

typedef struct noise{
    float forward_noise;
    float turn_noise;
    float sense_noise;
}noise;

typedef struct movement{
    int dist;
    float angle;
}movement;

class Robot
{

private:
    Map *map;

public:
    Robot(Map *myMap);

    position robot_pose;
    noise noises;
    vector< vector<float> > Measurements;
    vector<float> temp2;


    void init_map();
    void set_position(position n);
    void set_noise(float foward, float turn, float sense);
    void move(movement mv);
    float measurement_prob(vector<float> measurements);
    float gaussian(float dist, float sense_noise, float measurement);
    void representation();
    void show_measur(vector<float> measur);
    float mod(float a, float b);
    vector<float> sense();











};

#endif // ROBOT_H
