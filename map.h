#ifndef MAP_H
#define MAP_H
#include <vector>

using namespace std;

typedef struct coordinate{
    int x,y;
}coordinate;

typedef struct pose{
    int x,y;
    float theta;
}pose;

class Map
{
public:
    Map(int world);
    int world_size;
    static vector<coordinate> landmarks;
    vector< vector <int> > empty;
    void set_empty_map();
};

#endif // MAP_H
