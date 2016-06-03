#include "map.h"

Map::Map(int world)
{
    world_size = world;
    coordinate temp;
/*
  -----------
  |1       4|
  |         |
  |         |
  |2       3|
  -----------
*/
    //LEFT TOP
    temp.x = .2*world_size;
    temp.y = .2*world_size;
    landmarks.push_back(temp);

    //LEFT DOWN
    temp.x = .2*world_size;
    temp.y = .8*world_size;
    landmarks.push_back(temp);

    //RIGHT DOWN
    temp.x = .8*world_size;
    temp.y = .8*world_size;
    landmarks.push_back(temp);

    //RIGHT UP
    temp.x = .8*world_size;
    temp.y = .2*world_size;
    landmarks.push_back(temp);
}
