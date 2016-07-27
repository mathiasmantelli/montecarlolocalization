#include "map.h"

vector<coordinate> Map::landmarks;

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

//SET EMPTY GRID
void Map::set_empty_map(){
    for(int i = 0; i < world_size; i++){
        vector<int> test;
        for(int j = 0; j < empty.size(); j++){
            test.push_back(0);
        }
        empty.push_back(test);
    }
}
