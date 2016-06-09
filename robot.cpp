#include "robot.h"

Robot::Robot(Map *myMap)
{
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<float> dis(30.0, myMap->world_size-30);
    uniform_real_distribution<float> dis2(-M_PI,M_PI);
    robot_pose.x = dis(gen);
    robot_pose.y = dis(gen);
    robot_pose.orientation = dis2(gen);

    noises.forward_noise = 0.0;
    noises.turn_noise = 0.0;
    noises.sense_noise = 0.0;

    map = myMap;
}

void Robot::set_position(position n){
    if(n.x > map->world_size || n.x < 0) cout<<"X coordinate out of bound"<<endl;
    if(n.y > map->world_size || n.y < 0) cout<<"Y coordinate out of bound"<<endl;
//    if(n.orientation > -M_PI || n.orientation < M_PI) cout<<"Orientation coordinate out of bound"<<endl;
    robot_pose.x = n.x;
    robot_pose.y = n.y;
    robot_pose.orientation = n.orientation;

}

void Robot::set_noise(float foward, float turn, float sense){
    noises.forward_noise = foward;
    noises.turn_noise = turn;
    noises.sense_noise = sense;
}

vector<float> Robot::sense(){
    vector<float> Z;
    float dist;
    std::default_random_engine generator;
    normal_distribution<float> dis(0.0, noises.sense_noise);

    for(int i = 0; i < map->landmarks.size(); i++){
        dist = sqrt(pow(robot_pose.x - map->landmarks[i].x,2) + pow(robot_pose.y - map->landmarks[i].y,2));
        //dist += randomG(generator);
        dist += dis(generator);
        Z.push_back(dist);
    }
    return Z;
}

//void Robot::move(pose new_pose){

//    position aux;
//    std::default_random_engine generator;
//    normal_distribution<float> randomT(0.0,noises.turn_noise);
//    normal_distribution<double> randomF(0.0,noises.forward_noise);

//    //turn, and add randomness to the turning command

//    aux.x = robot_pose.x + cos(robot_pose.orientation)*new_pose.x - sin(robot_pose.orientation)*new_pose.y;//+ randomF(generator)
//    aux.y = robot_pose.y + sin(robot_pose.orientation)*new_pose.x + cos(robot_pose.orientation)*new_pose.y;//+ randomF(generator)
//    aux.orientation = robot_pose.orientation + new_pose.theta; //+ randomT(generator)
//    cout<<"1 - Orientação:"<<aux.orientation<<endl;
//    while(aux.orientation > M_PI)
//        aux.orientation -= 2*M_PI;
//    while(aux.orientation < -M_PI)
//        aux.orientation += 2*M_PI;
//    cout<<"2 - Orientação:"<<aux.orientation<<endl;

//    set_position(aux);
//    set_noise(noises.forward_noise, noises.turn_noise, noises.sense_noise);

//}

void Robot::move(movement mv){
    position aux;
    std::default_random_engine generator;
    normal_distribution<float> randomT(0.0,noises.turn_noise);
    normal_distribution<double> randomF(0.0,noises.forward_noise);

    if(mv.dist > 0){
        //turn, and add randomness to the turning command

        float orient = robot_pose.orientation + mv.angle + randomT(generator);
        //orient = fmod(orient,2*M_PI);
        orient = mod(orient, 2*M_PI);

        //move, and add randomness to the motion command
        float dist = mv.dist + randomF(generator);
        aux.x = robot_pose.x + (cos(orient) * dist);
        aux.y = robot_pose.y + (sin(orient) * dist);
        aux.x = fmod(aux.x,map->world_size);
        aux.y = fmod(aux.y,map->world_size);
        aux.orientation = orient;

        //set
        set_position(aux);
        set_noise(noises.forward_noise, noises.turn_noise, noises.sense_noise);
    }else{
        cout<<"DIST == 0"<<endl;
    }
}



//calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
float Robot::gaussian(float dist, float sense_noise, float measurement){

    return exp(-(pow(dist - measurement,2))/pow(sense_noise,2) / 2.0) / sqrt(2.0 * M_PI * pow(sense_noise,2));
}

//calculates how likely a measurement should be
float Robot::measurement_prob(vector<float> measurements){
    float prob = 1.0;
    float dist;

    for(int i = 0; i < map->landmarks.size(); i++){
        dist = sqrt(pow(robot_pose.x - map->landmarks[i].x,2) + pow(robot_pose.y - map->landmarks[i].y,2));
        prob *= gaussian(dist, noises.sense_noise, measurements[i]);
    }
    return prob;
}

void Robot::representation(){
    cout<<"ROBOT POSITION - X:["<<robot_pose.x<<"] Y:["<<robot_pose.y<<"] Ori:["<<robot_pose.orientation<<"]"<<endl;
}

void Robot::show_measur(vector<float> measur){
    for(int i = 0; i < measur.size(); i++){
        cout<<i+1<<":"<<"["<<measur[i]<<"]"<<" ";
    }
    cout<<endl;
}

float Robot::mod(float a, float b){
    return a - b*floor(a/b);
}
