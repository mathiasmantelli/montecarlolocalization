#include "mcl.h"

Mcl::Mcl(int num_pat, Map *MyMap){
    num_particles = num_pat;
    map = MyMap;

    minscale = 0.5;
    maxscale = 1.5;
    mt19937 gen(generator());
    uniform_int_distribution<int> randomXY(5.0,MyMap->world_size-5);
    uniform_real_distribution<float> randomTh(-M_PI,M_PI);
    uniform_real_distribution<double> randomS(minscale, maxscale);
    particles.resize(num_particles);

    //Creating the particles with position, orientation, and errors
    for(int i = 0; i < particles.size(); i++){
        particles[i].x = randomXY(gen);
        particles[i].y = randomXY(gen);
        particles[i].th = randomTh(gen);
        particles[i].w = 1.0/((double)num_particles);
        particles[i].error.forward_noise = 0.0;
        particles[i].error.turn_noise = 0.0;
        particles[i].error.sense_noise = 0.0;
        particles[i].s = randomS(gen);
    }

}

void Mcl::show_particles(){
    cout<<"--------- Particles ---------"<<endl;
    for(int i = 0; i < particles.size(); i++){
        cout<<i+1<<" - ";
        cout<<"X:["<<particles[i].x<<"] Y:["<<particles[i].y<<"] Orient:["<<particles[i].th<<"]";
//        cout<<" Weight: "<<particles[i].w<<endl;
    }
}

//TO MOVE THE PARTICLES
void Mcl::sampling(movement new_pose){

    float dist;
    for(int i = 0; i < particles.size(); i++){
        position aux;
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::uniform_real_distribution<double> randomValue(-.5,.5);
        uniform_real_distribution<double> randomS(-0.1,0.1);
        uniform_int_distribution<int> randomDist(1, 3);

        float orient = particles[i].th + new_pose.angle + randomValue(generator);
        orient = mod(orient, 2*M_PI);

        dist = new_pose.dist + randomDist(generator);

        aux.x = particles[i].x + (cos(orient) * dist);
        aux.y = particles[i].y + (sin(orient) * dist);

        particles[i].x = fmod(aux.x, map->world_size);
        particles[i].y = fmod(aux.y, map->world_size);
        particles[i].th = orient;
        particles[i].s += randomS(generator);
    }
}


//TO MOVE THE PARTICLES - KLD SAMPLING modification
particle Mcl::sampling_single(particle oneP, movement new_pose){
    //RANDOM VALUES TO SAMPLING
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    uniform_int_distribution<int> randomDist2(1, 2);
    uniform_real_distribution<double> randomTh2(-.5,.5);
    uniform_real_distribution<double> randomS(-0.01,0.01);
    float dist;

    position aux;

    float orient = oneP.th + new_pose.angle + randomTh2(generator);
    orient = mod(orient, 2*M_PI);

    dist = new_pose.dist + randomDist2(generator);

    aux.x = oneP.x + (cos(orient) * dist);
    aux.y = oneP.y + (sin(orient) * dist);

    oneP.x = fmod(aux.x, map->world_size);
    oneP.y = fmod(aux.y, map->world_size);
    oneP.th = orient;
    oneP.s += randomS(generator);

    return oneP;
}

//CALCULATES HOW LIKELY A MEASUREMENT SHOULD BE
void Mcl::weight_particles(vector<float> measur){
    float dist;
    for(int i = 0; i < particles.size(); i++){
        float prob = 1.0;
        //TO VERIFY IF THE PARTICLE IS OUT OF THE MAP
        if(particles[i].x < 0 || particles[i].x > map->world_size || particles[i].y < 0 || particles[i].y > map->world_size){
            particles[i].w = 0;
        }else{
            for(int j = 0; j < map->landmarks.size(); j++){
                dist = sqrt(pow(particles[i].x - map->landmarks[j].x, 2) + pow(particles[i].y - map->landmarks[j].y, 2));
                cout<<"DIST(part):"<<dist<<" - DIST(robot):"<<measur[j]<<" "<<" - DIST*SCALE:"<<dist*particles[i].s;
                dist *= particles[i].s;
                prob *= gaussian(measur[j], particles[i].error.sense_noise, dist);
                cout<<"Result gaussian["<<j+1<<"]:"<<gaussian(measur[j], particles[i].error.sense_noise, dist)<<endl;
            }
            cout<<endl;
            particles[i].w = prob;
        }
    }
    //NORMALIZING THE WEIGHT OF THE PARTICLES
    double soma = 0;
    int count = 0;
    for(int i = 0; i < particles.size(); i++){
        soma += particles[i].w;
        if(particles[i].w == 0) count++;  //COUNT THE AMOUNT OF DEAD PARTICLES
    }
    for(int i = 0; i < particles.size(); i++)
        particles[i].w /= soma;
}

//CALCULATES HOW LIKELY A MEASUREMENT SHOULD BE - KLD SAMPLING modification
particle Mcl::weight_particles_single(particle oneP, vector<float> measur){
    float dist, prob = 1.0;
    //TO VERIFY IF THE PARTICLE IS OUT OF THE MAP
    if(oneP.x < 0 || oneP.x > map->world_size || oneP.y < 0 || oneP.y > map->world_size){
        oneP.w = 0;
    }else{
        for(int j = 0; j < map->landmarks.size(); j++){
            dist = sqrt(pow(oneP.x - map->landmarks[j].x, 2) + pow(oneP.y - map->landmarks[j].y, 2));
            dist *= oneP.s;
            prob *= gaussian(measur[j], oneP.error.sense_noise, dist);
        }
        oneP.w = prob;
    }
    return oneP;
}

//NORMALIZING THE WEIGHT OF THE PARTICLES - KLD SAMPLING modification
void Mcl::normalizing_particle(){
    double soma = 0;
    int count = 0;
    for(int i = 0; i < particles.size(); i++){
        soma += particles[i].w;
        if(particles[i].w == 0) count++;  //COUNT THE AMOUNT OF DEAD PARTICLES
    }
    for(int i = 0; i < particles.size(); i++)
        particles[i].w /= soma;
}

//calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
float Mcl::gaussian(float mu, float sigma, float x){
    return exp(-(pow(mu - x,2))/pow(sigma,2)/2.0) / sqrt(2.0 * M_PI * pow(sigma,2));
}

//ALGORITHM 18 - Principles of Robot Motion 2005, pag 318
//RESAMPLING THE PARTICLES
void Mcl::resample(){
    vector<particle> newParticles;

    vector<int> temp;
    temp.resize(num_particles,0);

    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);

    uniform_real_distribution<double> randomV(0.0, 1.0/((double)num_particles));

    int i = 0;
    double r = randomV(gen);
    double c = particles[i].w;
    double div = 1.0/((double)num_particles);
    for(int j = 0; j < particles.size(); j++){
        double u = r + j*div;
        while(u > c){
            i++;
            c = c + particles[i].w;
        }
        temp[i]++;
    }

    set<pair<int, int> > particlesSet;
    set<pair<int, int> >::iterator it;

    for(int i = 0; i < particles.size(); i++){
        if(temp[i] > 0){
            pair<int, int> p(particles[i].x, particles[i].y);
            if(particlesSet.find(p) == particlesSet.end()){
                newParticles.push_back(particles[i]);
                particlesSet.insert(p);
            }
        }
    }

    for(int i = 0; i < newParticles.size(); i++){
        newParticles[i].w = div;
    }
    particles = newParticles;
}

//RENATA'S ALGORITHM - RESAMPLING THE PARTICLES
void Mcl::resample_Roleta(){
    vector <particle> populacaoAux;
    swap(populacaoAux,particles);
    particles.clear();

    double somaPesos=0.0;
    for(int k=0;k<populacaoAux.size();k++){
        somaPesos += populacaoAux[k].w;
    }

    if(somaPesos==0.0) cout<<"Hybrid::resampling  -  somaPesos eh 0"<<endl;

    //RECRIA A POPULACAO COM A ROLETA
    for(int k=0;k<num_particles;k++){
        int l;
        double r= (double)rand() / RAND_MAX;
        l=-1;
        do{
            l++;
            if(l>populacaoAux.size()-1){
                l=populacaoAux.size()-1;
            }
            r=r-populacaoAux[l].w;
        } while(r>0.0);
        particles.push_back(populacaoAux[l]);
    }
}

particle Mcl::resample_Roleta_single(){
    double somaPesos=0.0;
    for(int k=0;k<particles.size();k++){
        somaPesos += particles[k].w;
    }

    if(somaPesos==0.0) cout<<"Hybrid::resampling  -  somaPesos eh 0"<<endl;

    //RECRIA A POPULACAO COM A ROLETA
    int l;
    double r= (double)rand() / RAND_MAX;
    //           cout<<"r="<<r<<endl;
    l=-1;
    do{
        l++;
        if(l>particles.size()-1){
            l=particles.size()-1;
        }
        r=r-particles[l].w;
    } while(r>0.0);
    return particles[l];
}

void Mcl::set_noise_particles(float foward, float turn, float sense){
    for(int i = 0; i < particles.size(); i++){
        particles[i].error.forward_noise = foward;
        particles[i].error.turn_noise = turn;
        particles[i].error.sense_noise = sense;
    }
}

float Mcl::max_element(){
    float max = -1000;

    for(int i = 0; i < particles.size(); i++){
        if(particles[i].w > max)
            max = particles[i].w;
    }
    return max;
}

//CREATING THE IMAGE TO SHOW
QImage Mcl::Gera_Imagem_Pixmap(Robot *robo){

    int tam = map->world_size;
    QImage img(tam,tam,QImage::Format_ARGB32);
    img.fill(QColor(Qt::white).rgb());

    //BRACKGROUND COLOR
    for (int x = 0; x < tam; ++x)
        for (int y = 0; y < tam; ++y)
            img.setPixel(x,y,qRgb(255, 255, 255));

    //DRAWING PARTICLES
    for(int i = 0; i < particles.size(); i++){
        img = point(particles[i].x, particles[i].y,3,0,(1 - particles[i].w)*255,0,img);
//        cout<<"Weight: "<<particles[i].w<<endl;
        //PARTICLES' DIRECTION
        float coss,senn;
        coss = cos(particles[i].th);
        senn = sin(particles[i].th);
        for(int j = 2; j < 7; j++)
            img.setPixel(particles[i].x + coss*j, particles[i].y + senn*j, qRgb(0, 0, 0));

    }
    //DRAWING ROBOT
    img = point(robo->robot_pose.x,robo->robot_pose.y,3,255, 0, 255,img);

    //ROBOT'S DIRECTION
    float coss,senn;
    coss = cos(robo->robot_pose.orientation);
    senn = sin(robo->robot_pose.orientation);
    for(int i = 2; i < 7; i++)
        img.setPixel(robo->robot_pose.x + coss*i, robo->robot_pose.y + senn*i, qRgb(0, 0, 0));

    //DRAWING LANDMARKS
//    for(int i = 0; i < map->landmarks.size(); i++)
//        img = point(map->landmarks[i].x,map->landmarks[i].y,4,0,255,0,img);

    return img;
}

QImage Mcl::point(int x, int y, int tam, int r, int g, int b, QImage img){
    img.setPixel(x,y,qRgb(r,g,b));
    for(int i = 1; i <= tam; i++){
        img.setPixel(min(x+i, map->world_size),y, qRgb(r,g,b));
        img.setPixel(max(x-i, 0),y, qRgb(r,g,b));
        img.setPixel(x,min(y+i, map->world_size), qRgb(r,g,b));
        img.setPixel(x,max(y-i, 0), qRgb(r,g,b));
    }
    return img;
}

//SET PARTICLES' POSITION
void Mcl::set_position(position pos){
    for(int i = 0; i < particles.size(); i++){
        particles[i].x = pos.x;
        particles[i].y = pos.y;
        particles[i].th = pos.orientation;
    }
}

float Mcl::mod(float a, float b){
    return a - b*floor(a/b);
}

float Mcl::number_effective(){
    float neff = 0;
    for(int i = 0; i < particles.size(); i++){
        neff += pow(particles[i].w,2);
    }
//    cout<<"NEFF: "<<1/neff<<endl;
    return 1/neff;
}

/**
   Initialize a round of KLD sampling.  Takes in kld-parameters:
   quantile, kld-error, bin size, minimum number of samples.

   Some extra informations:
   -quantile: Probability that the KL-distance between the discrete, sampled distribution (set by bin-size) is
              less than ERROR from the true distribution. Must be between 0.5 and 1.0.

   -kld-error: Target KL-distance between discrete, sampled distribution and true underlying distribution. Must
               be greater than 0.

   -bin size: KLD-sampling uses discrete bins to detect how well the underlying distribution is being sampled.
              The larger the bins, the fewer particles needed, but the less accurate the discrete, sampled
              distribution will be. Value must be greater than 0.

   -minimum number of samples: Minimum number of samples. KLD-sampling always samples at least 10 samples.
**/

