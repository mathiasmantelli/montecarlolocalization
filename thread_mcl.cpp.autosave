#include "thread_mcl.h"

QImage thread_mcl::img;
QString thread_mcl::state;
QString thread_mcl::neff;
bool thread_mcl::kind;
int thread_mcl::n_particles;

thread_mcl::thread_mcl(Robot *robo, Mcl *myMcl, Map *myMap, int time, bool localization)
{
    this->robo = robo;
    this->myMcl = myMcl;
    this->localization = localization;
    this->time = time;
    this->myMap = myMap;
    this->kind = localization;
    this->n_particles = myMcl->num_particles;
}

void thread_mcl::run(){
    this->run_kdl_sampling();
}

void thread_mcl::run_normal(){

    bool flag_neff;
    int T;

    //--- SETTING VARIABLES ---
    T = 0;
    flag_neff = false;
    //-------------------------

    robo->representation();

//    myMcl.set_position(robo.robot_pose);
    myMcl->set_noise_particles(0.01, 0.01, 4.0);
    movement mv;

    random_device generator;
    mt19937 gen(generator());
    uniform_int_distribution<int> randomDist(1, 3);
    uniform_real_distribution<float> randomTh(-.5,.5);

    if(localization) //TRUE = LOCAL
        myMcl->set_position(robo->robot_pose);

    this->myMap->set_empty_map(); //SET EMPTY MAP - KLD sampling

    while(true){
        mv.dist = randomDist(gen);
        mv.angle = randomTh(gen);
//        mv.dist = 5;
//        mv.angle = 0;

        img = myMcl->Gera_Imagem_Pixmap(this->robo);

        robo->move(mv);

        myMcl->sampling(mv);
        state = "\n\t\t\t  Sampling";
        usleep(50000);

        myMcl->weight_particles(robo->sense());
        state = "\n\t\t\t  Weighting";
        usleep(50000);

        if(flag_neff){ //TRUE = neff on  - FALSE = neff off

            float neff2 = myMcl->number_effective();
            if(neff2 < myMcl->num_particles/2){
                cout<<"NEFF: "<<neff2<<" - Vou fazer resample ";
//                myMcl->resample();
                myMcl->resample_Roleta();
                state = "\n\t\t\t  Resampling";
                usleep(50000);
            }
            neff = QString::number(neff2);
        }else{
            myMcl->resample_Roleta();
            state = "\n\t\t\t  Resampling";
            usleep(50000);
            neff = "OFF";
        }
        cout<<"T:"<<T++<<endl;

    }
}

void thread_mcl::run_kdl_sampling(){
    float error, zvalue;
    int T, k, M, Mx;

    T = 0;

    robo->representation();

//    myMcl.set_position(robo.robot_pose);
    myMcl->set_noise_particles(0.01, 0.01, 4.0);
    movement mv;

    random_device generator;
    mt19937 gen(generator());
    uniform_int_distribution<int> randomDist(1, 3);
    uniform_real_distribution<float> randomTh(-.5,.5);

    if(localization) //TRUE = LOCAL
        myMcl->set_position(robo->robot_pose);

    //RANDOM VALUES TO SAMPLING
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator_sampling (seed);
    std::uniform_real_distribution<double> randomValue(-.5,.5);

    while(true){
        vector<particle> new_particles;
        k = M = 0;
        Mx = numeric_limits<int>::max();
        this->myMap->set_empty_map(); //SET EMPTY MAP - KLD sampling

        mv.dist = randomDist(gen);
        mv.angle = randomTh(gen);
//        mv.dist = 5;
//        mv.angle = 0;

        img = myMcl->Gera_Imagem_Pixmap(this->robo);

        robo->move(mv);
        do{
            myMcl->particles[M] = myMcl->sampling_single(generator_sampling, randomValue,myMcl->particles[M],mv);
            state = "\n\t\t\t  Sampling";
            usleep(50000);

            myMcl->particles[M] = myMcl->weight_particles_single(myMcl->particles[M], robo->sense());
            myMcl->normalizing_particle();
            state = "\n\t\t\t  Weighting";
            usleep(50000);

            new_particles.push_back(myMcl->particles[M]);

            if(bin_is_empty(myMcl->particles[M])){
                k++;
                if(k > 1){
                  k--;
                  Mx = (int)ceil((k/2*error)*pow(1 - (2/9*k) + (sqrt(2/9*k))*zvalue,3));
                  k++;
                }
            }
            M++;
    //        myMcl->resample_Roleta();
    //        state = "\n\t\t\t  Resampling";
    //        usleep(50000);
    //        neff = "OFF";

            cout<<"T:"<<T++<<endl;
        }while(M < Mx);
        show_bin();
        myMcl->particles.clear();
        myMcl->particles = new_particles;

    }
}

bool thread_mcl::bin_is_empty(particle oneP){
    if(myMap->empty[oneP.x][oneP.y] == 1){
        cout<<"FALSE - "<<myMap->empty[oneP.x][oneP.y]<<" "<<endl;
        return false;
    }else{
        cout<<"TRUE - "<<myMap->empty[oneP.x][oneP.y]<<" "<<endl;
        myMap->empty[oneP.x][oneP.y] = 1;
        return true;
    }
}

void thread_mcl::show_bin(){
    for(int i = 0; i < myMap->empty.size(); i++){
        for(int j = 0; j < myMap->empty[i].size(); j++){
//            cout<<myMap->empty[i][j]<<" ";
        }
//        cout<<endl;
    }
}

