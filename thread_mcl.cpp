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

    int T = 10;

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
//        myMcl->resample();
        float neff2 = myMcl->number_effective();
        if(neff2 < myMcl->num_particles*0.7){
            cout<<"NEFF: "<<neff2<<" - Vou fazer resample ";
            myMcl->resample_Roleta();
            state = "\n\t\t\t  Resampling";
            usleep(50000);
        }
        neff = QString::number(neff2);
        cout<<"T:"<<T++<<endl;

    }

}


