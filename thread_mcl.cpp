#include "thread_mcl.h"

QImage thread_mcl::img;

thread_mcl::thread_mcl(Robot *robo, Mcl *myMcl, Map *myMap, int time, bool localization)
{
    this->robo = robo;
    this->myMcl = myMcl;
    this->localization = localization;
    this->time = time;
    this->myMap = myMap;
}

void thread_mcl::run(){

    int T = 10;

    robo->representation();

//    myMcl.set_position(robo.robot_pose);
    myMcl->set_noise_particles(0.01, 0.01, 5.0);
    movement mv;

    random_device generator;
    mt19937 gen(generator());
    uniform_int_distribution<int> randomDist(1, 5);
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

        myMcl->weight_particles(robo->sense());

//        myMcl->resample();
        myMcl->resample_Roleta();

        cout<<"T:"<<T<<endl;
        usleep(900000);
    }

}


