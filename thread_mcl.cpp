#include "thread_mcl.h"

QImage thread_mcl::img;
QString thread_mcl::state;
QString thread_mcl::neff;
bool thread_mcl::kind;
int thread_mcl::n_particles;

thread_mcl::thread_mcl(Robot *robo, Mcl *myMcl, Map *myMap, int time, bool localization, bool kld)
{
    this->robo = robo;
    this->myMcl = myMcl;
    this->localization = localization;
    this->time = time;
    this->myMap = myMap;
    this->kind = localization;
    this->n_particles = myMcl->num_particles;
    this->kld = kld;
}

void thread_mcl::run(){
    if(kld){
        this->run_kdl_sampling();
    }else{
        this->run_normal();
    }
}

void thread_mcl::run_normal(){
    //--- SETTING VARIABLES ---
    int T = 0;
    int timeSleep = 90000;
    bool flag_neff = false;
    float noise_foward = 0.01;
    float noise_turn = 0.01;
    float noise_sense = 3.0;
    movement mv;
    random_device generator;
    mt19937 gen(generator());

    uniform_int_distribution<int> randomDist(1, 3);
    uniform_real_distribution<float> randomTh(-.5,.5);
    //-------------------------

    robo->representation();

    myMcl->set_noise_particles(noise_foward, noise_turn, noise_sense);

    if(localization) //TRUE = LOCAL
        myMcl->set_position(robo->robot_pose);

    this->myMap->set_empty_map(); //SET EMPTY MAP - KLD sampling

    while(true){
        mv.dist = randomDist(gen);
        mv.angle = randomTh(gen);

        img = myMcl->Gera_Imagem_Pixmap(this->robo);

        robo->move(mv);

        myMcl->sampling(mv);
        state = "\n\t\t\t  Sampling";
        usleep(timeSleep);

        myMcl->weight_particles(robo->sense());
        state = "\n\t\t\t  Weighting";
        usleep(timeSleep);

        if(flag_neff){ //TRUE = neff on  - FALSE = neff off

            float neff2 = myMcl->number_effective();
            if(neff2 < myMcl->num_particles/2){
                cout<<"NEFF: "<<neff2<<" - Vou fazer resample ";
//                myMcl->resample();
                myMcl->resample_Roleta();
                state = "\n\t\t\t  Resampling";
                usleep(timeSleep);
            }
            neff = QString::number(neff2);
        }else{
            myMcl->resample_Roleta();
            state = "\n\t\t\t  Resampling";
            usleep(timeSleep);
            neff = "OFF";
        }
        cout<<"T:"<<T++<<endl;
    }
}

void thread_mcl::run_kdl_sampling(){
    //--- SETTING VARIABLES ---
    int T = 0;
    int limit_min = 300;
    int k, M, Mx;
    float confidence = 0.3; // ztable is from right side of mean (Values between 0 ~ .5)
    float error = 0.4;
    float zvalue = 4.1;
    float noise_foward = 0.01;
    float noise_turn = 0.01;
    float noise_sense = 3.0;
    movement mv;
    random_device generator;
    mt19937 gen(generator());

    uniform_int_distribution<int> randomDist(1, 3);
    std::uniform_real_distribution<double> randomTh(-.5,.5);
    //-------------------------
    this->build_tablez();

    robo->representation();

    myMcl->set_noise_particles(noise_foward, noise_turn, noise_sense);

    if(localization) //TRUE = LOCAL
        myMcl->set_position(robo->robot_pose);

    confidence = fmin(0.49998,fmax(0,confidence));

    for(int i = 0; i < ztable.size(); i++){
        if(ztable[i] >= confidence){
            zvalue = i/100.00;
            cout<<"ztable["<<i<<"] = "<<ztable[i]<<"  >=  "<<confidence<<endl;
            cout<<"Z VALUE: "<<zvalue<<endl;
            break;
        }
    }

    while(true){
        particle new_particle;
        vector<particle> new_particles;
        k = M = 0;
        Mx = numeric_limits<int>::max();
        this->myMap->set_empty_map(); //SET EMPTY MAP

        mv.dist = randomDist(gen);
        mv.angle = randomTh(gen);
        robo->move(mv);

        img = myMcl->Gera_Imagem_Pixmap(this->robo);

        myMcl->normalizing_particle();
        new_particles.clear();
        do{
            new_particle = myMcl->resample_Roleta_single();  //RESAMPLING
            state = "\n\t\t\t  Resampling";

            new_particle = myMcl->sampling_single(new_particle, mv); //SAMPLING
            state = "\n\t\t\t  Sampling";

            new_particle = myMcl->weight_particles_single(new_particle, robo->sense()); //WEIGHTING
            state = "\n\t\t\t  Weighting";

            new_particles.push_back(new_particle);

            if(bin_is_empty(new_particle)){
                k++;
                if(k > 1){
                  Mx = (int)ceil(((k-1)/(2*error))*pow(1 - (2/(9.0*(k-1))) + (sqrt(2/(9.0*(k-1))))*zvalue,3));
                  cout<<"Mx VALUE: "<<Mx<<endl;
                }
            }
            M++;
            if(Mx < limit_min)
                Mx = limit_min;
            cout<<"M:"<<M<<" Mx:"<<Mx;
            cout<<" -- Scale particle:"<<new_particle.s<<endl;
        }while(M < Mx);

        n_particles = M;
        myMcl->particles.clear();
        myMcl->particles = new_particles;
        sleep(1);
    }
}

bool thread_mcl::bin_is_empty(particle oneP){
    if(myMap->empty[oneP.x][oneP.y]){ //TRUE -> IT MEANS THAT THE BIN WAS EMPTY
        myMap->empty[oneP.x][oneP.y] = false;
        return true;
    }else                            //FALSE -> IT MEANS THAT THE BIN IS NON-EMPTY
        return false;
}

void thread_mcl::build_tablez(){
    float tmp;
    ifstream ifile;
    ifile.open("/home/mathias/Dropbox/Mestrado/montecarlolocalization/ztable.data");
    if(ifile.is_open()){
        while(!ifile.eof()){
            ifile >> tmp;
            ztable.push_back(tmp);
        }
    }else{
        cout<<"ERROR - FILE ztable.data isn't open"<<endl;
        exit(-1);
    }
}
