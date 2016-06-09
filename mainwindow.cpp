#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int world_size, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    scene = new QGraphicsScene(this);
    ui->graphicsView->setFixedSize(world_size+10, world_size+10);
    ui->graphicsView->setScene(scene);

    QTimer *timer = new QTimer( this );
    timer->setInterval(.000003);

    connect(timer, SIGNAL(timeout()), this, SLOT(updateTime()));
    timer->start();
    ui->label_2->setText("");
    ui->label_4->setText("");
    ui->label_6->setText("");

}

//TO UPDATE THE IMAGE SHOWED
void MainWindow::updateTime(){

    scene->addPixmap(QPixmap::fromImage(thread_mcl::img));

    //DRAWING THE LANDMARKS
    for(int i = 0; i < Map::landmarks.size(); i++)
        scene->addEllipse(Map::landmarks[i].x-4,Map::landmarks[i].y-4,10,10,QPen(),QBrush(Qt::blue));

    //WRITING THE STATE OF THE SYSTEM
    scene->addSimpleText(thread_mcl::state);

    ui->label_2->setText(thread_mcl::neff);
    (thread_mcl::kind)? ui->label_4->setText(QString("Local")):ui->label_4->setText(QString("Global"));
    ui->label_6->setText(QString::number(thread_mcl::n_particles));

}

MainWindow::~MainWindow()
{
    delete ui;
}

//TO CLOSE THE WINDOW
void MainWindow::on_pushButton_clicked()
{
    close();
}



//    image.load("/home/mathias/Dropbox/Mestrado/build-MCL_GUI-Desktop_Qt_5_5_1_GCC_64bit-Release/teste10.png");
//    scene = new QGraphicsScene(this);

//    scene->addPixmap(image);

//    scene->setSceneRect(image.rect());
//    ui->graphicsView->setFixedSize(world_size+10, world_size+10);
//    ui->graphicsView->setScene(scene);
