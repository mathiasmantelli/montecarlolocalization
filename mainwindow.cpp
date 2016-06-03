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
    timer->setInterval(300);

    connect(timer, SIGNAL(timeout()), this, SLOT(updateTime()));
    timer->start();

}

//TO UPDATE THE IMAGE SHOWED
void MainWindow::updateTime(){
    scene->addPixmap(QPixmap::fromImage(thread_mcl::img));
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
