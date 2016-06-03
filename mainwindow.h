#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>
#include <QGraphicsScene>
#include <thread_mcl.h>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int world_size, QWidget *parent = 0);
    ~MainWindow();

public slots:
    void updateTime();

private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    QTimer Timer;
    thread_mcl *tMcl;
    Mcl *myMcl;
    Robot *robo;
    QPixmap image;


};

#endif // MAINWINDOW_H
