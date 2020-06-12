#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMutex>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <map_loader.h>
#include <rrt.h>
#include <QMouseEvent>

#include "CKobuki.h"
#include "rplidar.h"

typedef struct
{

    unsigned short e;
    unsigned short P;
    unsigned short I;
    unsigned short D;

} PID_v;

typedef struct
{

    unsigned short e;
    unsigned short P;
    unsigned short I;
    unsigned short D;

} PID_angle;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void robotprocess();
    void laserprocess();
    void processThisLidar(LaserMeasurement &laserData);
    void processThisRobot();
    pthread_t robotthreadHandle; // handle na vlakno
    int robotthreadID;  // id vlakna

    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }

    pthread_t laserthreadHandle; // handle na vlakno
    int laserthreadID;  // id vlakna

    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }

    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other, las_si_posli;
    int las_s,  las_recv_len;
    unsigned int las_slen;

    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other, rob_si_posli;
    int rob_s, rob_recv_len;
    unsigned int rob_slen;

    Rrt *rrt;
    QMutex mutex;

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
    std::vector<int> loadMap(TMapArea);
    void drawMap(QPainter*);
    void drawRand(QPainter*);
    void drawArc(QPainter*);
    void drawPath(QPainter*);
    void mousePressEvent(QMouseEvent *e);
    bool checkGoal(int, int);

    const double Kp = 1500;
    const double Ki = 0.01;
    const double Kd = 1;
    const double vmax = 350;
    const int mechConstrain = 20;

    bool canSolve, canGo, resetPos;
    int updateLaserPicture, node, xOrigin, yOrigin;
    LaserMeasurement copyOfLaserData;
    std::string ipaddress;
    CKobuki robot;
    TKobukiData robotdata;
    xi goal;
    point localPosition, globalPosition, localGoal;
    std::vector<int> mapDimensions;
    vector<QRect> obstacles;
    TMapArea area;
    QPoint startpoint, goalpoint;
    vector<xi> v_xi_rand, v_xi_k, v_xi_k1, path;
    spaceStruct space;
    ofstream logfile;

    double encLeft, encRight;
    double deltaLeft, deltaRight;
    double encoderLeftPrevious, encoderRightPrevious;
    double vLeft, vRight;
    double distanceLeftWheel, distanceRightWheel;
    double displacement;
    double orientation, deltaOrientation;
    double distToGoal, distOffset;
    double thetaGoal;
    double velocity, angle, polomer, distToGoalPrevious, omega, lastAngle;
    double integral, derivative;

public slots:
    void onTreeChanged(xi, xi, xi);
    void onPathChanged(xi);
    void isReady();
    void setUiValues(double robotX,double robotY,double robotFi);

signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
};

#endif // MAINWINDOW_H
