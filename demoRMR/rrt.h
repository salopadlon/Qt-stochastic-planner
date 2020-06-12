#ifndef RRTTHREAD_H
#define RRTTHREAD_H
#include <QThread>
#include <vector>
#include <QtCore>
#include <iostream>
#include <cmath>

using namespace std;

typedef struct   {
    double x;
    double y;
    int id;
    int parentID;
    vector<int> childrenID;
    double angle;
} xi;

typedef struct  {
    double x;
    double y;
} point;

typedef struct  {
    int width, height;
} spaceStruct;

Q_DECLARE_METATYPE(xi)

class Rrt : public QThread
{
    Q_OBJECT

public:
    explicit Rrt(QObject *parent = 0);
    void run() override;
    bool solve();

    void setStartPoint(QPoint point) {
        this->startpoint = point;
    }

    void setGoalPoint(QPoint point) {
        this->goalpoint = point;
    }

    void setSpace(spaceStruct space) {
        this->space = space;
    }

    void setObstacles(vector<QRect> obstacles) {
        this->obstacles = obstacles;
    }

    vector<xi> getTreeStart() {
        return this->tree;
    }

signals:
    void treeChanged(xi, xi, xi);
    void pathChanged(xi);

public slots:

private:
    int dist;
    xi xi_rand, xi_k, xi_k1, xi_path;
    double x_der, y_der, theta, theta_der, v_max, omega_max, x_k, y_k, x_k1, y_k1;
    vector<double> v, omega, xi_der;
    vector<xi> tree, path;
    vector<vector<double>> mi;
    vector<QRect> obstacles;
    QPoint goalpoint, startpoint;
    spaceStruct space;
    std::map<int, xi> m;
    bool tree_solved;

    bool checkArcCollision(vector<QRect>, xi, xi);
};

#endif // RRTTHREAD_H
