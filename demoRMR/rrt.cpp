#include "rrt.h"

Rrt::Rrt(QObject *parent) : QThread(parent)
{
    tree_solved = false;
}

void Rrt::run()
{
    /**
     * Function to obtain tree and path in tree
     */

    srand (time(NULL));

    xi xi_start;
    unsigned int id = 0;
    double diff;

    xi_start.x = startpoint.x();
    xi_start.y = startpoint.y();
    dist = (sqrt(pow(goalpoint.x() - startpoint.x(), 2) + pow(goalpoint.y() - startpoint.y(), 2)));
    xi_start.parentID = 0;
    xi_start.id = 1;
    tree.push_back(xi_start);
    m.insert({id, xi_start});

    clock_t start = clock();

    // Tree solving part of code
    while (dist > 20) {
        if(solve()) {
            dist = (sqrt(pow(goalpoint.x() - xi_k1.x, 2) + pow(goalpoint.y() - xi_k1.y, 2)));
            ++id;
            xi_k1.id = id;
            xi_k1.parentID = xi_k.id;
            xi_k.childrenID.push_back(id);
            m.insert({id, xi_k1});
            tree.push_back(xi_k1);
            emit treeChanged(xi_rand, xi_k, xi_k1);
//            this->msleep(15);
        }
        diff = ( clock() - start ) / (double)CLOCKS_PER_SEC;
    }

    cout << "tree solved" << endl;
    this->tree_solved = true;

    // Path solving part of code
    if (tree_solved) {
        path.push_back(xi_k1);
        emit pathChanged(xi_k1);

        while(xi_path.parentID != 1) {
            xi_path = m[xi_k1.parentID];
            path.push_back(xi_path);
            xi_k1 = xi_path;
            if (xi_path.id == 0) {
                break;
            }

            emit pathChanged(xi_path);
//            this->msleep(50);
        }

        xi_path = m[xi_k1.parentID];
        path.push_back(xi_path);
        emit pathChanged(xi_path);
//        this->msleep(50);

        cout << "path found" << endl;
        cout << "time: " << diff << endl;
    }
}

bool Rrt::solve()
{
    /**
     * Function to get suitable node to add to existing tree
     */

    double timeUnit = 1;
    bool collision = true;

    // check if random point is in free space
    while (collision) {
        xi_rand.x = rand() % space.width;
        xi_rand.y = rand() % space.height;
        collision = false;
        for (auto obstacle : this->obstacles) {
            if (obstacle.contains(xi_rand.x, xi_rand.y)) {
                collision = true;
            }
        }
    }

    // find nearest existing node
    for (std::vector<xi, std::allocator<xi> >::value_type i_xi : tree) {
        if ((sqrt(pow(xi_rand.x - i_xi.x, 2) + pow(xi_rand.y - i_xi.y, 2)))
                < (sqrt(pow(xi_rand.x - xi_k.x, 2) + pow(xi_rand.y - xi_k.y, 2)))) {
            xi_k.x = i_xi.x;
            xi_k.y = i_xi.y;
            xi_k.id = i_xi.id;
        }
    }

    // Dynamic constraints of robot, maximum angle and distance to follow during time unit
    double angle = atan2((xi_rand.y - xi_k.y), (xi_rand.x - xi_k.x));

    if (angle > M_PI) {
        angle = M_PI;
    }

    if (angle < -M_PI) {
        angle = -M_PI;
    }

    double distance = (timeUnit*250 - abs(angle*180/M_PI)*0.2) / 10; // distance is divided by 10 to convert from mm to px

    xi_k1.x = xi_k.x + distance*cos(angle);
    xi_k1.y = xi_k.y + distance*sin(angle);

    // check if node is in free space
    for (std::vector<QRect, std::allocator<QRect>>::value_type obstacle : obstacles) {
        if (obstacle.contains(xi_k1.x, xi_k1.y)) {
            return false;
        }
    }

    return checkArcCollision(obstacles, xi_k, xi_k1);
}

bool Rrt::checkArcCollision(vector<QRect> obstacles, xi xi_k, xi xi_k1)
{
    /**
     * Function to obtain arc and check if is in collision space
     */

    point linepoint;
    std::vector<point> line;
    float x0, x1, y0, y1;
    float deltaerr;

    if (((xi_k.y < xi_k1.y) && (xi_k.x < xi_k1.x)) || ((xi_k.y > xi_k1.y) && (xi_k.x < xi_k1.x))) {
        x0 = xi_k.x;
        y0 = xi_k.y;
        x1 = xi_k1.x;
        y1 = xi_k1.y;
    } else {
        x0 = xi_k1.x;
        y0 = xi_k1.y;
        x1 = xi_k.x;
        y1 = xi_k.y;
    }

    deltaerr = (y1 - y0)/(x1 - x0);

    for (int x = x0; x < x1; ++x) {
        int y = deltaerr*(x-x0)+y0;
        linepoint.x = x;
        linepoint.y = y;
        line.push_back(linepoint);
    }

    if (!line.empty()) {
        for (std::vector<QRect, std::allocator<QRect>>::value_type obstacle : obstacles) {
            for (unsigned long long j = 0; j < line.size(); ++j) {
                if (obstacle.contains(line[j].x, line[j].y)) {
                    return false;
                }
            }
        }
    }

    std::vector<point>().swap(line);
    return true;
}
