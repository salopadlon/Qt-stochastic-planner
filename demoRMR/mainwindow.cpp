#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    canGo = false;
    resetPos = true;
    globalPosition.x = 100;
    globalPosition.y = 100;
    ipaddress = "127.0.0.1";
    ui->setupUi(this);
    startpoint = QPoint(100, 100);
    goalpoint = QPoint(3,3);
    space.width = ui->frame_2->width();
    space.height = ui->frame_2->height();

    cout << "space: " << space.width << " " << space.height << endl;

    // Load configuration space from file
    char * filename = (char*) "../demoRMR/priestor.txt"; // change file path
    map_loader().load_map(filename,area);
    loadMap(area);

    qRegisterMetaType<xi>();
    canSolve = true;
}

MainWindow::~MainWindow()
{
    logfile.close();
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    /**
     * Function for drawing path planner and LIDAR data
     */

    // Planner drawing
    QPainter painter2(this);
    QPainter* p_painter = &painter2;
    QPen penGoal(Qt::red);
    QPen penStart(Qt::green);
    painter2.translate(rect().bottomLeft());
    painter2.scale(1.0, -1.0);

    xOrigin = ui->frame_2->geometry().x();
    yOrigin = size().height()-ui->frame_2->geometry().y()-ui->frame_2->geometry().height();

    drawMap(p_painter);

    penGoal.setWidth(6);
    painter2.setPen(penGoal);
    painter2.drawPoint(goalpoint.x()+xOrigin, goalpoint.y()+yOrigin);

    penStart.setWidth(6);
    painter2.setPen(penStart);
    painter2.drawPoint(startpoint.x()+xOrigin, startpoint.y()+yOrigin);

    drawMap(p_painter);
    drawArc(p_painter);
    drawPath(p_painter);

    /* uncomment function below for drawing generated random points */
//    drawRand(p_painter);

    // LIDAR data painting
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect;
    rect = ui->frame->geometry();
    painter.drawRect(rect);

    if (updateLaserPicture==1)
    {
        mutex.lock();
        updateLaserPicture=0;
        painter.setPen(pero);

        for (int k=0;k<copyOfLaserData.numberOfScans;k++)
        {
            int dist=copyOfLaserData.Data[k].scanDistance/15;
            int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
            int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();
            if (rect.contains(xp,yp))
                painter.drawEllipse(QPoint(xp, yp),2,2);
        }
        mutex.unlock();
    }
}

void MainWindow::processThisRobot()
{
    /**
     * Function for processing robot related data
     */

    std::vector<unsigned char> mess;

    if (canGo) {
        encLeft = robotdata.EncoderLeft;
        encRight = robotdata.EncoderRight;

        if (resetPos) {
            encoderLeftPrevious = encLeft;
            encoderRightPrevious = encRight;
            goal = path[path.size()-node++];
            localGoal.x = (goal.y - globalPosition.y)/100;
            localGoal.y = (globalPosition.x - goal.x)/100;
            integral = 0;
            derivative = 0;
            resetPos = false;
        }

        deltaLeft = encLeft - encoderLeftPrevious;
        deltaRight = encRight - encoderRightPrevious;

        // Odometry
        if ((deltaLeft > -100 && deltaLeft < 100) && (deltaRight > -100 && deltaRight < 100)) {
            distanceLeftWheel = deltaLeft * robot.getTickToMeter();
            distanceRightWheel = deltaRight * robot.getTickToMeter();
            displacement = (distanceLeftWheel + distanceRightWheel) / 2;

            deltaOrientation = (distanceRightWheel - distanceLeftWheel) / robot.getWheelbase();
            orientation = orientation + deltaOrientation;

            localPosition.x = localPosition.x + displacement * cos(orientation);
            localPosition.y = localPosition.y + displacement * sin(orientation);

            distToGoal = sqrt(pow((localGoal.x - localPosition.x), 2.0) + pow((localGoal.y - localPosition.y), 2.0));
            thetaGoal = atan2((localGoal.y - localPosition.y), (localGoal.x - localPosition.x));

            angle = (thetaGoal-orientation);
        }

        encoderLeftPrevious = encLeft;
        encoderRightPrevious = encRight;

        if (angle > 0.01) {
            mess = robot.setRotationSpeed(M_PI/2);
        } else if (angle < -0.01) {
            mess = robot.setRotationSpeed(-M_PI/2);
        } else {

            /* PID SPEED REGULATOR */
            integral = integral + distToGoal;
            derivative = distToGoal - distToGoalPrevious;
            velocity = (Kp * distToGoal) + (Ki * integral) + (Kd * derivative);

            if (velocity > vmax)
                velocity = vmax;

            if (distToGoal > 0.01) {
                mess = robot.setTranslationSpeed(velocity);
            } else {
                mess = robot.setTranslationSpeed(0);
                globalPosition.x = globalPosition.x - localGoal.y;
                globalPosition.y = globalPosition.y - localGoal.x;
                resetPos = true;
            }

            distToGoalPrevious = distToGoal;
        }

        logfile << "x: " << localPosition.x << " y: " << localPosition.y << "\n";
        emit uiValuesChanged(localPosition.x,localPosition.y,orientation*180/M_PI);

        if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*)&rob_si_posli,rob_slen) == -1)
        {
        }
    }
}

void MainWindow::processThisLidar(LaserMeasurement &laserData)
{
    /**
     * Function for processing LiDAR related data
     */

    mutex.lock();
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    updateLaserPicture=1;
    mutex.unlock();
    update();
}


void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

void MainWindow::on_pushButton_9_clicked() //start button
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    string relpath = "../demoRMR/log/log_";
    string fileformat = ".txt";
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    string filename = relpath + s + fileformat;

    logfile.open(filename);
    logfile << "Data logging initiated.\n\n";

    // start threds for reading data from robot and LIDAR
    laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
    robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);

    // connect signals for writing changed values into GUI
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(100);

    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_3_clicked() //back
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_6_clicked() //left
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(M_PI/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_5_clicked()//right
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(-M_PI/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::laserprocess()
{
    /**
     * Function for reading LiDAR data using UDP
     */

    // Initialize Winsock

    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    int las_broadcastene=1;
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,(char*)&las_broadcastene,sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);//toto je port z ktoreho pocuvame
    las_si_me.sin_addr.s_addr =htonl(INADDR_ANY);//moze dojst od hocikial..

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);//toto je port na ktory posielame
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;

    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, las_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(1)
    {

        if ((las_recv_len = recvfrom(las_s, (char*)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other,&las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu
        processThisLidar(measure);
    }
}

void MainWindow::robotprocess()
{
    /**
     * Function for loading map into drawable format
     */

    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    usleep(100*1000);
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
   unsigned char buff[50000];
    while(1)
    {
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }

        int returnval=robot.fillData(robotdata,(unsigned char*)buff);
        if(returnval==0)
        {
            processThisRobot();
        }


    }
}

std::vector<int> MainWindow::loadMap(TMapArea area)
{
    /**
     * Function for loading map into drawable format
     */

    int maxWidth = 0;
    int maxHeight = 0;

    for (std::vector<TMapPoint, std::allocator<TMapPoint> >::iterator it = area.wall.points.begin(); it < area.wall.points.end(); ++it) {
        if (it->point.x > maxWidth)
            maxWidth = it->point.x;

        if (it->point.y > maxHeight)
            maxHeight = it->point.y;
    }

    for (std::vector<TMapPoint, std::allocator<TMapPoint> >::iterator it = area.wall.points.begin(); it < area.wall.points.end(); ++it) {
        if (it == area.wall.points.end()-1)
            obstacles.push_back(QRect(0, 0, mechConstrain, it->point.y));

        if (it->point.x < (it+1)->point.x && it->point.y == (it+1)->point.y)
            obstacles.push_back(QRect(it->point.x, it->point.y, (it+1)->point.x-it->point.x, mechConstrain));

        if (it->point.y < (it+1)->point.y && it->point.x == (it+1)->point.x)
            obstacles.push_back(QRect(it->point.x-mechConstrain, it->point.y, mechConstrain, (it+1)->point.y-it->point.y));

        if (it->point.x > (it+1)->point.x && it->point.y == (it+1)->point.y && (it->point.x < maxWidth/2 && (it+1)->point.x < maxWidth/2))
            obstacles.push_back(QRect((it+1)->point.x+mechConstrain, it->point.y-mechConstrain, it->point.x-(it+1)->point.x-mechConstrain, mechConstrain));

        if (it->point.x > (it+1)->point.x && it->point.y == (it+1)->point.y && (it->point.x > maxWidth/2 && (it+1)->point.x < maxWidth/2))
            obstacles.push_back(QRect((it+1)->point.x, it->point.y-mechConstrain, it->point.x-(it+1)->point.x-mechConstrain, mechConstrain));

        if (it->point.y > (it+1)->point.y && it->point.x == (it+1)->point.x && it != area.wall.points.end()-1)
            obstacles.push_back(QRect((it+1)->point.x, (it+1)->point.y-mechConstrain, mechConstrain, it->point.y-(it+1)->point.y));

        if (it->point.x > (it+1)->point.x && it->point.y == (it+1)->point.y && (it->point.x > maxWidth/2 && (it+1)->point.x > maxWidth/2))
            obstacles.push_back(QRect((it+1)->point.x-mechConstrain, it->point.y-mechConstrain, it->point.x-(it+1)->point.x, mechConstrain));
    }

    for (auto obstacle : area.obstacle) {
        int xmin = size().width();
        int ymin = size().height();
        int xmax = 0;
        int ymax = 0;
        int width, height;

        for (int j = 0; j < obstacle.numofpoints; ++j) {
            if (obstacle.points[j].point.x < xmin)
                xmin = obstacle.points[j].point.x;
            if (obstacle.points[j].point.y < ymin)
                ymin = obstacle.points[j].point.y;
            if (obstacle.points[j].point.x > xmax)
                xmax = obstacle.points[j].point.x;
            if (obstacle.points[j].point.y > ymax)
                ymax = obstacle.points[j].point.y;

            width = xmax - xmin;
            height = ymax - ymin;
        }

        obstacles.push_back(QRect(xmin-mechConstrain, ymin, width+mechConstrain, height+mechConstrain));
    }

    return std::vector<int>{maxWidth, maxHeight};
}

void MainWindow::drawMap(QPainter* painter)
{
    /**
     * Function for drawing map
     */

    painter->setPen(Qt::black);
    if (!obstacles.empty()) {
        for (auto obstacle : obstacles) {
            obstacle.setX(obstacle.x()+xOrigin);
            obstacle.setY(obstacle.y()+yOrigin);
            obstacle.setWidth(obstacle.width()+xOrigin);
            obstacle.setHeight(obstacle.height()+yOrigin);
            painter->fillRect(obstacle, Qt::black);
        }
    }
}

void MainWindow::mousePressEvent(QMouseEvent *e)
{
    /**
     * Function for initialising path planning
     * Goal point is set by mouse click
     */

    if(e->buttons() == Qt::LeftButton) {
        int xOrigin = ui->frame_2->geometry().x();
        int yOrigin = size().height()-ui->frame_2->geometry().y()-ui->frame_2->geometry().height();
        int x = e->x()-xOrigin;
        int y = size().height()-e->y()-yOrigin;

        if (checkGoal(x, y) && canSolve) {
            goalpoint = QPoint(x, y);
            canSolve = false;
            rrt = new Rrt(this);
            v_xi_rand.clear();
            v_xi_k.clear();
            v_xi_k1.clear();
            path.clear();

            rrt->setGoalPoint(goalpoint);
            rrt->setStartPoint(startpoint);
            rrt->setSpace(space);
            rrt->setObstacles(obstacles);

            connect(rrt, SIGNAL(treeChanged(xi, xi, xi)), this, SLOT(onTreeChanged(xi, xi, xi)));
            connect(rrt, SIGNAL(pathChanged(xi)), this, SLOT(onPathChanged(xi)));
            connect(rrt, SIGNAL(finished()), this, SLOT(isReady()));

            rrt->start();
            update();
        }
    }
}

bool MainWindow::checkGoal(int x, int y)
{
    /**
     * Function for checking if goal point is in non-collision space
     */

    if (x <= 0 || x >= ui->frame_2->width() || y <= 0 || y >= ui->frame_2->height()) {
        return false;
    }

    for (std::vector<QRect, std::allocator<QRect>>::value_type obstacle : obstacles) {
        if (obstacle.contains(x, y)) {
            return false;
        }
    }

    return true;
}

void MainWindow::isReady()
{
    node = 1;
    canSolve = true;
    canGo = true;
}

void MainWindow::drawRand(QPainter* painter)
{
    /**
     * Function for drawing random nodes
     */

    painter->setPen(Qt::black);
    if (!v_xi_rand.empty()) {
        for (std::vector<xi, std::allocator<xi> >::value_type xi_r : v_xi_rand) {
            QPoint rand(xi_r.x+xOrigin, xi_r.y+yOrigin);
            painter->drawPoint(rand);
        }
    }
}

void MainWindow::drawArc(QPainter* painter)
{
    /**
     * Function for drawing arcs between nodes
     */

    if (!v_xi_k1.empty()) {
        for (std::vector<xi, std::allocator<xi> >::size_type i_xi_k1 = 0; i_xi_k1 < v_xi_k1.size(); ++i_xi_k1) {
            QPoint xi_k1_draw(v_xi_k1.at(i_xi_k1).x+xOrigin, v_xi_k1.at(i_xi_k1).y+yOrigin);
            QPoint xi_k_draw(v_xi_k.at(i_xi_k1).x+xOrigin, v_xi_k.at(i_xi_k1).y+yOrigin);
            painter->setPen(Qt::blue);
            painter->drawPoint(xi_k1_draw);

            painter->setPen(Qt::black);
            painter->drawLine(xi_k1_draw, xi_k_draw);
        }
    }
}

void MainWindow::drawPath(QPainter* painter)
{
    /**
     * Function for drawing path
     */

    if (!path.empty()) {
        QPoint temp_xi_path(goalpoint.x()+xOrigin, goalpoint.y()+yOrigin);
        for (std::vector<xi, std::allocator<xi> >::value_type i_path : path) {
            QPoint xi_path_draw(i_path.x+xOrigin, i_path.y+yOrigin);
            painter->setPen(Qt::yellow);
            painter->drawPoint(xi_path_draw);
            painter->drawLine(temp_xi_path, xi_path_draw);
            temp_xi_path = xi_path_draw;
        }
        QPoint startpoint_draw(startpoint.x()+xOrigin, startpoint.y()+yOrigin);
        painter->drawLine(temp_xi_path, startpoint_draw);
    }
}

void MainWindow::onTreeChanged(xi xi_rand, xi xi_k, xi xi_k1)
{
    v_xi_rand.push_back(xi_rand);
    v_xi_k.push_back(xi_k);
    v_xi_k1.push_back(xi_k1);
    update();
}

void MainWindow::onPathChanged(xi xi_path)
{
    path.push_back(xi_path);
    update();
}
