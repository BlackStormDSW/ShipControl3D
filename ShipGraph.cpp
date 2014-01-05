/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2014-01-04							**
******************************************************************/

//ShipGraph.cpp

#include "ShipGraph.h"

#include <QMouseEvent>
#include <QTimer>
#include <math.h>
#include <QDebug>

ShipGraph::ShipGraph(QWidget *parent)
    : QGLWidget(parent)
{
    ship = 0;
    xRot = 0;
    yRot = 0;
    zRot = 0;

	xPos = 0.0;
	yPos = 0.0;
	zPos = 0.0;
	phi = 0.0;
	theta = 0.0;
	psi = 0.0;

	zoomScale = 1.0;

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(advanceGears()));
    timer->start(20);
}

ShipGraph::~ShipGraph()
{
    makeCurrent();
    glDeleteLists(ship, 1);
    glDeleteLists(sea, 1);
    glDeleteLists(goal, 1);
}

void ShipGraph::setXRotation(int angle)
{
    normalizeXYAngle(&angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

void ShipGraph::setYRotation(int angle)
{
    normalizeXYAngle(&angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
        updateGL();
    }
}

void ShipGraph::setZRotation(int angle)
{
    normalizeAngle(&angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}

void ShipGraph::setZoom(int scale)
{
	if (fabs(static_cast<double>(scale)/160.0 - zoomScale) > 0.00001) {
		zoomScale = static_cast<double>(scale)/160.0;
		emit zoomChanged(scale);
		updateGL();
	}
}

void ShipGraph::shipEta(Eta eta)
{
	xPos = eta.n/10;
	yPos = eta.e/10.0;
	zPos = eta.d/10.0;
	phi = eta.phi;
	theta = eta.theta;
	psi = eta.psi;

	qDebug() << xPos << "\t" << yPos << "\t" << zPos << "\t" << phi << "\t" << theta << "\t" << psi << endl;

	updateGL();
}

void ShipGraph::initializeGL()
{
    static const GLfloat lightPos[4] = { 5.0f, 5.0f, 10.0f, 1.0f };
    static const GLfloat reflectanceShip[4] = { 0.195f, 0.195f, 0.195f, 1.0f };
    static const GLfloat reflectanceSea[4] = { 0.0f, 0.39f, 1.0f, 0.1f };
    static const GLfloat reflectanceGoal[4] = { 0.8f, 0.1f, 0.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
//    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_DEPTH);

    ship = makeShip(reflectanceShip, 2.0, 5.0, 1.0);

    sea = makeSea(reflectanceSea);

    goal = makeGoal(reflectanceGoal, 5.0, 5.0, 0.1, 1.0);

    glEnable(GL_NORMALIZE);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void ShipGraph::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    glRotated(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0, 0.0, 0.0, 1.0);

	glScaled(zoomScale, zoomScale, zoomScale);

    drawShip(ship, xPos, yPos, zPos, phi * radToAng / 16.0, theta * radToAng / 16.0, psi * radToAng / 16.0);
  /*  drawShip(sea, 0.0, 0.0, 0.0, 0.0 / 16.0);
    drawShip(goal, 0.0, 0.0, 0.0, 0.0 / 16.0);*/
	glCallList(sea);
	glCallList(goal);

    glPopMatrix();
}

void ShipGraph::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-1.0, +1.0, -1.0, 1.0, 5.0, 60.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -40.0);
}

void ShipGraph::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void ShipGraph::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot + 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot + 8 * dy);
        setZRotation(zRot + 8 * dx);
    }
    lastPos = event->pos();
}

void ShipGraph::advanceGears()
{
    phi += 2 * 16 * 0;
    updateGL();
}

GLuint ShipGraph::makeShip(const GLfloat *reflectance, GLdouble width, GLdouble length, GLdouble scale)
{
    const double Pi = 3.14159265358979323846;

    GLuint list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);

    glShadeModel(GL_SMOOTH);

    GLdouble rH = width / 2.0;
    GLdouble rL = rH * 2.0;
    GLdouble ll = length - rL;
    GLdouble deltaPhi = 0.01;
    GLdouble delta = 0.01;

    GLdouble x = 0.0, y = 0.0, z = 0.0;


    //绘制船头主体部分
    glNormal3d(0.0, 0.0, -1.0);
    glBegin(GL_POLYGON);
    for (GLdouble temp = 0.0; temp < rL+delta; temp += delta)
    {
        for (GLdouble angle = 0.0; angle < Pi; angle += deltaPhi)
        {
            x = -sqrt(1.0-pow(y/rL,2.0)) * rH * cos(angle);
            y = temp;
            z = -sqrt(1.0-pow(y/rL,2.0)) * rH * sin(angle) + rH/5.0;
            glVertex3d(x, y, z);
        }
    }
    glEnd();

    //绘制船舶驾驶室部分
    glBegin(GL_POLYGON);
    for (GLdouble z =  rH/5.0; z < rH + delta; z += delta)
    {
        for (GLdouble y = -(0.0 - z*0.1); y < rH - z*0.3; y += delta)
        {
            for (GLdouble x = -(rH/1.5 - z*0.3 - y/5.0); x < rH/1.5 - z*0.3 - y/5.0; x += delta)
            {
                glNormal3d(x, y, z);
                glVertex3d(x, y, z);
            }
        }
    }
    glEnd();


    //绘制船体部分
    glNormal3d(0.0, 0.0, -1.0);
    glBegin(GL_QUADS);
        for (GLdouble angle = -deltaPhi*0.1; angle <= Pi; angle += deltaPhi)
        {
            x = -rH * cos(angle);
            y = 0.0;
            z = -rH * sin(angle) + rH/5.0;
            glVertex3d(x, y, z);

            x = -rH * cos(angle);
            y = -ll;
            z = -rH * sin(angle) + rH/5.0;
            glVertex3d(x, y, z);

            x = -rH * cos(angle+deltaPhi);
            y = -ll;
            z = -rH * sin(angle+deltaPhi) + rH/5.0;
            glVertex3d(x, y, z);

            x = -rH * cos(angle+deltaPhi);
            y = 0.0;
            z = -rH * sin(angle+deltaPhi) + rH/5.0;
            glVertex3d(x, y, z);
        }

        //画船舶甲板
        glNormal3d(0.0, 0.0, 1.0);
        z = rH/5.0;
        for (double temp = -rH; temp < rH; temp += delta)
        {
            x = temp;
            y = sqrt(1.0-pow(x/rH,2.0)) * rL;
            glVertex3d(x, y, z);

            x = temp;
            y = -ll;
            glVertex3d(x, y, z);

            x = temp + delta;
            y = -ll;
            glVertex3d(x, y, z);

            x = temp + delta;
            y = sqrt(1.0-pow(x/rH,2.0)) * rL;
            glVertex3d(x, y, z);
        }
        glEnd();

        //画船舶尾部
        glNormal3d(0.0, 0.0, 1.0);
        glBegin(GL_TRIANGLES);
        y = -ll;
        for (double angle = 0.0; angle < Pi; angle += deltaPhi)
        {
            x = 0.0;
            z = 0.0 + rH/5.0;
            glVertex3d(x, y, z);

            x = rH * cos(angle);
            z = -rH * sin(angle) + rH/5.0;
            glVertex3d(x, y, z);

            x = rH * cos(angle+deltaPhi);
            z = -rH * sin(angle+deltaPhi) + rH/5.0;
            glVertex3d(x, y, z);
        }
    glEnd();
    glEndList();

    return list;
}

GLuint ShipGraph::makeSea(const GLfloat *reflectance)
{
    GLuint list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);

    glShadeModel(GL_SMOOTH);

    GLdouble xMax = 10000.0;
    GLdouble yMax = 10000.0;
    GLdouble zMax = 10000.0;
    GLdouble x = 0.0, y = 0.0, z = 0.0;

    //绘制海洋
    glBegin(GL_QUADS);

    //上层
    glVertex3d(-xMax, -yMax, 0);
    glVertex3d(-xMax, yMax, 0);
    glVertex3d(xMax, yMax, 0);
    glVertex3d(xMax, -yMax, 0);

    //侧面1
    glVertex3d(xMax, yMax, 0);
    glVertex3d(xMax, -yMax, 0);
    glVertex3d(xMax, -yMax, -zMax);
    glVertex3d(xMax, yMax, -zMax);

    //侧面2
    glVertex3d(xMax, yMax, 0);
    glVertex3d(-xMax, yMax, 0);
        glVertex3d(-xMax, yMax, -zMax);
    glVertex3d(xMax, yMax, -zMax);

    //侧面3
    glVertex3d(-xMax, yMax, 0);
    glVertex3d(-xMax, -yMax, 0);
        glVertex3d(-xMax, -yMax, -zMax);
    glVertex3d(-xMax, yMax, -zMax);

    //侧面4
    glVertex3d(xMax, -yMax, 0);
    glVertex3d(-xMax, -yMax, 0);
        glVertex3d(-xMax, -yMax, -zMax);
    glVertex3d(xMax, -yMax, -zMax);

    //底层
    glVertex3d(-xMax, -yMax, -zMax);
    glVertex3d(-xMax, yMax, -zMax);
    glVertex3d(xMax, yMax, -zMax);
    glVertex3d(xMax, -yMax, -zMax);

    glEnd();

    glEndList();

    return list;
}

GLuint ShipGraph::makeGoal(const GLfloat *reflectance, const GLdouble xPoint,  const GLdouble yPoint,  const GLdouble radius, GLdouble scale)
{
    const double Pi = 3.14159265358979323846;

    GLuint list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance);

    glShadeModel(GL_SMOOTH);

    GLdouble deltaPhi = 0.1;
    GLdouble x = 0.0, y = 0.0, z = 0.0;

    glBegin(GL_POLYGON);
    for (GLdouble angle1 = 0.0; angle1 <= 2*Pi; angle1 += deltaPhi)
    {
        for (GLdouble angle2 = -Pi/2.0; angle2 <= Pi/2.0; angle2 += deltaPhi)
        {
            x = xPoint + radius * cos(angle2) * cos(angle1);
            y = yPoint + radius * cos(angle2) * sin(angle1);
            z = 0.0 + radius * sin(angle2);
            glNormal3d(-x,-y,-z);
            glVertex3d(x,y,z);
        }

    }
    glEnd();

    glEndList();

    return list;
}

void ShipGraph::drawShip(GLuint object, GLdouble dx, GLdouble dy, GLdouble dz,
                        GLdouble phi, GLdouble theta, GLdouble psi)
{
    glPushMatrix();
    glTranslated(dx, dy, dz);
	glRotated(phi, 1.0, 0.0, 0.0);
	glRotated(theta, 0.0, 1.0, 0.0);
	glRotated(psi, 0.0, 0.0, 1.0);
    glCallList(object);
    glPopMatrix();
}

void ShipGraph::normalizeAngle(int *angle)
{
    while (*angle < 0)
        *angle += 360 * 16;
    while (*angle > 360 * 16)
        *angle -= 360 * 16;
}

void ShipGraph::normalizeXYAngle(int *angle)
{
	while (*angle < 0)
		*angle += 360 * 16;
	while (*angle > 360 * 16)
		*angle -= 360 * 16;
	if ( *angle > 270 * 16)
	{
		*angle  = 270 * 16;
	} else if ( *angle < 90 * 16)
	{
		*angle  = 90 * 16;
	}  
	
}

