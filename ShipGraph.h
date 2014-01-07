/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2014-01-04							**
******************************************************************/

//ShipGraph.h

#ifndef SHIPGRAPH_H
#define SHIPGRAPH_H

#include <QGLWidget>
#include "DataStruct.h"

class ShipGraph : public QGLWidget
{
    Q_OBJECT

public:
    ShipGraph(QWidget *parent = 0);
    ~ShipGraph();

    int xRotation() const { return xRot; }
    int yRotation() const { return yRot; }
    int zRotation() const { return zRot; }

	//输入船舶位置
	void shipEta(Eta eta);

public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
	void setZRotation(int angle);
	void setZoom(int angle);

	void pointMove(double xMove, double yMove);

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
	void zRotationChanged(int angle);
	void zoomChanged(int scale);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);

private:
    GLuint makeShip(const GLfloat *reflectance, GLdouble width, GLdouble length, GLdouble scale);
    GLuint makeSea(const GLfloat *reflectance);
	GLuint makeGoal(const GLfloat *reflectance, const GLdouble xPoint, const GLdouble yPoint, const GLdouble radius, GLdouble scale);
	GLuint makeLine(const GLfloat *reflectance);

	void drawShip(GLuint gear, GLdouble dx, GLdouble dy, GLdouble dz, GLdouble phi, GLdouble theta, GLdouble psi);
    void normalizeAngle(int *angle);

	void normalizeXYAngle(int *angle);

    GLuint ship;
    GLuint sea;
    GLuint goal;
	GLuint line;
    int xRot;
    int yRot;
    int zRot;

	double xPoint, yPoint, zPoint;

    double xPos, yPos, zPos, phi, theta, psi;
	double zoomScale;

	int scaleWheel;

	GLdouble xMax;
	GLdouble yMax;
	GLdouble zMax;

    QPoint lastPos;
};

#endif // SHIPGRAPH
