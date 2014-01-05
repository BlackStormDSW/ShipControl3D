/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2014-01-04							**
******************************************************************/

//mainwindow.h

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
class QLabel;
class QMenu;
class QScrollArea;
class QSlider;
class QPushButton;
QT_END_NAMESPACE

class ShipGraph;
class ShipControl;
class ShipParameter;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow();

private slots:
    void about();
    void controlStart();

private:
    void createActions();
    void createMenus();
    QSlider *createSlider(const char *changedSignal, const char *setterSlot);
    QSize getSize();

    ShipControl *shipCtrl;
    ShipParameter *shipPara;

    QWidget *centralWidget;
    QScrollArea *glWidgetArea;
    ShipGraph *shipWidget;

    QSlider *xSlider;
    QSlider *ySlider;
	QSlider *zSlider;
	QSlider *zoomSlider;

    QPushButton *startButton;

    QMenu *fileMenu;
    QMenu *helpMenu;
//    QAction *grabFrameBufferAct;
//    QAction *renderIntoPixmapAct;
//    QAction *clearPixmapAct;
    QAction *exitAct;
    QAction *aboutAct;
    QAction *aboutQtAct;
};

#endif // MAINWINDOW_H
