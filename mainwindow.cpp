/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2014-01-04							**
******************************************************************/

//Mainwindow.cpp

#include "ShipGraph.h"
#include "ShipControl.h"
#include "ShipParameter.h"
#include "OptionDialog.h"
#include "Mainwindow.h"
#include <QtWidgets>
#include <QTimer>
#include <QTextCodec>
#include <QMetaType>
#include <QDebug>
#include <iostream>
using namespace std;

MainWindow::MainWindow()
{
	//qRegisterMetaType<DataSetStruct>("DataSetStruct");

    centralWidget = new QWidget;
    setCentralWidget(centralWidget);

	timer = new QTimer(this);

	optionDlg = new OptionDialog;

    shipPara = new ShipParameter;
    shipCtrl = new ShipControl;

    shipPara->readMat("E:/Program/matlab_program/s175.mat");

    Data *shipData = new Data;
    shipData = &(shipPara->getData());
	
    shipWidget = new ShipGraph;

    glWidgetArea = new QScrollArea;
    glWidgetArea->setWidget(shipWidget);
    glWidgetArea->setWidgetResizable(true);
    glWidgetArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    glWidgetArea->setMinimumSize(50, 50);

    xSlider = createSlider(SIGNAL(xRotationChanged(int)),
                           SLOT(setXRotation(int)));
    ySlider = createSlider(SIGNAL(yRotationChanged(int)),
                           SLOT(setYRotation(int)));
    zSlider = createSlider(SIGNAL(zRotationChanged(int)),
						   SLOT(setZRotation(int)));

	xSlider->setRange(90*16, 270*16);
	ySlider->setRange(90*16, 270*16);

	zoomSlider = new QSlider(Qt::Vertical);
	zoomSlider->setRange(1*16, 50 * 16);
	zoomSlider->setSingleStep(16);
	zoomSlider->setPageStep(10*16);
	zoomSlider->setTickInterval(10*16);
	zoomSlider->setTickPosition(QSlider::TicksRight);
	connect(zoomSlider, SIGNAL(valueChanged(int)), shipWidget, SLOT(setZoom(int)));
	connect(shipWidget, SIGNAL(zoomChanged(int)), zoomSlider, SLOT(setValue(int)));

	startButton = new QPushButton("Start");
	connect(startButton, SIGNAL(clicked()), this, SLOT(controlStart()));

	pauseButton = new QPushButton("Pause");
	connect(pauseButton, SIGNAL(clicked()), this, SLOT(controlPause()));

	stopButton = new QPushButton("Stop");
	connect(stopButton, SIGNAL(clicked()), this, SLOT(controlStop()));

	connect(timer, SIGNAL(timeout()), this, SLOT(updateShip()));

	connect(optionDlg, SIGNAL(dataChanged(DataSetStruct)), this,	  SLOT(updateTarget(DataSetStruct)));
	connect(shipCtrl,  SIGNAL(sendDataSet(DataSetStruct)), this,	  SLOT(updateTarget(DataSetStruct)));
	connect(optionDlg, SIGNAL(dataChanged(DataSetStruct)), shipCtrl,  SLOT(receivDataSet(DataSetStruct)));
	connect(shipCtrl,  SIGNAL(sendDataSet(DataSetStruct)), optionDlg, SLOT(dataInit(DataSetStruct)));
	connect(shipCtrl,  SIGNAL(runState(bool)),			   optionDlg, SLOT(shipControlRun(bool)));

    createActions();
    createMenus();

	shipCtrl->setData(shipData);
	shipCtrl->setParameter();

    QGridLayout *centralLayout = new QGridLayout;
	centralLayout->addWidget(glWidgetArea, 0, 0, 4, 4);
	centralLayout->addWidget(startButton, 0, 4, 1, 1);
	centralLayout->addWidget(pauseButton, 1, 4, 1, 1);
	centralLayout->addWidget(stopButton, 2, 4, 1, 1);
	centralLayout->addWidget(zoomSlider, 3, 4, 1, 1);
    centralLayout->addWidget(xSlider, 4, 0, 1, 5);
    centralLayout->addWidget(ySlider, 5, 0, 1, 5);
    centralLayout->addWidget(zSlider, 6, 0, 1, 5);
    centralWidget->setLayout(centralLayout);

    xSlider->setValue(150 * 16);
    ySlider->setValue(200 * 16);
	zSlider->setValue(150 * 16);
	zoomSlider->setValue(10 * 16);

//    QTextCodec *codec = QTextCodec::codecForName("GB18030");
//    setWindowTitle(codec->toUnicode("èˆ¹èˆ¶æ¨¡æ‹Ÿï¿?));
    resize(500, 600);

	pauseButton->setEnabled(false);
	stopButton->setEnabled(false);

	//³õÊ¼»¯Îª´¬²°¿ØÖÆ³õ´ÎÔËÐÐ
	firstRun = true;
}

void MainWindow::about()
{
    QMessageBox::about(this, tr("About ShipControl(3D)"),
            tr("The <b>ShipControl 3D</b> is designed by Dong Shengwei."));
}

//Í£Ö¹´¬²°¿ØÖÆ
void MainWindow::controlStart()
{
	if (firstRun)
	{
		shipCtrl->start();
		timer->start(20);
		firstRun = false;
	} else {
		shipCtrl->startRun();
	}
	
	startButton->setEnabled(false);
	pauseButton->setEnabled(true);
	stopButton->setEnabled(true);

	emit shipCtrl->runState(true);
}

//ÔÝÍ£´¬²°¿ØÖÆ
void MainWindow::controlPause()
{
	shipCtrl->pauseRun();
	startButton->setEnabled(true);
	pauseButton->setEnabled(false);
	stopButton->setEnabled(true);

	emit shipCtrl->runState(false);
}

//Í£Ö¹´¬²°¿ØÖÆ
void MainWindow::controlStop()
{
	shipCtrl->stopRun();
	startButton->setEnabled(false);
	pauseButton->setEnabled(false);
	stopButton->setEnabled(false);

	emit shipCtrl->runState(false);
}

//¸üÐÂ´¬²°Î»ÖÃ×ËÌ¬
void MainWindow::updateShip()
{
	shipWidget->shipEta(shipCtrl->getEta());
}

//¸üÐÂ´¬²°Ä¿±êÎ»ÖÃô¼Ïò
void MainWindow::updateTarget(DataSetStruct)
{
	shipWidget->targetEta(shipCtrl->getTarget());
}

void MainWindow::createActions()
{
    setDialogAct = new QAction(tr("Op&tion..."), this);
    setDialogAct->setShortcut(tr("Ctrl+T"));
    connect(setDialogAct, SIGNAL(triggered()),
            optionDlg, SLOT(show()));

    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    aboutAct = new QAction(tr("&About"), this);
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
}

void MainWindow::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(setDialogAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
}

QSlider *MainWindow::createSlider(const char *changedSignal,
                                  const char *setterSlot)
{
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    connect(slider, SIGNAL(valueChanged(int)), shipWidget, setterSlot);
    connect(shipWidget, changedSignal, slider, SLOT(setValue(int)));
    return slider;
}
