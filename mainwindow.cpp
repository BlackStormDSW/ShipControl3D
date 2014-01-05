/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2014-01-04							**
******************************************************************/

//mainwindow.cpp

#include "ShipGraph.h"
#include "DataStruct.h"
#include "ShipControl.h"
#include "ShipParameter.h"
#include <iostream>
using namespace std;

#include "mainwindow.h"

#include <QtWidgets>
#include <QTextCodec>

MainWindow::MainWindow()
{
    centralWidget = new QWidget;
    setCentralWidget(centralWidget);

    shipPara = new ShipParameter;
    shipCtrl = new ShipControl;

    shipPara->readMat("E:/Program/matlab_program/s175.mat");

    Data *shipData = new Data;
    shipData = &(shipPara->getData());

    shipCtrl->setData(shipData);

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

	zoomSlider = new QSlider(Qt::Vertical);
	zoomSlider->setRange(0, 50 * 16);
	zoomSlider->setSingleStep(16);
	zoomSlider->setPageStep(10*16);
	zoomSlider->setTickInterval(10*16);
	zoomSlider->setTickPosition(QSlider::TicksRight);
	connect(zoomSlider, SIGNAL(valueChanged(int)), shipWidget, SLOT(setZoom(int)));
	connect(shipWidget, SIGNAL(zoomChanged(int)), zoomSlider, SLOT(setValue(int)));

	startButton = new QPushButton("Start");
	connect(startButton, SIGNAL(clicked()), shipCtrl, SLOT(start()));

    createActions();
    createMenus();

    QGridLayout *centralLayout = new QGridLayout;
	centralLayout->addWidget(glWidgetArea, 0, 0, 3, 3);
	centralLayout->addWidget(startButton, 0, 3, 1, 1);
	centralLayout->addWidget(zoomSlider, 1, 3, 2, 1);
    centralLayout->addWidget(xSlider, 3, 0, 1, 4);
    centralLayout->addWidget(ySlider, 4, 0, 1, 4);
    centralLayout->addWidget(zSlider, 5, 0, 1, 4);
    centralWidget->setLayout(centralLayout);

    xSlider->setValue(15 * 16);
    ySlider->setValue(345 * 16);
	zSlider->setValue(0 * 16);
	zoomSlider->setValue(10 * 16);

//    QTextCodec *codec = QTextCodec::codecForName("GB18030");
//    setWindowTitle(codec->toUnicode("船舶模拟器"));
    resize(500, 600);
}

void MainWindow::about()
{
    QMessageBox::about(this, tr("About Grabber"),
            tr("The <b>Grabber</b> example demonstrates two approaches for "
               "rendering OpenGL into a Qt pixmap."));
}

void MainWindow::controlStart()
{
    shipCtrl->start();
}

void MainWindow::createActions()
{
//    renderIntoPixmapAct = new QAction(tr("&Render into Pixmap..."), this);
//    renderIntoPixmapAct->setShortcut(tr("Ctrl+R"));
//    connect(renderIntoPixmapAct, SIGNAL(triggered()),
//            this, SLOT(renderIntoPixmap()));

    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    aboutAct = new QAction(tr("&About"), this);
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

    aboutQtAct = new QAction(tr("About &Qt"), this);
    connect(aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
}

void MainWindow::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
//    fileMenu->addAction(renderIntoPixmapAct);
//    fileMenu->addAction(grabFrameBufferAct);
//    fileMenu->addAction(clearPixmapAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
    helpMenu->addAction(aboutQtAct);
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

QSize MainWindow::getSize()
{
    bool ok;
    QString text = QInputDialog::getText(this, tr("Grabber"),
                                         tr("Enter pixmap size:"),
                                         QLineEdit::Normal,
                                         tr("%1 x %2").arg(shipWidget->width())
                                                      .arg(shipWidget->height()),
                                         &ok);
    if (!ok)
        return QSize();

    QRegExp regExp(tr("([0-9]+) *x *([0-9]+)"));
    if (regExp.exactMatch(text)) {
        int width = regExp.cap(1).toInt();
        int height = regExp.cap(2).toInt();
        if (width > 0 && width < 2048 && height > 0 && height < 2048)
            return QSize(width, height);
    }

    return shipWidget->size();
}
