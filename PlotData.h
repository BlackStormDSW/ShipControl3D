/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2014-01-07						**
******************************************************************/

//PlotData.h

#ifndef PLOTDATA_H_
#define PLOTDATA_H_

#include <QObject>
#include <engine.h>

class PlotData : public QObject
{
	Q_OBJECT
public:
	PlotData(void);
	~PlotData(void);

	//≥ı ºªØ
	void init();

public slots:
	void drawCurve();

private:
	Engine *ep;
};

#endif//PLOTDATA_H_