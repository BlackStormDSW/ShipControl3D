/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2014-01-07							**
******************************************************************/

//OptionDialog.h

#ifndef SETTINGDIALOG_H
#define SETTINGDIALOG_H

#include <QDialog>
#include "DataStruct.h"

namespace Ui {
class OptionDialog;
}

class OptionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OptionDialog(QWidget *parent = 0);
    ~OptionDialog();


private:
    void showEvent(QShowEvent * event);

    void init();
    void showData();
    void apply();

    //环境设置界面内可以或者不可以进行编辑
    void envEditEnable(bool flag);
    //动力定位模式设置界面内可以或者不可以进行编辑
    void dpModeEditEnable(bool flag);
    //控制器设置界面内可以或者不可以进行编辑
    void ctrlEditEnable(bool flag);
    //动力定位任务设置界面内可以或者不可以进行编辑
    void missionEditEnable(bool flag);
    //其他设置界面内可以或者不可以进行编辑
    void otherEditEnable(bool flag);
    //PID参数可以或者不可以进行编辑
    void pidEditEnable(bool flag);
    //NMPC参数可以或者不可以进行编辑
    void nmpcEditEnable(bool flag);
    //WOPC中虚拟圆半径可以或者不可以进行编辑
    void radiusEditEnable(bool flag);
    //环境估计参数可以或者不可以进行编辑
    void envObsEditEnable(bool flag);
    //环境最优艏向参数可以或者不可以进行编辑
    void wohcEditEnable(bool flag);

public slots:
    void on_applyBtn_clicked();
    void on_okBtn_clicked();
    void on_cancelBtn_clicked();

    //常规动力定位模式选择按钮按下触发相应动作
    void on_normalDP_clicked();
    //zpcw动力定位模式选择按钮按下触发相应动作
    void on_zpcwDP_clicked();
    //WOPC动力定位模式选择按钮按下触发相应动作
    void on_wopcDP_clicked();
    //环境最优(基于环境估计)动力定位模式选择按钮按下触发相应动作

    //PID控制器选择按钮按下触发相应动作
    void on_pidController_clicked();
    //NMPC控制器选择按钮按下触发相应动作
    void on_nmpcController_clicked();

    //接收数据
    void dataInit(DataSetStruct dataOp);
    //接收船舶控制是否正在执行的状态
    void shipControlRun(bool flag);

signals:
	//发送数据
    void dataChanged(DataSetStruct);

private:
    Ui::OptionDialog *ui;
    DataSetStruct dataOption;
    //船舶运行标志
    bool runFlag;
};

#endif // SETTINGDIALOG_H
