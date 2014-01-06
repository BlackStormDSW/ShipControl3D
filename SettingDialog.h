#ifndef SETTINGDIALOG_H
#define SETTINGDIALOG_H

#include <QDialog>
#include "DataStruct.h"

namespace Ui {
class SettingDialog;
}

class SettingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SettingDialog(QWidget *parent = 0);
    ~SettingDialog();


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
    void applyBtn_clicked();
    void okBtn_clicked();
    void cancelBtn_clicked();

    //常规动力定位模式选择按钮按下触发相应动作
    void normalDP_clicked();
    //zpcw动力定位模式选择按钮按下触发相应动作
    void zpcwDP_clicked();
    //WOPC动力定位模式选择按钮按下触发相应动作
    void wopcDP_clicked();
    //环境最优(基于环境估计)动力定位模式选择按钮按下触发相应动作

    //PID控制器选择按钮按下触发相应动作
    void pidController_clicked();
    //NMPC控制器选择按钮按下触发相应动作
    void nmpcController_clicked();

    //初始化数据
    void dataInit(DataSetStruct dataSet);
    //接收船舶控制是否正在执行的状态
    void shipControlRun(bool flag);

signals:
    void dataChanged(DataSetStruct);

private:
    Ui::SettingDialog *ui;
    DataSetStruct data;
    //船舶运行标志
    bool runFlag;
};

#endif // SETTINGDIALOG_H
