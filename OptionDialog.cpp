/*****************************************************************
**				Project:	ShipControl(WOPC)					**
**				Author:		Dong Shengwei						**
**				Library:	BestSea								**
**				Date:		2014-01-07							**
******************************************************************/

//OptionDialog.h

#include "OptionDialog.h"
#include "ui_OptionDialog.h"
#include <QDebug>

OptionDialog::OptionDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::OptionDialog)
{
    init();
    ui->setupUi(this);
}

OptionDialog::~OptionDialog()
{
    delete ui;
}

void OptionDialog::init()
{
    runFlag = false;
}

void OptionDialog::showData()
{
    ui->windSpeed->setText(QString::number(dataOption.windSpeed, 'f', 1));
    ui->windDir->setText(QString::number(dataOption.windDir, 'f', 1));
    ui->waveHeight->setText(QString::number(dataOption.waveHeight, 'f', 1));
    ui->waveDir->setText(QString::number(dataOption.waveDir, 'f', 1));
    ui->curSpeed->setText(QString::number(dataOption.curSpeed, 'f', 1));
    ui->curDir->setText(QString::number(dataOption.curDir, 'f', 1));

    //显示动力定位控制模式
    switch (dataOption.dpMode) {
    case NORMAL_DP:
        ui->normalDP->setChecked(true);
        break;
    case ZPCW_DP:
        ui->zpcwDP->setChecked(true);
        break;
    case WOPC_DP:
        ui->wopcDP->setChecked(true);
        break;
    case OPT_DP:
        ui->optDP->setChecked(true);
        break;
    default:
        break;
    }

    //显示动力定位控制器类型
    switch (dataOption.ctrlType) {
    case PID_CTRL:
        ui->pidController->setChecked(true);
        break;
    case NMPC_CTRL:
        ui->nmpcController->setChecked(true);
        break;
    default:
        break;
    }

    //显示PID参数
	ui->kp->setText(QString::number(dataOption.kp, 'f', 1));
	ui->ki->setText(QString::number(dataOption.ki, 'f', 1));
	ui->kd->setText(QString::number(dataOption.kd, 'f', 1));

    //显示NMPC参数
    ui->tNMPC->setText(QString::number(dataOption.tNMPC, 'f', 1));
    ui->w1NMPC->setText(QString::number(dataOption.w1NMPC, 'f', 1));
    ui->w2NMPC->setText(QString::number(dataOption.w2NMPC, 'f', 1));
    ui->w3NMPC->setText(QString::number(dataOption.w3NMPC, 'f', 1));

    //初始位置与艏向
    ui->nOrigin->setText(QString::number(dataOption.nOrigin, 'f', 1));
    ui->eOrigin->setText(QString::number(dataOption.eOrigin, 'f', 1));
    ui->psiOrigin->setText(QString::number(dataOption.psiOrigin, 'f', 1));

    //目标位置与艏向
    ui->nTarget->setText(QString::number(dataOption.nTarget, 'f', 1));
    ui->eTarget->setText(QString::number(dataOption.eTarget, 'f', 1));
    ui->psiTarget->setText(QString::number(dataOption.psiTarget, 'f', 1));

    //WOPC中虚拟圆的半径
    ui->radius->setText(QString::number(dataOption.radius, 'f', 1));

    //环境最优艏向控制的PID参数
    ui->kpWOHC->setText(QString::number(dataOption.kpWOHC, 'f', 1));
    ui->kiWOHC->setText(QString::number(dataOption.kiWOHC, 'f', 1));
    ui->kdWOHC->setText(QString::number(dataOption.kdWOHC, 'f', 1));

    //环境估计的参数
    ui->k1->setText(QString::number(dataOption.k1, 'f', 1));
    ui->k2->setText(QString::number(dataOption.k2, 'f', 1));
    ui->k3->setText(QString::number(dataOption.k3, 'f', 1));

}

void OptionDialog::apply()
{
    dataOption.windSpeed  = ui->windSpeed->text().toDouble();
    dataOption.windDir    = ui->windDir->text().toDouble();
    dataOption.waveHeight = ui->waveHeight->text().toDouble();
    dataOption.waveDir    = ui->waveDir->text().toDouble();
    dataOption.curSpeed   = ui->curSpeed->text().toDouble();
    dataOption.curDir     = ui->curDir->text().toDouble();

    if (ui->normalDP->isChecked()) {
        dataOption.dpMode = NORMAL_DP;
    } else if (ui->zpcwDP->isChecked()) {
        dataOption.dpMode = ZPCW_DP;
    } else if (ui->wopcDP->isChecked()) {
        dataOption.dpMode = WOPC_DP;
    } else if (ui->optDP->isChecked()) {
        dataOption.dpMode = OPT_DP;
    }

    if (ui->pidController->isChecked()) {
		dataOption.ctrlType = PID_CTRL;
		dataOption.kp = ui->kp->text().toDouble();
		dataOption.ki = ui->ki->text().toDouble();
		dataOption.kd = ui->kd->text().toDouble();
	} else if (ui->nmpcController->isChecked()) {
		dataOption.ctrlType = NMPC_CTRL;
        dataOption.tNMPC = ui->tNMPC->text().toDouble();
        dataOption.w1NMPC = ui->w1NMPC->text().toDouble();
        dataOption.w2NMPC = ui->w2NMPC->text().toDouble();
        dataOption.w3NMPC = ui->w3NMPC->text().toDouble();
    }

    dataOption.nOrigin    = ui->nOrigin->text().toDouble();
    dataOption.eOrigin    = ui->eOrigin->text().toDouble();
    dataOption.psiOrigin  = ui->psiOrigin->text().toDouble();

    dataOption.nTarget    = ui->nTarget->text().toDouble();
    dataOption.eTarget    = ui->eTarget->text().toDouble();
    dataOption.psiTarget  = ui->psiTarget->text().toDouble();

    dataOption.radius     = ui->radius->text().toDouble();

    dataOption.kpWOHC = ui->kpWOHC->text().toDouble();
    dataOption.kiWOHC = ui->kiWOHC->text().toDouble();
    dataOption.kdWOHC = ui->kdWOHC->text().toDouble();

    dataOption.k1 = ui->k1->text().toDouble();
    dataOption.k2 = ui->k2->text().toDouble();
    dataOption.k3 = ui->k3->text().toDouble();

    //发送数据
    emit dataChanged(dataOption);

}

//环境设置界面内可以或者不可以进行编辑
void OptionDialog::envEditEnable(bool flag)
{
    ui->windSpeed->setEnabled(flag);
    ui->windDir->setEnabled(flag);
    ui->waveHeight->setEnabled(flag);
    ui->waveDir->setEnabled(flag);
    ui->curSpeed->setEnabled(flag);
    ui->curDir->setEnabled(flag);
}

//动力定位模式设置界面内可以或者不可以进行编辑
void OptionDialog::dpModeEditEnable(bool flag)
{
    ui->normalDP->setEnabled(flag);
    ui->zpcwDP->setEnabled(flag);
    ui->wopcDP->setEnabled(flag);
    ui->optDP->setEnabled(flag);
}

//控制器设置界面内可以或者不可以进行编辑
void OptionDialog::ctrlEditEnable(bool flag)
{
    ui->pidController->setEnabled(flag);
    ui->nmpcController->setEnabled(flag);
    pidEditEnable(flag);
    nmpcEditEnable(flag);
}

//动力定位任务设置界面内可以或者不可以进行编辑
void OptionDialog::missionEditEnable(bool flag)
{
    ui->nOrigin->setEnabled(flag);
    ui->eOrigin->setEnabled(flag);
    ui->psiOrigin->setEnabled(flag);

    ui->nTarget->setEnabled(flag);
    ui->eTarget->setEnabled(flag);
    ui->psiTarget->setEnabled(flag);
}

//其他设置界面内可以或者不可以进行编辑
void OptionDialog::otherEditEnable(bool flag)
{
    radiusEditEnable(flag);
    wohcEditEnable(flag);
    envObsEditEnable(flag);
}

//PID参数可以或者不可以进行编辑
void OptionDialog::pidEditEnable(bool flag)
{
    ui->kp->setEnabled(flag);
    ui->ki->setEnabled(flag);
    ui->kd->setEnabled(flag);
}

//NMPC参数可以或者不可以进行编辑
void OptionDialog::nmpcEditEnable(bool flag)
{
    ui->tNMPC->setEnabled(flag);
    ui->w1NMPC->setEnabled(flag);
    ui->w2NMPC->setEnabled(flag);
    ui->w3NMPC->setEnabled(flag);
}

//WOPC中虚拟圆半径可以或者不可以进行编辑
void OptionDialog::radiusEditEnable(bool flag)
{
    ui->radius->setEnabled(flag);
}

//环境估计参数可以或者不可以进行编辑
void OptionDialog::envObsEditEnable(bool flag)
{
    ui->k1->setEnabled(flag);
    ui->k2->setEnabled(flag);
    ui->k3->setEnabled(flag);
}

//环境最优艏向参数可以或者不可以进行编辑
void OptionDialog::wohcEditEnable(bool flag)
{
    ui->kpWOHC->setEnabled(flag);
    ui->kiWOHC->setEnabled(flag);
    ui->kdWOHC->setEnabled(flag);
}

void OptionDialog::showEvent(QShowEvent *event)
{
    showData();

    envEditEnable(!runFlag);
    dpModeEditEnable(!runFlag);
    ctrlEditEnable(!runFlag);
    missionEditEnable(!runFlag);
    otherEditEnable(!runFlag);
}

void OptionDialog::on_applyBtn_clicked()
{
    apply();
}

void OptionDialog::on_okBtn_clicked()
{
	qDebug() << "Ok";
    apply();
    accept();
}

void OptionDialog::on_cancelBtn_clicked()
{
	qDebug() << "Cancel";
    reject();
}

void OptionDialog::on_normalDP_clicked()
{
    radiusEditEnable(false);
    wohcEditEnable(false);
}

void OptionDialog::on_zpcwDP_clicked()
{
    wohcEditEnable(true);
}

void OptionDialog::on_wopcDP_clicked()
{
    radiusEditEnable(true);
    wohcEditEnable(true);
}

void OptionDialog::on_pidController_clicked()
{
    pidEditEnable(true);
    nmpcEditEnable(false);
    envObsEditEnable(false);
}

void OptionDialog::on_nmpcController_clicked()
{
    pidEditEnable(false);
    nmpcEditEnable(true);
    envObsEditEnable(true);
}

void OptionDialog::dataInit(DataSetStruct dataSet)
{
    dataOption = dataSet;
}

void OptionDialog::shipControlRun(bool flag)
{
    runFlag = flag;
}
