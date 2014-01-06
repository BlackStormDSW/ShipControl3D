#include "SettingDialog.h"
#include "ui_SettingDialog.h"

SettingDialog::SettingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingDialog)
{
    init();
    ui->setupUi(this);
}

SettingDialog::~SettingDialog()
{
    delete ui;
}

void SettingDialog::init()
{
    runFlag = false;
}

void SettingDialog::showData()
{
    ui->windSpeed->setText(QString::number(data.windSpeed, 'f', 1));
    ui->windDir->setText(QString::number(data.windDir, 'f', 1));
    ui->waveHeight->setText(QString::number(data.waveHeight, 'f', 1));
    ui->waveDir->setText(QString::number(data.waveDir, 'f', 1));
    ui->curSpeed->setText(QString::number(data.curSpeed, 'f', 1));
    ui->curDir->setText(QString::number(data.curDir, 'f', 1));

    //显示动力定位控制模式
    switch (data.dpMode) {
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
    switch (data.ctrlType) {
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
    ui->pValue->setText(QString::number(data.pValue, 'f', 1));
    ui->iValue->setText(QString::number(data.iValue, 'f', 1));
    ui->dValue->setText(QString::number(data.dValue, 'f', 1));

    //显示NMPC参数
    ui->tValue->setText(QString::number(data.tValue, 'f', 1));
    ui->w1Value->setText(QString::number(data.w1Value, 'f', 1));
    ui->w2Value->setText(QString::number(data.w2Value, 'f', 1));
    ui->w3Value->setText(QString::number(data.w3Value, 'f', 1));

    //初始位置与艏向
    ui->nOrigin->setText(QString::number(data.nOrigin, 'f', 1));
    ui->eOrigin->setText(QString::number(data.eOrigin, 'f', 1));
    ui->psiOrigin->setText(QString::number(data.psiOrigin, 'f', 1));

    //目标位置与艏向
    ui->nTarget->setText(QString::number(data.nTarget, 'f', 1));
    ui->eTarget->setText(QString::number(data.eTarget, 'f', 1));
    ui->psiTarget->setText(QString::number(data.psiTarget, 'f', 1));

    //WOPC中虚拟圆的半径
    ui->radius->setText(QString::number(data.radius, 'f', 1));

    //环境最优艏向控制的PID参数
    ui->kp->setText(QString::number(data.kp, 'f', 1));
    ui->ki->setText(QString::number(data.ki, 'f', 1));
    ui->kd->setText(QString::number(data.kd, 'f', 1));

    //环境估计的参数
    ui->k1->setText(QString::number(data.k1, 'f', 1));
    ui->k2->setText(QString::number(data.k2, 'f', 1));
    ui->k3->setText(QString::number(data.k3, 'f', 1));

}

void SettingDialog::apply()
{
    data.windSpeed  = ui->windSpeed->text().toDouble();
    data.windDir    = ui->windDir->text().toDouble();
    data.waveHeight = ui->waveHeight->text().toDouble();
    data.waveDir    = ui->waveDir->text().toDouble();
    data.curSpeed   = ui->curSpeed->text().toDouble();
    data.curDir     = ui->curDir->text().toDouble();

    if (ui->normalDP->isChecked()) {
        data.dpMode = NORMAL_DP;
    } else if (ui->zpcwDP->isChecked()) {
        data.dpMode = ZPCW_DP;
    } else if (ui->wopcDP->isChecked()) {
        data.dpMode = WOPC_DP;
    } else if (ui->optDP->isChecked()) {
        data.dpMode = OPT_DP;
    }

    if (ui->pidController->isChecked()) {
		data.ctrlType = PID_CTRL;
        data.pValue = ui->pValue->text().toDouble();
        data.iValue = ui->iValue->text().toDouble();
        data.dValue = ui->dValue->text().toDouble();
	} else if (ui->nmpcController->isChecked()) {
		data.ctrlType = NMPC_CTRL;
        data.tValue = ui->tValue->text().toDouble();
        data.w1Value = ui->w1Value->text().toDouble();
        data.w2Value = ui->w2Value->text().toDouble();
        data.w3Value = ui->w3Value->text().toDouble();
    }

    data.nOrigin    = ui->nOrigin->text().toDouble();
    data.eOrigin    = ui->eOrigin->text().toDouble();
    data.psiOrigin  = ui->psiOrigin->text().toDouble();

    data.nTarget    = ui->nTarget->text().toDouble();
    data.eTarget    = ui->eTarget->text().toDouble();
    data.psiTarget  = ui->psiTarget->text().toDouble();

    data.radius     = ui->radius->text().toDouble();

    data.kp = ui->kp->text().toDouble();
    data.ki = ui->ki->text().toDouble();
    data.kd = ui->kd->text().toDouble();

    data.k1 = ui->k1->text().toDouble();
    data.k2 = ui->k2->text().toDouble();
    data.k3 = ui->k3->text().toDouble();

    //发送数据
    emit dataChanged(data);

}

//环境设置界面内可以或者不可以进行编辑
void SettingDialog::envEditEnable(bool flag)
{
    ui->windSpeed->setEnabled(flag);
    ui->windDir->setEnabled(flag);
    ui->waveHeight->setEnabled(flag);
    ui->waveDir->setEnabled(flag);
    ui->curSpeed->setEnabled(flag);
    ui->curDir->setEnabled(flag);
}

//动力定位模式设置界面内可以或者不可以进行编辑
void SettingDialog::dpModeEditEnable(bool flag)
{
    ui->normalDP->setEnabled(flag);
    ui->zpcwDP->setEnabled(flag);
    ui->wopcDP->setEnabled(flag);
    ui->optDP->setEnabled(flag);
}

//控制器设置界面内可以或者不可以进行编辑
void SettingDialog::ctrlEditEnable(bool flag)
{
    ui->pidController->setEnabled(flag);
    ui->nmpcController->setEnabled(flag);
    pidEditEnable(flag);
    nmpcEditEnable(flag);
}

//动力定位任务设置界面内可以或者不可以进行编辑
void SettingDialog::missionEditEnable(bool flag)
{
    ui->nOrigin->setEnabled(flag);
    ui->eOrigin->setEnabled(flag);
    ui->psiOrigin->setEnabled(flag);

    ui->nTarget->setEnabled(flag);
    ui->eTarget->setEnabled(flag);
    ui->psiTarget->setEnabled(flag);
}

//其他设置界面内可以或者不可以进行编辑
void SettingDialog::otherEditEnable(bool flag)
{
    radiusEditEnable(flag);
    wohcEditEnable(flag);
    envObsEditEnable(flag);
}

//PID参数可以或者不可以进行编辑
void SettingDialog::pidEditEnable(bool flag)
{
    ui->pValue->setEnabled(flag);
    ui->iValue->setEnabled(flag);
    ui->dValue->setEnabled(flag);
}

//NMPC参数可以或者不可以进行编辑
void SettingDialog::nmpcEditEnable(bool flag)
{
    ui->tValue->setEnabled(flag);
    ui->w1Value->setEnabled(flag);
    ui->w2Value->setEnabled(flag);
    ui->w3Value->setEnabled(flag);
}

//WOPC中虚拟圆半径可以或者不可以进行编辑
void SettingDialog::radiusEditEnable(bool flag)
{
    ui->radius->setEnabled(flag);
}

//环境估计参数可以或者不可以进行编辑
void SettingDialog::envObsEditEnable(bool flag)
{
    ui->k1->setEnabled(flag);
    ui->k2->setEnabled(flag);
    ui->k3->setEnabled(flag);
}

//环境最优艏向参数可以或者不可以进行编辑
void SettingDialog::wohcEditEnable(bool flag)
{
    ui->kp->setEnabled(flag);
    ui->ki->setEnabled(flag);
    ui->kd->setEnabled(flag);
}

void SettingDialog::showEvent(QShowEvent *event)
{
    showData();

    envEditEnable(!runFlag);
    dpModeEditEnable(!runFlag);
    ctrlEditEnable(!runFlag);
    missionEditEnable(!runFlag);
    otherEditEnable(!runFlag);
}

void SettingDialog::applyBtn_clicked()
{
    apply();
}

void SettingDialog::okBtn_clicked()
{
    apply();
    accepted();
}

void SettingDialog::cancelBtn_clicked()
{
    rejected();
}

void SettingDialog::normalDP_clicked()
{
    radiusEditEnable(false);
    wohcEditEnable(false);
}

void SettingDialog::zpcwDP_clicked()
{
    wohcEditEnable(true);
}

void SettingDialog::wopcDP_clicked()
{
    radiusEditEnable(true);
    wohcEditEnable(true);
}

void SettingDialog::pidController_clicked()
{
    pidEditEnable(true);
    nmpcEditEnable(false);
    envObsEditEnable(false);
}

void SettingDialog::nmpcController_clicked()
{
    pidEditEnable(true);
    nmpcEditEnable(false);
    envObsEditEnable(true);
}

void SettingDialog::dataInit(DataSetStruct dataSet)
{
    data = dataSet;
}

void SettingDialog::shipControlRun(bool flag)
{
    runFlag = flag;
}
