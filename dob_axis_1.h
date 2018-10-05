#ifndef DOB_AXIS_1_H
#define DOB_AXIS_1_H

#include <QMainWindow>
#include <QtMath>
#include <QTimer>
#include <QDebug>
#include "DxlControl.h"
#include "ui_dob_axis_1.h"

namespace Ui {
class DOB_axis_1;
}

class DOB_axis_1 : public QMainWindow
{
    Q_OBJECT

public:
    explicit DOB_axis_1(QWidget *parent = nullptr);
    ~DOB_axis_1();

private slots:
    void on_btnQuit_clicked();
    void on_btnStart_clicked();
    void on_btnStop_clicked();
    void on_btnReset_clicked();
    void on_cbControl_clicked();
    void on_cbCCW_clicked();
    void on_cbCW_clicked();
    void timer_out();

    void on_sbPos_valueChanged(double arg1);
    void on_sbVel_valueChanged(double arg1);
    void on_sbKgain_valueChanged(double arg1);
    void on_sbThreshP_valueChanged(double arg1);
    void on_sbThreshN_valueChanged(double arg1);

private:
    Ui::DOB_axis_1 *ui;
    DxlControl dxlControl;
    QTimer *timer;
    void init();
    void update();
    void run();
    void update_plot(QCustomPlot *plot, QVector<double> x, QVector<double> y);
    void init_plot(QCustomPlot *plot);

    double m = 0.5*2.0;
    double L = 0.2;
    double g = -9.80665;

    double t = 0.0;
    double h = 0.0;

    double q1 = 0;
    double q1_dot = 0;
    double q1_ddot = 0;

    double des_pos = 0;
    double err_pos = 0;
    double err_pos_accum = 0;
    double err_pos_prev = 0;

    double Kp_pos = 0;
    double Kd_pos = 0;
    double Ki_pos = 0;

    double des_vel = 0;
    double err_vel = 0;
    double err_vel_accum = 0;
    double err_vel_prev = 0;

    double Kp_vel = 0;
    double Kd_vel = 0;
    double Ki_vel = 0;

    double Tc_pos = 0, Tc_vel = 0, Tc = 0, Tg = 0;

    int intcount = 1;

    double r_hat = 0;
    double K = 0;
    double threshN, threshP;
    double y, yp, y_old, yp_old;
    bool collision;

    bool control = false;
    bool CCW = true;
    bool stop = false;

    QVector<double> x_pos, y_pos, x_r, y_r, x_torque, y_torque, x_current, y_current;
};

#endif // DOB_AXIS_1_H
