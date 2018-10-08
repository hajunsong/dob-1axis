#include "dob_axis_1.h"

DOB_axis_1::DOB_axis_1(QWidget *parent) : QMainWindow(parent), ui(new Ui::DOB_axis_1)
{
    ui->setupUi(this);
    this->setFixedSize(1400, 820);

    timer = new QTimer(this);
    timer->setInterval(1);
    connect(timer, SIGNAL(timeout()), this, SLOT(timer_out()));

    ui->sbPos->setDecimals(3);
    ui->sbVel->setDecimals(3);
    ui->sbAcc->setDecimals(3);
    ui->sbTorque->setDecimals(3);

    ui->sbPos->setSingleStep(0.1);
    ui->sbVel->setSingleStep(0.1);
    ui->sbAcc->setSingleStep(0.1);
    ui->sbTorque->setSingleStep(0.01);

    ui->sbVel->setRange(-999, 999);

    ui->sbKgain->setRange(0, 9999);
    ui->sbThreshN->setRange(-999, 0);
    ui->sbThreshP->setRange(0, 999);

    ui->sbKgain->setDecimals(1);
    ui->sbKgain->setSingleStep(0.1);

    ui->plotPos->xAxis->setLabel("time [s]");
    ui->plotPos->yAxis->setLabel("position [deg]");

    ui->plotTorque->xAxis->setLabel("time [s]");
    ui->plotTorque->yAxis->setLabel("Torque [Nm]");

    ui->plotCurrent->xAxis->setLabel("time [s]");
    ui->plotCurrent->yAxis->setLabel("Current [A]");

    ui->plotRhat->xAxis->setLabel("time [s]");
    ui->plotRhat->yAxis->setLabel("r [Nm]");

    dxlControl.dxl_init();

    init();
}

void DOB_axis_1::init()
{
    dxlControl.setOperateMode(position_mode);
    dxlControl.setHomePosition();
    dxlControl.setOperateMode(velocity_mode);

    des_pos = 0;
    des_vel = 0;

    Kp_pos = 0;
    Ki_pos = 0;
    Kd_pos = 0;
    err_pos = 0;
    err_pos_accum = 0;
    err_pos_prev = 0;

    Kp_vel = 0;
    Ki_vel = 0;
    Kd_vel = 0;
    err_vel = 0;
    err_vel_accum = 0;
    err_vel_prev = 0;

    K = 1000;
    threshN = -5.3;
    threshP = 4.5;

    control = false;
    CCW = true;
    stop = false;

    init_plot(ui->plotPos);
    init_plot(ui->plotTorque);
    init_plot(ui->plotCurrent);
    init_plot(ui->plotRhat);

    x_pos.clear();
    y_pos.clear();
    x_r.clear();
    y_r.clear();
    x_torque.clear();
    y_torque.clear();
    x_current.clear();
    y_current.clear();

    intcount = 1;

    y = 0;
    yp = 0;
    y_old = 0;
    yp_old = 0;

    collision = false;

    update();
}

void DOB_axis_1::update()
{
    ui->cbControl->setChecked(control);
    ui->cbCCW->setChecked(CCW);
    ui->cbCW->setChecked(!CCW);

    update_plot(ui->plotPos, x_pos, y_pos);
    update_plot(ui->plotTorque, x_torque, y_torque);
    update_plot(ui->plotCurrent, x_current, y_current);
    update_plot(ui->plotRhat, x_r, y_r);

    ui->sbKgain->setValue(K);
    ui->sbThreshN->setValue(threshN);
    ui->sbThreshP->setValue(threshP);
}

void DOB_axis_1::run()
{
    dxlControl.setLEDon(Blue);	// Write LED Blue

    uint16_t start_time = dxlControl.getRealtimeTick();

    int16_t input_torque = 0;
    if (!stop) {
        dxlControl.setVelocity(static_cast<int32_t>(des_vel / (0.01*6.0 * M_PI) * 180));
    }
    else {
        Tg = -m*g*L*qSin(q1);
        input_torque = static_cast<int16_t>((Tg / TORQUE_CONSTANT / GEAR_RATIO) * 1000);
        input_torque = input_torque > dxlControl.current_limit ? dxlControl.current_limit : input_torque < -dxlControl.current_limit ? -dxlControl.current_limit : input_torque;
        dxlControl.setInputTorque(input_torque);
    }

    double voltage = 0;
    double current = 0;

    q1 = dxlControl.getPresentPosition();		// Read Present Position [rad]
    q1_dot = dxlControl.getPresentVelocity();	// Read Present Velocity [rad/s]

    current = dxlControl.getPresentCurrent();	// Read Present Current [A]
    voltage = dxlControl.getPresentVoltage();	// Read Present Voltage [V]

    if (!stop)
        input_torque = static_cast<int16_t>(current*1000);

    double torque = 0;
    torque = current*TORQUE_CONSTANT*GEAR_RATIO;

    uint16_t end_time = dxlControl.getRealtimeTick();

    if (start_time > end_time)
        end_time += 32767;
    double h_save = static_cast<double>(end_time - start_time) * 0.001;
    h = 0.001;

    double g_q = m*g*L*qSin(q1);
    double p = 0.5*m*L*L*q1_dot*q1_dot;
    Tg = -m*g*L*qSin(q1);

    torque_cur = torque;
    yp = torque - Tg - r_hat + g_q;

    y = y_old + yp_old*h + h*0.5*(yp - yp_old);

    r_hat = K * (y - p);

    y_old = y;
    yp_old = yp;

    char message[255];
    sprintf(message, "[ID:%d] GoalTorque:%d, PresPos:%f deg, PresVel:%f deg/s, PresCur:%.3f A, InputVolt:%.1f V", DXL_ID, input_torque, q1 * 180 / M_PI, q1_dot * 180 / M_PI, current, voltage);
    ui->textBrowser->append(message);

    x_pos.push_back(t);
    x_torque.push_back(t);
    x_current.push_back(t);
    x_r.push_back(t);

    y_pos.push_back(q1);
    y_torque.push_back(torque);
    y_current.push_back(current);
    y_r.push_back(r_hat);

    if ((r_hat > threshP || r_hat < threshN) && !collision) {
        on_btnStop_clicked();
        collision = true;
    }

    t += h_save;

    dxlControl.setLEDoff();
}

void DOB_axis_1::init_plot(QCustomPlot *plot)
{
    plot->addGraph();
    plot->graph(0)->setScatterStyle(QCPScatterStyle::ssDot);
    plot->graph(0)->setLineStyle(QCPGraph::lsLine);

    plot->xAxis->setRange(-1, 1);
    plot->yAxis->setRange(-1, 1);
    plot->replot();
}

void DOB_axis_1::update_plot(QCustomPlot *plot, QVector<double> x, QVector<double> y)
{
    plot->graph(0)->setData(x, y);

    if (!x.empty() && !y.empty())
    {
        vector<double> x_temp, y_temp;
        x_temp = x.toStdVector();
        y_temp = y.toStdVector();
        sort(x_temp.begin(), x_temp.end());
        sort(y_temp.begin(), y_temp.end());

        plot->xAxis->setRange(x_temp.front(), x_temp.back());
        plot->yAxis->setRange(y_temp.front(), y_temp.back());
    }
    plot->replot();
}

DOB_axis_1::~DOB_axis_1()
{
    delete ui;
}

void DOB_axis_1::on_btnQuit_clicked()
{
    FILE *fp;
    fopen_s(&fp, "../dob_data.txt","w+");
    uint8_t length = static_cast<uint8_t>(x_pos.size());
    for(uint8_t i = 0; i < length; i++){
        fprintf(fp, "%.5f\t%10.10f\t%10.10f\t%10.10f\t%10.10f\n", x_pos[i], y_pos[i], y_torque[i], y_current[i], y_r[i]);
    }
    fclose(fp);

    this->close();
}

void DOB_axis_1::timer_out()
{
    ui->btnStart->setDisabled(true);
    timer->start();
    update();
    run();
}

void DOB_axis_1::on_btnStart_clicked()
{
    timer->start();
    ui->btnStart->setDisabled(true);
    update();
    run();
    stop = false;
}

void DOB_axis_1::on_btnStop_clicked()
{
    stop ^= true;
    collision = false;
    if (stop){
        ui->btnStop->setText("Resum");
        dxlControl.setOperateMode(torque_mode);
    }
    else {
        ui->btnStop->setText("Pause");
        dxlControl.setOperateMode(velocity_mode);
    }
}

void DOB_axis_1::on_btnReset_clicked()
{
    timer->stop();
    ui->btnStart->setDisabled(false);
    init();
}

void DOB_axis_1::on_cbControl_clicked()
{
    ui->stSpinBox->setCurrentIndex(ui->stSpinBox->currentIndex()^1);
    control ^= true;
}

void DOB_axis_1::on_cbCCW_clicked()
{
    CCW = ui->cbCCW->isChecked();
//    CCW ^= true;
    if (CCW && des_vel > 0)
        des_vel *= -1;
}

void DOB_axis_1::on_cbCW_clicked()
{
    if (ui->cbCW->isChecked()){
        CCW = false;
    }
//    CCW ^= true;
    if (!CCW && des_vel < 0)
        des_vel *= -1;
}

void DOB_axis_1::on_sbPos_valueChanged(double pos)
{
    des_pos = pos;
}

void DOB_axis_1::on_sbVel_valueChanged(double vel)
{
    des_vel = vel;
}

void DOB_axis_1::on_sbKgain_valueChanged(double value)
{
    K = value;
}

void DOB_axis_1::on_sbThreshP_valueChanged(double value)
{
    threshP = value;
}

void DOB_axis_1::on_sbThreshN_valueChanged(double value)
{
    threshN = value;
}
