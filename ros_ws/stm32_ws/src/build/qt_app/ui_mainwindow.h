/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtQuickWidgets/QQuickWidget>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <qcustomplot.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *ros_uri_layout;
    QLabel *ros_uri_lbl;
    QLabel *ros_uri;
    QCustomPlot *cubic_plot;
    QWidget *layoutWidget;
    QVBoxLayout *gpslayout;
    QGroupBox *gpsGroupBox;
    QFormLayout *formLayout;
    QLabel *latitude_lbl;
    QLabel *latitude;
    QLabel *longitude_lbl;
    QLabel *longitude;
    QLabel *speed_kmh_lbl;
    QLabel *speed_kmh;
    QLabel *northing_lbl;
    QLabel *northing;
    QLabel *easting_lbl;
    QLabel *easting;
    QLabel *gps_angle_lbl;
    QLabel *gps_angle;
    QQuickWidget *google_maps;
    QWidget *layoutWidget1;
    QHBoxLayout *kmllayout;
    QLabel *kml_lbl;
    QLineEdit *kml_path;
    QPushButton *kml_browse;
    QPushButton *pid_send;
    QWidget *layoutWidget2;
    QVBoxLayout *mpu_pid_layout;
    QGroupBox *mpuGroupBox;
    QFormLayout *formLayout_2;
    QLabel *roll_lbl;
    QLabel *roll;
    QLabel *pitch_lbl;
    QLabel *pitch;
    QLabel *yaw_lbl;
    QLabel *yaw;
    QGroupBox *pidGroupBox;
    QFormLayout *formLayout_3;
    QLabel *rpm1_lbl;
    QLabel *rpm1;
    QLabel *controller1_lbl;
    QLabel *controller1;
    QLabel *rpm2_lbl;
    QLabel *rpm2;
    QLabel *controller2_lbl;
    QLabel *controller2;
    QGroupBox *pidcontrollayout;
    QFormLayout *formLayout_4;
    QLabel *kp1_lbl;
    QLineEdit *kp1;
    QLabel *ki1_lbl;
    QLineEdit *ki1;
    QLabel *kd1_lbl;
    QLineEdit *kd1;
    QLabel *setpoint2_lbl;
    QLabel *kp2_lbl;
    QLineEdit *kp2;
    QLabel *ki2_lbl;
    QLineEdit *ki2;
    QLabel *kd2_lbl;
    QLineEdit *kd2;
    QLineEdit *setpoint1;
    QLineEdit *setpoint2;
    QLabel *setpoint1_lbl;
    QPushButton *stop_send;
    QPushButton *clear_button;

    void setupUi(QWidget *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1747, 819);
        horizontalLayoutWidget = new QWidget(MainWindow);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 10, 451, 41));
        ros_uri_layout = new QHBoxLayout(horizontalLayoutWidget);
        ros_uri_layout->setObjectName(QString::fromUtf8("ros_uri_layout"));
        ros_uri_layout->setContentsMargins(0, 0, 0, 0);
        ros_uri_lbl = new QLabel(horizontalLayoutWidget);
        ros_uri_lbl->setObjectName(QString::fromUtf8("ros_uri_lbl"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        ros_uri_lbl->setFont(font);

        ros_uri_layout->addWidget(ros_uri_lbl);

        ros_uri = new QLabel(horizontalLayoutWidget);
        ros_uri->setObjectName(QString::fromUtf8("ros_uri"));
        ros_uri->setFont(font);
        ros_uri->setAutoFillBackground(false);
        ros_uri->setStyleSheet(QString::fromUtf8("color: green"));

        ros_uri_layout->addWidget(ros_uri);

        cubic_plot = new QCustomPlot(MainWindow);
        cubic_plot->setObjectName(QString::fromUtf8("cubic_plot"));
        cubic_plot->setGeometry(QRect(1030, 260, 561, 481));
        layoutWidget = new QWidget(MainWindow);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 90, 613, 649));
        gpslayout = new QVBoxLayout(layoutWidget);
        gpslayout->setObjectName(QString::fromUtf8("gpslayout"));
        gpslayout->setContentsMargins(0, 0, 0, 0);
        gpsGroupBox = new QGroupBox(layoutWidget);
        gpsGroupBox->setObjectName(QString::fromUtf8("gpsGroupBox"));
        QFont font1;
        font1.setBold(false);
        font1.setWeight(50);
        gpsGroupBox->setFont(font1);
        gpsGroupBox->setAutoFillBackground(false);
        gpsGroupBox->setAlignment(Qt::AlignCenter);
        gpsGroupBox->setFlat(false);
        formLayout = new QFormLayout(gpsGroupBox);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        latitude_lbl = new QLabel(gpsGroupBox);
        latitude_lbl->setObjectName(QString::fromUtf8("latitude_lbl"));

        formLayout->setWidget(0, QFormLayout::LabelRole, latitude_lbl);

        latitude = new QLabel(gpsGroupBox);
        latitude->setObjectName(QString::fromUtf8("latitude"));

        formLayout->setWidget(0, QFormLayout::FieldRole, latitude);

        longitude_lbl = new QLabel(gpsGroupBox);
        longitude_lbl->setObjectName(QString::fromUtf8("longitude_lbl"));

        formLayout->setWidget(1, QFormLayout::LabelRole, longitude_lbl);

        longitude = new QLabel(gpsGroupBox);
        longitude->setObjectName(QString::fromUtf8("longitude"));

        formLayout->setWidget(1, QFormLayout::FieldRole, longitude);

        speed_kmh_lbl = new QLabel(gpsGroupBox);
        speed_kmh_lbl->setObjectName(QString::fromUtf8("speed_kmh_lbl"));

        formLayout->setWidget(2, QFormLayout::LabelRole, speed_kmh_lbl);

        speed_kmh = new QLabel(gpsGroupBox);
        speed_kmh->setObjectName(QString::fromUtf8("speed_kmh"));

        formLayout->setWidget(2, QFormLayout::FieldRole, speed_kmh);

        northing_lbl = new QLabel(gpsGroupBox);
        northing_lbl->setObjectName(QString::fromUtf8("northing_lbl"));

        formLayout->setWidget(3, QFormLayout::LabelRole, northing_lbl);

        northing = new QLabel(gpsGroupBox);
        northing->setObjectName(QString::fromUtf8("northing"));

        formLayout->setWidget(3, QFormLayout::FieldRole, northing);

        easting_lbl = new QLabel(gpsGroupBox);
        easting_lbl->setObjectName(QString::fromUtf8("easting_lbl"));

        formLayout->setWidget(4, QFormLayout::LabelRole, easting_lbl);

        easting = new QLabel(gpsGroupBox);
        easting->setObjectName(QString::fromUtf8("easting"));

        formLayout->setWidget(4, QFormLayout::FieldRole, easting);

        gps_angle_lbl = new QLabel(gpsGroupBox);
        gps_angle_lbl->setObjectName(QString::fromUtf8("gps_angle_lbl"));

        formLayout->setWidget(5, QFormLayout::LabelRole, gps_angle_lbl);

        gps_angle = new QLabel(gpsGroupBox);
        gps_angle->setObjectName(QString::fromUtf8("gps_angle"));

        formLayout->setWidget(5, QFormLayout::FieldRole, gps_angle);


        gpslayout->addWidget(gpsGroupBox);

        google_maps = new QQuickWidget(layoutWidget);
        google_maps->setObjectName(QString::fromUtf8("google_maps"));
        google_maps->setResizeMode(QQuickWidget::SizeRootObjectToView);

        gpslayout->addWidget(google_maps);

        layoutWidget1 = new QWidget(MainWindow);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(490, 10, 401, 27));
        kmllayout = new QHBoxLayout(layoutWidget1);
        kmllayout->setObjectName(QString::fromUtf8("kmllayout"));
        kmllayout->setContentsMargins(0, 0, 0, 0);
        kml_lbl = new QLabel(layoutWidget1);
        kml_lbl->setObjectName(QString::fromUtf8("kml_lbl"));

        kmllayout->addWidget(kml_lbl);

        kml_path = new QLineEdit(layoutWidget1);
        kml_path->setObjectName(QString::fromUtf8("kml_path"));
        kml_path->setEnabled(true);

        kmllayout->addWidget(kml_path);

        kml_browse = new QPushButton(layoutWidget1);
        kml_browse->setObjectName(QString::fromUtf8("kml_browse"));

        kmllayout->addWidget(kml_browse);

        pid_send = new QPushButton(MainWindow);
        pid_send->setObjectName(QString::fromUtf8("pid_send"));
        pid_send->setGeometry(QRect(910, 10, 80, 25));
        layoutWidget2 = new QWidget(MainWindow);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(651, 91, 351, 651));
        mpu_pid_layout = new QVBoxLayout(layoutWidget2);
        mpu_pid_layout->setObjectName(QString::fromUtf8("mpu_pid_layout"));
        mpu_pid_layout->setContentsMargins(0, 0, 0, 0);
        mpuGroupBox = new QGroupBox(layoutWidget2);
        mpuGroupBox->setObjectName(QString::fromUtf8("mpuGroupBox"));
        mpuGroupBox->setAlignment(Qt::AlignCenter);
        formLayout_2 = new QFormLayout(mpuGroupBox);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        roll_lbl = new QLabel(mpuGroupBox);
        roll_lbl->setObjectName(QString::fromUtf8("roll_lbl"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, roll_lbl);

        roll = new QLabel(mpuGroupBox);
        roll->setObjectName(QString::fromUtf8("roll"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, roll);

        pitch_lbl = new QLabel(mpuGroupBox);
        pitch_lbl->setObjectName(QString::fromUtf8("pitch_lbl"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, pitch_lbl);

        pitch = new QLabel(mpuGroupBox);
        pitch->setObjectName(QString::fromUtf8("pitch"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, pitch);

        yaw_lbl = new QLabel(mpuGroupBox);
        yaw_lbl->setObjectName(QString::fromUtf8("yaw_lbl"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, yaw_lbl);

        yaw = new QLabel(mpuGroupBox);
        yaw->setObjectName(QString::fromUtf8("yaw"));

        formLayout_2->setWidget(2, QFormLayout::FieldRole, yaw);


        mpu_pid_layout->addWidget(mpuGroupBox);

        pidGroupBox = new QGroupBox(layoutWidget2);
        pidGroupBox->setObjectName(QString::fromUtf8("pidGroupBox"));
        pidGroupBox->setAlignment(Qt::AlignCenter);
        formLayout_3 = new QFormLayout(pidGroupBox);
        formLayout_3->setObjectName(QString::fromUtf8("formLayout_3"));
        rpm1_lbl = new QLabel(pidGroupBox);
        rpm1_lbl->setObjectName(QString::fromUtf8("rpm1_lbl"));

        formLayout_3->setWidget(0, QFormLayout::LabelRole, rpm1_lbl);

        rpm1 = new QLabel(pidGroupBox);
        rpm1->setObjectName(QString::fromUtf8("rpm1"));

        formLayout_3->setWidget(0, QFormLayout::FieldRole, rpm1);

        controller1_lbl = new QLabel(pidGroupBox);
        controller1_lbl->setObjectName(QString::fromUtf8("controller1_lbl"));

        formLayout_3->setWidget(1, QFormLayout::LabelRole, controller1_lbl);

        controller1 = new QLabel(pidGroupBox);
        controller1->setObjectName(QString::fromUtf8("controller1"));

        formLayout_3->setWidget(1, QFormLayout::FieldRole, controller1);

        rpm2_lbl = new QLabel(pidGroupBox);
        rpm2_lbl->setObjectName(QString::fromUtf8("rpm2_lbl"));

        formLayout_3->setWidget(2, QFormLayout::LabelRole, rpm2_lbl);

        rpm2 = new QLabel(pidGroupBox);
        rpm2->setObjectName(QString::fromUtf8("rpm2"));

        formLayout_3->setWidget(2, QFormLayout::FieldRole, rpm2);

        controller2_lbl = new QLabel(pidGroupBox);
        controller2_lbl->setObjectName(QString::fromUtf8("controller2_lbl"));

        formLayout_3->setWidget(3, QFormLayout::LabelRole, controller2_lbl);

        controller2 = new QLabel(pidGroupBox);
        controller2->setObjectName(QString::fromUtf8("controller2"));

        formLayout_3->setWidget(3, QFormLayout::FieldRole, controller2);


        mpu_pid_layout->addWidget(pidGroupBox);

        pidcontrollayout = new QGroupBox(layoutWidget2);
        pidcontrollayout->setObjectName(QString::fromUtf8("pidcontrollayout"));
        pidcontrollayout->setAlignment(Qt::AlignCenter);
        formLayout_4 = new QFormLayout(pidcontrollayout);
        formLayout_4->setObjectName(QString::fromUtf8("formLayout_4"));
        kp1_lbl = new QLabel(pidcontrollayout);
        kp1_lbl->setObjectName(QString::fromUtf8("kp1_lbl"));

        formLayout_4->setWidget(1, QFormLayout::LabelRole, kp1_lbl);

        kp1 = new QLineEdit(pidcontrollayout);
        kp1->setObjectName(QString::fromUtf8("kp1"));

        formLayout_4->setWidget(1, QFormLayout::FieldRole, kp1);

        ki1_lbl = new QLabel(pidcontrollayout);
        ki1_lbl->setObjectName(QString::fromUtf8("ki1_lbl"));

        formLayout_4->setWidget(2, QFormLayout::LabelRole, ki1_lbl);

        ki1 = new QLineEdit(pidcontrollayout);
        ki1->setObjectName(QString::fromUtf8("ki1"));

        formLayout_4->setWidget(2, QFormLayout::FieldRole, ki1);

        kd1_lbl = new QLabel(pidcontrollayout);
        kd1_lbl->setObjectName(QString::fromUtf8("kd1_lbl"));

        formLayout_4->setWidget(3, QFormLayout::LabelRole, kd1_lbl);

        kd1 = new QLineEdit(pidcontrollayout);
        kd1->setObjectName(QString::fromUtf8("kd1"));

        formLayout_4->setWidget(3, QFormLayout::FieldRole, kd1);

        setpoint2_lbl = new QLabel(pidcontrollayout);
        setpoint2_lbl->setObjectName(QString::fromUtf8("setpoint2_lbl"));

        formLayout_4->setWidget(4, QFormLayout::LabelRole, setpoint2_lbl);

        kp2_lbl = new QLabel(pidcontrollayout);
        kp2_lbl->setObjectName(QString::fromUtf8("kp2_lbl"));

        formLayout_4->setWidget(5, QFormLayout::LabelRole, kp2_lbl);

        kp2 = new QLineEdit(pidcontrollayout);
        kp2->setObjectName(QString::fromUtf8("kp2"));

        formLayout_4->setWidget(5, QFormLayout::FieldRole, kp2);

        ki2_lbl = new QLabel(pidcontrollayout);
        ki2_lbl->setObjectName(QString::fromUtf8("ki2_lbl"));

        formLayout_4->setWidget(6, QFormLayout::LabelRole, ki2_lbl);

        ki2 = new QLineEdit(pidcontrollayout);
        ki2->setObjectName(QString::fromUtf8("ki2"));

        formLayout_4->setWidget(6, QFormLayout::FieldRole, ki2);

        kd2_lbl = new QLabel(pidcontrollayout);
        kd2_lbl->setObjectName(QString::fromUtf8("kd2_lbl"));

        formLayout_4->setWidget(7, QFormLayout::LabelRole, kd2_lbl);

        kd2 = new QLineEdit(pidcontrollayout);
        kd2->setObjectName(QString::fromUtf8("kd2"));

        formLayout_4->setWidget(7, QFormLayout::FieldRole, kd2);

        setpoint1 = new QLineEdit(pidcontrollayout);
        setpoint1->setObjectName(QString::fromUtf8("setpoint1"));
        setpoint1->setEnabled(true);

        formLayout_4->setWidget(0, QFormLayout::FieldRole, setpoint1);

        setpoint2 = new QLineEdit(pidcontrollayout);
        setpoint2->setObjectName(QString::fromUtf8("setpoint2"));
        setpoint2->setEnabled(true);

        formLayout_4->setWidget(4, QFormLayout::FieldRole, setpoint2);

        setpoint1_lbl = new QLabel(pidcontrollayout);
        setpoint1_lbl->setObjectName(QString::fromUtf8("setpoint1_lbl"));

        formLayout_4->setWidget(0, QFormLayout::LabelRole, setpoint1_lbl);


        mpu_pid_layout->addWidget(pidcontrollayout);

        stop_send = new QPushButton(MainWindow);
        stop_send->setObjectName(QString::fromUtf8("stop_send"));
        stop_send->setGeometry(QRect(1010, 10, 80, 25));
        clear_button = new QPushButton(MainWindow);
        clear_button->setObjectName(QString::fromUtf8("clear_button"));
        clear_button->setGeometry(QRect(910, 50, 80, 25));

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QWidget *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Form", nullptr));
        ros_uri_lbl->setText(QApplication::translate("MainWindow", "ROS_MASTER_URI", nullptr));
        ros_uri->setText(QApplication::translate("MainWindow", "TextLabel", nullptr));
        gpsGroupBox->setTitle(QApplication::translate("MainWindow", "GPS Data", nullptr));
        latitude_lbl->setText(QApplication::translate("MainWindow", "Latitude", nullptr));
        latitude->setText(QString());
        longitude_lbl->setText(QApplication::translate("MainWindow", "Longitude", nullptr));
        longitude->setText(QString());
        speed_kmh_lbl->setText(QApplication::translate("MainWindow", "Speed (km/h)", nullptr));
        speed_kmh->setText(QString());
        northing_lbl->setText(QApplication::translate("MainWindow", "Northing (m)", nullptr));
        northing->setText(QString());
        easting_lbl->setText(QApplication::translate("MainWindow", "Easting (m)", nullptr));
        easting->setText(QString());
        gps_angle_lbl->setText(QApplication::translate("MainWindow", "Heading (degree)", nullptr));
        gps_angle->setText(QString());
        kml_lbl->setText(QApplication::translate("MainWindow", "KML file:", nullptr));
        kml_browse->setText(QApplication::translate("MainWindow", "Browse", nullptr));
        pid_send->setText(QApplication::translate("MainWindow", "Send PID", nullptr));
        mpuGroupBox->setTitle(QApplication::translate("MainWindow", "MPU Data", nullptr));
        roll_lbl->setText(QApplication::translate("MainWindow", "roll", nullptr));
        roll->setText(QString());
        pitch_lbl->setText(QApplication::translate("MainWindow", "pitch", nullptr));
        pitch->setText(QString());
        yaw_lbl->setText(QApplication::translate("MainWindow", "yaw", nullptr));
        yaw->setText(QString());
        pidGroupBox->setTitle(QApplication::translate("MainWindow", "PID Data", nullptr));
        rpm1_lbl->setText(QApplication::translate("MainWindow", "RPM wheel 1", nullptr));
        rpm1->setText(QString());
        controller1_lbl->setText(QApplication::translate("MainWindow", "Controller wheel 1", nullptr));
        controller1->setText(QString());
        rpm2_lbl->setText(QApplication::translate("MainWindow", "RPM wheel 2", nullptr));
        rpm2->setText(QString());
        controller2_lbl->setText(QApplication::translate("MainWindow", "Controller wheel 2", nullptr));
        controller2->setText(QString());
        pidcontrollayout->setTitle(QApplication::translate("MainWindow", "PID Control", nullptr));
        kp1_lbl->setText(QApplication::translate("MainWindow", "Kp1", nullptr));
        ki1_lbl->setText(QApplication::translate("MainWindow", "Ki1", nullptr));
        kd1_lbl->setText(QApplication::translate("MainWindow", "Kd1", nullptr));
        setpoint2_lbl->setText(QApplication::translate("MainWindow", "Setpoint wheel 2", nullptr));
        kp2_lbl->setText(QApplication::translate("MainWindow", "Kp2", nullptr));
        ki2_lbl->setText(QApplication::translate("MainWindow", "Ki2", nullptr));
        kd2_lbl->setText(QApplication::translate("MainWindow", "Kd2", nullptr));
        setpoint1_lbl->setText(QApplication::translate("MainWindow", "Setpoint wheel 1", nullptr));
        stop_send->setText(QApplication::translate("MainWindow", "Stop", nullptr));
        clear_button->setText(QApplication::translate("MainWindow", "Clear", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
