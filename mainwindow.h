#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include <QTimer>
#include "opencvdeal.h"
#include "serialconnect.h"

namespace Ui {
class MainWindow;
}
using namespace cv;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QImage Mat2QImage(Mat image1);
public slots:
    void processFrameAndUpdateGUI();
private slots:
    void on_cancelButton_clicked();
    void on_circleButton_clicked();
    void on_stereoButton_clicked();
    void on_saveButton_clicked();
    void on_startButton_clicked();
private:
    Ui::MainWindow *ui;
    opencvdeal cvdeal;
    QTimer *tmrTimer;
    Mat image1;
    serialconnect Serialconnect;
    bool serialopened=false;
    bool recognizestart=false;
    int settemp,nowtemp;
};

#endif // MAINWINDOW_H
