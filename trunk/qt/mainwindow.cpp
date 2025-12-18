#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "rosworker.h"

#include <rclcpp/rclcpp.hpp>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <cmath>
#include <QTimer>
#include <QResizeEvent>
#include <QMouseEvent>
#define HEAT_ADD_POINT(hm, x, y, v)  (hm)->addPoint((x), (y), (v))


static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

static float rssiToIntensity(float rssi)
{
    // clamp [-90, -30]
    if (rssi < -90.f) rssi = -90.f;
    if (rssi > -30.f) rssi = -30.f;

    float t = (rssi + 90.f) / 60.f; // 0..1 (0 bad, 1 good)
    // QHeatMap이 “강할수록 더 뜨겁게(빨강)”면 뒤집기:
    return 1.0f - t;
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->viewport()->installEventFilter(this);

    mapItem = scene->addPixmap(QPixmap());
    mapItem->setZValue(0);

    // 로봇 마커(원), 수정 예정
    robotItem = scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
    robotItem->setZValue(10);

    const QString yamlPath = "/home/ubuntu/ros2/maps/map_simul_world.yaml";
    if (!loadStaticMap(yamlPath)) {
        qDebug() << "Failed to load map yaml:" << yamlPath;
    }
    initHeatmapLayer();

    rosThread = new RosWorker(this);
    connect(rosThread, &RosWorker::statusChanged, this, &MainWindow::onRosStatus);
    connect(rosThread, &RosWorker::robotPose, this, &MainWindow::onRobotPose);
    connect(rosThread, &RosWorker::sample, this, &MainWindow::onSample);
    connect(rosThread, &RosWorker::goalStatus,
            this, [](const QString& s){
                qDebug() << s;
            });


    rosThread->start();
}

MainWindow::~MainWindow()
{
    if (rosThread) {
        rosThread->requestInterruption();
        rosThread->wait();
    }
    delete ui;
}

void MainWindow::onRosStatus(const QString &msg)
{
    // 상태 표시용: label이 있다면 연결
    // ui->statusLabel->setText(msg);
    qDebug() << msg;
}

bool MainWindow::loadStaticMap(const QString &yamlPath)
{
    MapMeta meta;
    if (!parseMapYaml(yamlPath, meta)) return false;

    QImage img(meta.imagePath);
    if (img.isNull()) return false;

    mapItem->setPixmap(QPixmap::fromImage(img));
    scene->setSceneRect(mapItem->boundingRect());
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

    QTimer::singleShot(0, this, [this](){
        ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    });

    mapMeta_ = meta;
    mapImageSize_ = img.size();

    scene->setSceneRect(mapItem->boundingRect());
    applyViewTransform();

    return true;
}

bool MainWindow::parseMapYaml(const QString &yamlPath, MapMeta &out)
{
    QFile f(yamlPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

    const QFileInfo yinfo(yamlPath);

    while (!f.atEnd()) {
        QString line = QString::fromUtf8(f.readLine()).trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        if (line.startsWith("image:")) {
            QString v = line.mid(QString("image:").size()).trimmed();
            v.remove("\""); v.remove("'");
            out.imagePath = QFileInfo(yinfo.dir(), v).absoluteFilePath();
        }
        else if (line.startsWith("resolution:")) {
            out.resolution = line.mid(QString("resolution:").size()).trimmed().toDouble();
        }
        else if (line.startsWith("origin:")) {
            int l = line.indexOf('['), r = line.indexOf(']');
            if (l >= 0 && r > l) {
                auto parts = line.mid(l + 1, r - l - 1).split(',', Qt::SkipEmptyParts);
                if (parts.size() >= 2) {
                    out.origin_x = parts[0].trimmed().toDouble();
                    out.origin_y = parts[1].trimmed().toDouble();
                }
                if (parts.size() >= 3) {
                    out.origin_yaw = parts[2].trimmed().toDouble();
                }
            }
        }
    }

    return !out.imagePath.isEmpty() && out.resolution > 0.0;
}

QPointF MainWindow::mapMeterToPixel(double x, double y) const
{
    const double res = mapMeta_.resolution;
    const double ox  = mapMeta_.origin_x;
    const double oy  = mapMeta_.origin_y;
    const double th  = mapMeta_.origin_yaw;

    // 월드좌표를 origin 기준으로 평행이동
    const double dx = x - ox;
    const double dy = y - oy;

    // origin_yaw 회전 반영 (월드 -> 맵 픽셀축)
    // yaw가 0이면 아래는 사실상 dx,dy 그대로
    const double c = std::cos(th);
    const double s = std::sin(th);
    const double mx =  c * dx + s * dy;
    const double my = -s * dx + c * dy;

    const double px = mx / res;
    const double py = (mapImageSize_.height() - 1) - (my / res); // y 뒤집기

    return QPointF(px, py);
}

void MainWindow::onRobotPose(double x, double y, double yaw)
{
    if (mapImageSize_.isEmpty()) return;

    QPointF p = mapMeterToPixel(x, y);
    robotItem->setPos(p);
    robotItem->setRotation(-rad2deg(yaw));
    qDebug() << "AMCL pose:" << x << y << yaw;

}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);

    if (scene && ui->graphicsView) {
        ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    }
    applyViewTransform();
}

void MainWindow::applyViewTransform()
{
    // 변환 기준을 화면 중심으로
    ui->graphicsView->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
    ui->graphicsView->setResizeAnchor(QGraphicsView::AnchorViewCenter);

    // 1) 먼저 회전 적용 (왼쪽 90도 = -90)
    QTransform t;
    t.rotate(-90.0);
    ui->graphicsView->setTransform(t);

    // 2) 그 다음 현재 화면 크기에 맞게 fit (※ fit은 transform을 덮어쓸 수 있어서 순서 중요함)
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

    // 3) fit 이후 회전이 풀리는 Qt 버전도 있어서 한 번 더 강제
    ui->graphicsView->setTransform(t, false);
}

void MainWindow::initHeatmapLayer()
{
    if (mapImageSize_.isEmpty() || !scene) return;

    const int w = mapImageSize_.width();
    const int h = mapImageSize_.height();

    // 1) HeatMapper가 사용할 캔버스 (투명으로 시작)
    heatCanvas_ = QImage(w, h, QImage::Format_ARGB32);
    heatCanvas_.fill(Qt::transparent);

    // 2) 팔레트(그라데이션) 설정: 나쁨(-90)=빨강, 좋음(-30)=초록
    // GradientPalette는 0..1 index로 색을 설정하는 구조
    delete palette_;
    palette_ = new GradientPalette(256);
    palette_->setColorAt(0.0, QColor(255, 0, 0));     // red
    palette_->setColorAt(0.5, QColor(255, 255, 0));   // yellow
    palette_->setColorAt(1.0, QColor(0, 255, 0));     // green

    // 3) HeatMapper 생성 (radius/opacity는 MVP 값)
    delete heatMapper_;
    heatMapper_ = new HeatMapper(&heatCanvas_, palette_, /*radius=*/18, /*opacity=*/160);

    // 4) Scene에 overlay 아이템으로 올림
    if (!heatItem_) {
        heatItem_ = scene->addPixmap(QPixmap::fromImage(heatCanvas_));
        heatItem_->setZValue(5); // map(0) < heat(5) < robot(10)
    } else {
        heatItem_->setPixmap(QPixmap::fromImage(heatCanvas_));
    }

    // 5) 성능: 샘플마다 colorize 하지 말고 10Hz로만 갱신
    if (!heatFlushTimer_) {
        heatFlushTimer_ = new QTimer(this);
        connect(heatFlushTimer_, &QTimer::timeout, this, &MainWindow::flushHeatmap);
        heatFlushTimer_->start(100); // 10Hz
    }
}

bool MainWindow::meterToPixel(double x, double y, int& px, int& py) const
{
    if (mapImageSize_.isEmpty()) return false;

    const double res = mapMeta_.resolution;
    const double ox  = mapMeta_.origin_x;
    const double oy  = mapMeta_.origin_y;

    const int gx = static_cast<int>(std::floor((x - ox) / res));
    const int gy = static_cast<int>(std::floor((y - oy) / res));

    if (gx < 0 || gy < 0 || gx >= mapImageSize_.width() || gy >= mapImageSize_.height())
        return false;

    px = gx;
    py = (mapImageSize_.height() - 1) - gy;  // y flip
    return true;
}

int MainWindow::rssiToAddCount(float rssi) const
{
    // clamp [-90, -30]
    if (rssi < -90.f) rssi = -90.f;
    if (rssi > -30.f) rssi = -30.f;

    // t: 0(나쁨) ~ 1(좋음)
    float t = (rssi + 90.f) / 60.f;

    // 좋을수록 더 많이 찍기: 1~6회 (MVP)
    int k = 1 + static_cast<int>(t * 5.0f);
    if (k < 1) k = 1;
    if (k > 8) k = 8;
    return k;
}

void MainWindow::onSample(double x, double y, double /*yaw*/, float rssi)
{
    if (!heatMapper_) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    const int k = rssiToAddCount(rssi);
    for (int i=0; i<k; ++i) {
        heatMapper_->addPoint(px, py);
    }
}

void MainWindow::flushHeatmap()
{
    if (!heatMapper_ || !heatItem_) return;

    // HeatMapper가 data_ 기반으로 heatCanvas_를 다시 칠함
    heatMapper_->colorize();

    // scene에 반영
    heatItem_->setPixmap(QPixmap::fromImage(heatCanvas_));
}

bool MainWindow::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == ui->graphicsView->viewport() &&
        ev->type() == QEvent::MouseButtonPress) {

        auto* me = static_cast<QMouseEvent*>(ev);

        // view → scene 좌표 변환
        QPointF scenePos = ui->graphicsView->mapToScene(me->pos());

        onMapClicked(scenePos);
        return true;
    }
    return false;
}

void MainWindow::onMapClicked(const QPointF& scenePos)
{
    const int px = static_cast<int>(scenePos.x());
    const int py = static_cast<int>(scenePos.y());

    double x_m = 0.0;
    double y_m = 0.0;

    if (!pixelToMap(px, py, x_m, y_m)) {
        qDebug() << "Clicked outside map";
        return;
    }

    rosThread->requestNavigateTo(x_m, y_m, 0.0);

    qDebug() << "NavigateTo:" << x_m << y_m;
}

bool MainWindow::pixelToMap(int px, int py, double& x_m, double& y_m) const
{
    if (mapImageSize_.isEmpty()) return false;

    const int H = mapImageSize_.height();

    const int gx = px;
    const int gy = (H - 1) - py;

    x_m = mapMeta_.origin_x + gx * mapMeta_.resolution;
    y_m = mapMeta_.origin_y + gy * mapMeta_.resolution;
    return true;
}

void MainWindow::refitView()
{
    if (!mapItem) return;

    ui->graphicsView->resetTransform();

    QTransform t;
    t.rotate(-90.0);
    ui->graphicsView->setTransform(t);

    ui->graphicsView->fitInView(mapItem->boundingRect(),
                                Qt::KeepAspectRatio);

    ui->graphicsView->setTransform(t, true);
}

