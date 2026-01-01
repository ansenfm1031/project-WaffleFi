#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QPixmap>
#include <QDebug>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QShowEvent>
#include <QDateTime>
#include <cmath>

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // ===== graphics =====
    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->viewport()->installEventFilter(this);

    mapItem = scene->addPixmap(QPixmap());
    mapItem->setZValue(0);

    robotItem = scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
    robotItem->setZValue(10);
    robotItem->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    robotItem->setVisible(false);

    // =======================
    // Load static map
    // =======================
    const QString yamlPath = "../../docs/maps/map.yaml";
    if (!loadStaticMap(yamlPath)) {
        qDebug() << "Failed to load map yaml:" << yamlPath;
    } else {
        mapReady_ = true;
    }

    // ===== heatmaps init =====
    initLiveHeatmap();
    initQueryHeatmap();

    // ===== ROS worker =====
    rosThread = new RosWorker(this);
    connect(rosThread, &RosWorker::statusChanged, this, &MainWindow::onRosStatus);
    connect(rosThread, &RosWorker::robotPose,     this, &MainWindow::onRobotPose);
    connect(rosThread, &RosWorker::fusedSample,   this, &MainWindow::onFusedSample);
    connect(rosThread, &RosWorker::heatmapReplyArrived, this, &MainWindow::onHeatmapReplyArrived);
    rosThread->start();

    // ===== UI connects =====
    if (ui->chkShowHeatmap) connect(ui->chkShowHeatmap, &QCheckBox::toggled, this, &MainWindow::onLayerHeatmap);
    if (ui->chkShowRobot)   connect(ui->chkShowRobot,   &QCheckBox::toggled, this, &MainWindow::onLayerRobot);

    if (ui->btnQueryClear) connect(ui->btnQueryClear, &QPushButton::clicked, this, &MainWindow::onQueryClear);
    if (ui->btnApplyFilter) connect(ui->btnApplyFilter, &QPushButton::clicked, this, &MainWindow::onApplyFilter);

    // 세션 목록 “refresh”는 지금 단계에서는 DB 서비스가 따로 list를 안 주므로
    // UI에서 cbSessionId가 이미 채워진다고 가정하거나, 팀에서 list 서비스를 별도로 붙이면 됩니다.
    // 여기서는 버튼이 눌리면 화면 메시지만 갱신.
    if (ui->btnSessionRefresh) connect(ui->btnSessionRefresh, &QPushButton::clicked, this, &MainWindow::onSessionRefresh);

    // 기본 필터값
    if (ui->cbSsid && ui->cbSsid->count() == 0) ui->cbSsid->addItem("ALL");

    applyLayersPolicy();
}

MainWindow::~MainWindow()
{
    if (rosThread) {
        rosThread->requestInterruption();
        rosThread->wait();
    }

    delete liveFlushTimer_;
    liveFlushTimer_ = nullptr;

    delete liveMapper_;
    liveMapper_ = nullptr;
    delete livePalette_;
    livePalette_ = nullptr;

    delete queryMapper_;
    queryMapper_ = nullptr;
    delete queryPalette_;
    queryPalette_ = nullptr;

    delete ui;
}

void MainWindow::onRosStatus(const QString &msg)
{
    qDebug() << msg;
    statusBar()->showMessage(msg);
}

// ======================
// Map load/parse
// ======================
bool MainWindow::loadStaticMap(const QString &yamlPath)
{
    MapMeta meta;
    if (!parseMapYaml(yamlPath, meta)) return false;

    QImage img(meta.imagePath);
    if (img.isNull()) return false;

    mapItem->setPixmap(QPixmap::fromImage(img));
    mapMeta_ = meta;
    mapImageSize_ = img.size();

    scene->setSceneRect(QRectF(0, 0, mapImageSize_.width(), mapImageSize_.height()));
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
        } else if (line.startsWith("resolution:")) {
            out.resolution = line.mid(QString("resolution:").size()).trimmed().toDouble();
        } else if (line.startsWith("origin:")) {
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

// ======================
// View transform
// ======================
void MainWindow::resizeEvent(QResizeEvent* e)
{
    QMainWindow::resizeEvent(e);
    applyViewTransform();
}

void MainWindow::showEvent(QShowEvent* e)
{
    QMainWindow::showEvent(e);
    QTimer::singleShot(0, this, [this]{
        applyViewTransform();
    });
}

void MainWindow::applyViewTransform()
{
    if (!ui->graphicsView || !scene) return;
    if (scene->sceneRect().isEmpty()) return;

    auto* v = ui->graphicsView;
    v->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
    v->setResizeAnchor(QGraphicsView::AnchorViewCenter);
    v->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->setRenderHint(QPainter::SmoothPixmapTransform, true);

    v->resetTransform();
    v->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

    // 기존 프로젝트처럼 -90 회전 유지
    QTransform t = v->transform();
    t.rotate(-90.0);
    v->setTransform(t, false);

    v->centerOn(scene->sceneRect().center());
}

// ======================
// Coordinate transforms
// ======================
bool MainWindow::meterToPixel(double x, double y, int& px, int& py) const
{
    if (mapImageSize_.isEmpty()) return false;

    const double res = mapMeta_.resolution;
    const double ox  = mapMeta_.origin_x;
    const double oy  = mapMeta_.origin_y;
    const double th  = mapMeta_.origin_yaw;

    const double dx = x - ox;
    const double dy = y - oy;

    const double c = std::cos(th);
    const double s = std::sin(th);
    const double mx =  c * dx + s * dy;
    const double my = -s * dx + c * dy;

    const double fx = mx / res;
    const double fy = my / res;

    int gx = static_cast<int>(std::floor(fx));
    int gy = static_cast<int>(std::floor(fy));

    if (gx < 0 || gy < 0 || gx >= mapImageSize_.width() || gy >= mapImageSize_.height())
        return false;

    px = gx;
    py = (mapImageSize_.height() - 1) - gy;
    return true;
}

bool MainWindow::pixelToMap(int px, int py, double& x_m, double& y_m) const
{
    if (mapImageSize_.isEmpty()) return false;

    const int W = mapImageSize_.width();
    const int H = mapImageSize_.height();
    if (px < 0 || py < 0 || px >= W || py >= H) return false;

    const double res = mapMeta_.resolution;
    const double ox  = mapMeta_.origin_x;
    const double oy  = mapMeta_.origin_y;
    const double th  = mapMeta_.origin_yaw;

    const int gx = px;
    const int gy = (H - 1) - py;

    const double mx = (static_cast<double>(gx) + 0.5) * res;
    const double my = (static_cast<double>(gy) + 0.5) * res;

    const double c = std::cos(th);
    const double s = std::sin(th);

    const double dx =  c * mx - s * my;
    const double dy =  s * mx + c * my;

    x_m = ox + dx;
    y_m = oy + dy;
    return true;
}

bool MainWindow::sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !mapItem) return false;

    QPointF local = mapItem->mapFromScene(scenePos);
    int ix = static_cast<int>(std::floor(local.x()));
    int iy = static_cast<int>(std::floor(local.y()));

    if (ix < 0 || iy < 0 || ix >= mapImageSize_.width() || iy >= mapImageSize_.height())
        return false;

    px = ix;
    py = iy;
    return true;
}

// ======================
// Robot pose
// ======================
void MainWindow::onRobotPose(double x, double y, double yaw)
{
    if (!mapReady_) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    robotItem->setPos(QPointF(px, py));
    robotItem->setRotation(-rad2deg(yaw));

    if (!poseReady_) poseReady_ = true;
    robotItem->setVisible(ui->chkShowRobot && ui->chkShowRobot->isChecked());
}

// ======================
// Heatmap utils
// ======================
int MainWindow::brushRadiusPx() const
{
    const double r_m = 0.25; // 25cm
    return qMax(1, int(std::round(r_m / mapMeta_.resolution)));
}

float MainWindow::rssiToIntensity01(float rssi) const
{
    // 연속형 정규화 (원하면 3단계 양자화로 바꿔도 됨)
    if (rssi < -90.f) rssi = -90.f;
    if (rssi > -30.f) rssi = -30.f;
    return (rssi + 90.f) / 60.f;
}

// ======================
// Live heatmap (from /wifi/fused)
// ======================
void MainWindow::initLiveHeatmap()
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !scene) return;

    liveCanvas_ = QImage(mapImageSize_, QImage::Format_ARGB32);
    liveCanvas_.fill(Qt::transparent);

    livePalette_ = new GradientPalette(256);
    livePalette_->setColorAt(0.0, QColor(255, 0, 0));
    livePalette_->setColorAt(0.5, QColor(255, 255, 0));
    livePalette_->setColorAt(1.0, QColor(0, 255, 0));

    liveMapper_ = new HeatMapper(&liveCanvas_, livePalette_, brushRadiusPx(), 160, true, true);

    liveHeatItem = scene->addPixmap(QPixmap::fromImage(liveCanvas_));
    liveHeatItem->setZValue(5);
    liveHeatItem->setPos(0, 0);

    liveFlushTimer_ = new QTimer(this);
    connect(liveFlushTimer_, &QTimer::timeout, this, &MainWindow::flushLiveHeatmap);
    liveFlushTimer_->start(100);
}

void MainWindow::flushLiveHeatmap()
{
    if (!liveMapper_ || !liveHeatItem) return;
    liveMapper_->colorize();
    liveHeatItem->setPixmap(QPixmap::fromImage(liveCanvas_));
}

void MainWindow::onFusedSample(double x_m, double y_m, const QString& ssid, int rssi)
{
    if (!mapReady_ || !liveMapper_) return;
    if (!accumulating_) return; // Start/Stop 정책

    // Live에도 필터 적용(원치 않으면 이 2줄 제거)
    if (!filterSsid_.isEmpty() && filterSsid_ != "ALL" && ssid != filterSsid_) return;
    if (filterThrEnable_ && rssi < filterThrRssi_) return;

    int px=0, py=0;
    if (!meterToPixel(x_m, y_m, px, py)) return;

    liveMapper_->addPoint(px + 1, py + 1, rssiToIntensity01((float)rssi));
}

// ======================
// Query heatmap (from /db/get_heatmap)
// ======================
void MainWindow::initQueryHeatmap()
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !scene) return;

    queryCanvas_ = QImage(mapImageSize_, QImage::Format_ARGB32);
    queryCanvas_.fill(Qt::transparent);

    queryPalette_ = new GradientPalette(256);
    queryPalette_->setColorAt(0.0, QColor(255, 0, 0));
    queryPalette_->setColorAt(0.5, QColor(255, 255, 0));
    queryPalette_->setColorAt(1.0, QColor(0, 255, 0));

    queryMapper_ = new HeatMapper(&queryCanvas_, queryPalette_, brushRadiusPx(), 160, true, true);

    queryHeatItem = scene->addPixmap(QPixmap::fromImage(queryCanvas_));
    queryHeatItem->setZValue(6);
    queryHeatItem->setPos(0, 0);
    queryHeatItem->setVisible(false);
}

void MainWindow::flushQueryHeatmap()
{
    if (!queryMapper_ || !queryHeatItem) return;
    queryMapper_->colorize();
    queryHeatItem->setPixmap(QPixmap::fromImage(queryCanvas_));
}

void MainWindow::clearQueryHeatmap()
{
    if (queryCanvas_.isNull()) return;
    queryCanvas_.fill(Qt::transparent);
    flushQueryHeatmap();
    if (queryHeatItem) queryHeatItem->setVisible(false);
}

// ======================
// UI actions
// ======================
void MainWindow::onLayerHeatmap(bool)
{
    applyLayersPolicy();
}

void MainWindow::onLayerRobot(bool)
{
    if (!robotItem) return;
    robotItem->setVisible(ui->chkShowRobot && ui->chkShowRobot->isChecked());
}

void MainWindow::onSessionRefresh()
{
    // 지금 단계에서는 “세션 리스트”를 DB에서 직접 뽑지 않음(요구사항: 과거는 DB 서비스)
    // -> 팀에서 list_sessions 서비스를 추가하거나, UI에서 미리 채운다고 가정
    statusBar()->showMessage("Session refresh: UI list is assumed (no direct DB access).");
}

void MainWindow::on_btnSessionLoad_clicked()
{
    if (!ui->cbSessionId) return;

    const QString sid = ui->cbSessionId->currentData().toString().isEmpty()
                            ? ui->cbSessionId->currentText()
                            : ui->cbSessionId->currentData().toString();

    if (sid.isEmpty()) {
        statusBar()->showMessage("Load failed: empty session_id");
        return;
    }

    currentSessionId_ = sid;
    queryOffset_ = 0;
    requestPastHeatmap();
    applyLayersPolicy();
}

void MainWindow::onQueryClear()
{
    currentSessionId_.clear();
    clearQueryHeatmap();
    applyLayersPolicy();
    statusBar()->showMessage("Query cleared (back to live).");
}

void MainWindow::onApplyFilter()
{
    filterSsid_ = (ui->cbSsid ? ui->cbSsid->currentText() : "ALL");
    filterThrEnable_ = (ui->chkThrEnable ? ui->chkThrEnable->isChecked() : false);
    filterThrRssi_ = (ui->cbThr ? ui->cbThr->currentText().toInt() : -70);

    // 과거 모드면 즉시 재요청
    if (!currentSessionId_.isEmpty()) {
        queryOffset_ = 0;
        requestPastHeatmap();
    }

    statusBar()->showMessage(QString("Filter: ssid=%1 thr=%2")
                                 .arg(filterSsid_).arg(filterThrRssi_));
}

void MainWindow::requestPastHeatmap()
{
    if (!rosThread) return;
    if (currentSessionId_.isEmpty()) return;

    const QString ssid = filterSsid_.isEmpty() ? "ALL" : filterSsid_;
    rosThread->requestHeatmap(currentSessionId_,
                              ssid,
                              filterThrEnable_,
                              filterThrRssi_,
                              queryLimit_,
                              queryOffset_);
}

void MainWindow::onHeatmapReplyArrived(bool ok,
                                       const QString& message,
                                       const QString& session_id,
                                       const QString& ssid,
                                       bool thr_enable,
                                       int thr_rssi,
                                       const QVector<double>& xs,
                                       const QVector<double>& ys,
                                       const QVector<int>& rssis,
                                       const QVector<QString>& /*ssids*/,
                                       const QVector<QString>& /*stamps*/)
{
    Q_UNUSED(session_id);
    Q_UNUSED(ssid);
    Q_UNUSED(thr_enable);
    Q_UNUSED(thr_rssi);

    if (!ok) {
        statusBar()->showMessage("Heatmap load failed: " + message);
        return;
    }

    // query heatmap rebuild
    if (!queryMapper_) return;
    queryCanvas_.fill(Qt::transparent);

    const int n = qMin(xs.size(), ys.size());
    for (int i=0; i<n; ++i) {
        int px=0, py=0;
        if (!meterToPixel(xs[i], ys[i], px, py)) continue;
        queryMapper_->addPoint(px + 1, py + 1, rssiToIntensity01((float)rssis[i]));
    }

    flushQueryHeatmap();
    if (queryHeatItem) queryHeatItem->setVisible(true);

    applyLayersPolicy();
    statusBar()->showMessage(QString("Loaded past heatmap: %1 points (%2)")
                                 .arg(n).arg(message));
}

// ======================
// policy: Query 우선 / 아니면 Live
// ======================
void MainWindow::applyLayersPolicy()
{
    const bool showHeat = (ui->chkShowHeatmap ? ui->chkShowHeatmap->isChecked() : true);

    if (liveHeatItem)  liveHeatItem->setVisible(false);
    if (queryHeatItem) queryHeatItem->setVisible(false);

    if (!showHeat) return;

    const bool queryCtx = !currentSessionId_.isEmpty();
    if (queryCtx) {
        if (queryHeatItem) queryHeatItem->setVisible(true);
        return;
    }
    if (liveHeatItem) liveHeatItem->setVisible(true);
}

// ======================
// click -> nav goal
// ======================
bool MainWindow::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == ui->graphicsView->viewport() &&
        ev->type() == QEvent::MouseButtonPress) {

        auto* me = static_cast<QMouseEvent*>(ev);
        const QPointF scenePos = ui->graphicsView->mapToScene(me->pos());

        int px=0, py=0;
        if (!sceneToMapPixel(scenePos, px, py)) return true;

        double x_m=0.0, y_m=0.0;
        if (!pixelToMap(px, py, x_m, y_m)) return true;

        if (rosThread) rosThread->requestNavigateTo(x_m, y_m, 0.0);
        return true;
    }
    return false;
}
