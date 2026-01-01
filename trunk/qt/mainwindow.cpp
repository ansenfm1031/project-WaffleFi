#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "rosworker.h"          // your RosWorker class
#include "legendbarwidget.h"    // your legend widget

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QDebug>
#include <QResizeEvent>
#include <QShowEvent>
#include <QMouseEvent>
#include <QStatusBar>
#include <QtMath>
#include <cmath>

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // ===== Graphics =====
    initGraphics();

    // ===== Load map =====
    const QString yamlPath = "../../../../docs/maps/map.yaml";
    if (!loadStaticMap(yamlPath)) {
        qDebug() << "Failed to load map yaml:" << yamlPath;
    } else {
        mapReady_ = true;
    }

    // ===== Heat layers init ( live/query/sim) =====
    initHeatLayers();

    // ===== Legend =====
    initLegendOverlay();

    // ===== SSID combo =====
    if (ui->cbSsid) {
        ui->cbSsid->clear();
        ui->cbSsid->addItem("ALL");
        ssidSetLive_.clear();
        ssidSetLive_.insert("ALL");
    }

    // ===== Session combo =====
    if (ui->cbSessionId) {
        ui->cbSessionId->clear();
        ui->cbSessionId->setEditable(true);
        ui->cbSessionId->addItem("(start to create session)", "");
    }

    // ===== Left panel toggle + page switch (by objectName) =====
    // 요구사항: 버튼 누르면 열림/이동, 다시 누르면 닫힘. autoExclusive는 사용하지 않음(기본 false 유지).

    auto setLeftPanelVisible = [this](bool visible){
        if (!ui->leftPanel_2) return;
        ui->leftPanel_2->setVisible(visible);
        // 패널이 열리고 닫힐 때 레이아웃이 깨지는 경우, 필요하면 아래 한 줄 추가 가능
        // ui->centralwidget->updateGeometry();
    };

    auto switchLeftByName = [this](const QString& name){
        if (!ui->leftStack) return;
        QWidget* w = ui->leftStack->findChild<QWidget*>(name);
        if (!w) return;
        ui->leftStack->setCurrentWidget(w);
    };

    // 다른 버튼을 끄는 함수(블록으로 재귀 방지)
    auto uncheckOthers = [this](QToolButton* keep){
        const QList<QToolButton*> btns = { ui->tbSession, ui->tbSim, ui->tbLayers };
        for (auto* b : btns) {
            if (!b || b == keep) continue;
            QSignalBlocker blocker(b);
            b->setChecked(false);
        }
    };

    // 패널 닫기(모든 버튼 OFF + 패널 hide)
    auto closeLeftPanel = [this, setLeftPanelVisible](){
        // 버튼 상태 OFF
        if (ui->tbSession) { QSignalBlocker b(ui->tbSession); ui->tbSession->setChecked(false); }
        if (ui->tbSim)     { QSignalBlocker b(ui->tbSim);     ui->tbSim->setChecked(false); }
        if (ui->tbLayers)  { QSignalBlocker b(ui->tbLayers);  ui->tbLayers->setChecked(false); }

        // 패널 숨김
        setLeftPanelVisible(false);
    };

    // autoExclusive는 쓰지 않으므로, checkable만 확실히 설정
    if (ui->tbSession) { ui->tbSession->setCheckable(true); ui->tbSession->setAutoExclusive(false); }
    if (ui->tbSim)     { ui->tbSim->setCheckable(true);     ui->tbSim->setAutoExclusive(false); }
    if (ui->tbLayers)  { ui->tbLayers->setCheckable(true);  ui->tbLayers->setAutoExclusive(false); }

    //  버튼 토글 로직
    if (ui->tbSession) {
        connect(ui->tbSession, &QToolButton::toggled, this, [=](bool on){
            if (on) {
                uncheckOthers(ui->tbSession);
                setLeftPanelVisible(true);
                switchLeftByName("pageSession");
            } else {
                // 이 버튼이 꺼지는 순간, 다른 버튼도 다 꺼져있다면 패널 닫기
                const bool anyOn = (ui->tbSim && ui->tbSim->isChecked()) || (ui->tbLayers && ui->tbLayers->isChecked());
                if (!anyOn) closeLeftPanel();
            }
        });
    }

    if (ui->tbSim) {
        connect(ui->tbSim, &QToolButton::toggled, this, [=](bool on){
            if (on) {
                uncheckOthers(ui->tbSim);
                setLeftPanelVisible(true);
                switchLeftByName("pageSim");
            } else {
                const bool anyOn = (ui->tbSession && ui->tbSession->isChecked()) || (ui->tbLayers && ui->tbLayers->isChecked());
                if (!anyOn) closeLeftPanel();
            }
        });
    }

    if (ui->tbLayers) {
        connect(ui->tbLayers, &QToolButton::toggled, this, [=](bool on){
            if (on) {
                uncheckOthers(ui->tbLayers);
                setLeftPanelVisible(true);
                switchLeftByName("pageLayers");
            } else {
                const bool anyOn = (ui->tbSession && ui->tbSession->isChecked()) || (ui->tbSim && ui->tbSim->isChecked());
                if (!anyOn) closeLeftPanel();
            }
        });
    }

    //  디폴트: autoExclusive 비활성 + 패널은 닫힌 상태로 시작(원하는 경우)
    closeLeftPanel();
    applyViewTransform();


    // ===== ROS worker =====
    rosThread = new RosWorker(this);

    connect(rosThread, &RosWorker::statusChanged, this, &MainWindow::onRosStatus);
    connect(rosThread, &RosWorker::robotPose,     this, &MainWindow::onRobotPose);
    connect(rosThread, &RosWorker::fusedSample,   this, &MainWindow::onFusedSample);
    connect(rosThread, &RosWorker::startSessionReply, this, &MainWindow::onStartSessionReply);
    connect(rosThread, &RosWorker::stopSessionReply,  this, &MainWindow::onStopSessionReply);
    connect(rosThread, &RosWorker::heatmapReplyArrived, this, &MainWindow::onHeatmapReplyArrived);
    connect(rosThread, &RosWorker::listSessionsReply, this, &MainWindow::onListSessionsReply);
    connect(rosThread, &RosWorker::listSsidsReply, this, &MainWindow::onListSsidsReply);

    rosThread->start();

    // ===== UI connects =====
    connect(ui->chkShowHeatmap, &QCheckBox::toggled, this, &MainWindow::onLayerHeatmap);
    connect(ui->chkShowRobot,   &QCheckBox::toggled, this, &MainWindow::onLayerRobot);
    connect(ui->chkShowApPins,  &QCheckBox::toggled, this, &MainWindow::onLayerPins);

    connect(ui->btnApplyFilter, &QPushButton::clicked, this, &MainWindow::onApplyFilter);

    // Start/Stop toggle (버튼 눌림 유지 방지)
    if (ui->btnMeasureStart) {
        ui->btnMeasureStart->setCheckable(false); // 눌림 유지 방지
        ui->btnMeasureStart->setText("Start");
        connect(ui->btnMeasureStart, &QPushButton::clicked,
                this, &MainWindow::onMeasureToggle);
    }

    if (ui->btnQueryClear) {
        connect(ui->btnQueryClear, &QPushButton::clicked, this, &MainWindow::onQueryClear);
    }

    // Refresh 버튼
    if (ui->btnSessionRefresh) {
        connect(ui->btnSessionRefresh, &QPushButton::clicked,
                this, &MainWindow::onSessionRefresh);
    }

    // 세션 콤보 변경 시 ListSsid 호출
    if (ui->cbSessionId) {
        connect(ui->cbSessionId,
                QOverload<int>::of(&QComboBox::currentIndexChanged),
                this,
                &MainWindow::onSessionComboChanged);
    }


    updateUiByContext();

    // 앱 시작 시 자동 refresh
    QTimer::singleShot(0, this, [this](){ onSessionRefresh(); });
}

MainWindow::~MainWindow()
{
    if (rosThread) {
        rosThread->requestInterruption();
        rosThread->wait();
        // rosThread owned by Qt parent(this) -> auto delete
    }

    if (heatFlushTimer_) {
        heatFlushTimer_->stop();
        delete heatFlushTimer_;
        heatFlushTimer_ = nullptr;
    }

    if (legendPosTimer_) {
        legendPosTimer_->stop();
    }
    // legendOverlay_ is child of viewport, will be deleted by Qt parent chain

    delete ui;
}

void MainWindow::initGraphics()
{
    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->viewport()->installEventFilter(this);

    mapItem = scene->addPixmap(QPixmap());
    mapItem->setZValue(0);

    robotItem = scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
    robotItem->setZValue(10);
    robotItem->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    robotItem->setVisible(false);
}

void MainWindow::onRosStatus(const QString &msg)
{
    qDebug() << msg;
    statusBar()->showMessage(msg, 2000);
}

// ======================
// Map load
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
void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    applyViewTransform();
    updateLegendOverlayGeometry();
}

void MainWindow::showEvent(QShowEvent* e)
{
    QMainWindow::showEvent(e);
    QTimer::singleShot(0, this, [this]{
        applyViewTransform();
        updateLegendOverlayGeometry();
    });
}

void MainWindow::applyViewTransform()
{
    if (!ui->graphicsView || !scene) return;
    if (scene->sceneRect().isEmpty()) return;

    auto* v = ui->graphicsView;
    v->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->setRenderHint(QPainter::SmoothPixmapTransform, true);

    const QRectF sr = scene->sceneRect();
    const QPointF sc = sr.center();
    const QRect vp = v->viewport()->rect();
    if (vp.width() <= 1 || vp.height() <= 1) return;

    const double rotDeg = -180.0; //  오른쪽 180도 유지

    const double w = sr.width();
    const double h = sr.height();
    const double rad = rotDeg * M_PI / 180.0;
    const double c = std::abs(std::cos(rad));
    const double s = std::abs(std::sin(rad));
    const double bboxW = c*w + s*h;
    const double bboxH = s*w + c*h;

    //  "전체가 보이도록" (스크롤 X, 잘림 X)
    const double sx = double(vp.width())  / bboxW;
    const double sy = double(vp.height()) / bboxH;
    const double scale = std::min(sx, sy);
    // const double scale = std::max(sx, sy); //  화면 꽉 채움(잘림 가능)


    QTransform t;
    t.translate(vp.width()*0.5, vp.height()*0.5);
    t.scale(scale, scale);
    t.rotate(rotDeg);
    t.translate(-sc.x(), -sc.y());

    v->setTransform(t, false);
}



// void MainWindow::applyViewTransform()
// {
//     if (!ui->graphicsView || !scene) return;
//     if (scene->sceneRect().isEmpty()) return;

//     auto* v = ui->graphicsView;

//     v->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
//     v->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
//     v->setRenderHint(QPainter::SmoothPixmapTransform, true);

//     const QRectF sr = scene->sceneRect();
//     const QPointF sc = sr.center();
//     const QRect vp = v->viewport()->rect();

//     if (vp.width() <= 1 || vp.height() <= 1) return;

//     //  오른쪽 180도
//     const double rotDeg = -180.0;

//     const double w = sr.width();
//     const double h = sr.height();
//     const double rad = rotDeg * M_PI / 180.0;
//     const double c = std::abs(std::cos(rad));
//     const double s = std::abs(std::sin(rad));

//     const double bboxW = c*w + s*h;
//     const double bboxH = s*w + c*h;

//     // 가로 꽉 채우기
//     const double scale = (bboxW > 1e-6)
//                              ? double(vp.width()) / bboxW
//                              : 1.0;

//     QTransform t;
//     t.translate(vp.width() * 0.5, vp.height() * 0.5);
//     t.scale(scale, scale);
//     t.rotate(rotDeg);
//     t.translate(-sc.x(), -sc.y());

//     v->setTransform(t, false);
// }



// void MainWindow::()
// {
//     if (!ui->graphicsView || !scene) return;
//     if (scene->sceneRect().isEmpty()) return;

//     auto* v = ui->graphicsView;

//     v->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
//     v->setResizeAnchor(QGraphicsView::AnchorViewCenter);
//     v->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
//     v->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
//     v->setRenderHint(QPainter::SmoothPixmapTransform, true);

//     v->resetTransform();
//     v->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

//     // 기존 프로젝트 정책(회전 -90)
//     QTransform t = v->transform();
//     t.rotate(-90.0);
//     v->setTransform(t, false);

//     v->centerOn(scene->sceneRect().center());
// }

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
// Heat layers init ( live/query/sim)
// ======================
void MainWindow::initHeatLayers()
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !scene) return;

    const int rpx = qMax(1, int(std::round(0.25 / mapMeta_.resolution)));
    const int opacity = 160;

    // live layer (z=5)
    liveLayer_.init(scene, mapImageSize_, /*z=*/5, rpx, opacity);
    liveLayer_.setVisible(true);

    // query layer (z=6)
    queryLayer_.init(scene, mapImageSize_, /*z=*/6, rpx, opacity);
    queryLayer_.setVisible(false);

    // sim layer (z=7)
    simLayer_.init(scene, mapImageSize_, /*z=*/7, rpx, opacity);
    simLayer_.setVisible(false);

    // flush timer: 평균 렌더는 100ms에 한 번
    if (!heatFlushTimer_) {
        heatFlushTimer_ = new QTimer(this);
        connect(heatFlushTimer_, &QTimer::timeout, this, [this](){
            // Query 모드에서는 live를 숨기므로, live flush는 해도 되지만 비용 아끼려면 조건으로
            if (loadedSessionId_.isEmpty() && ui->chkShowHeatmap->isChecked()) {
                liveLayer_.flushMean();
            } else {
                // 그래도 탐사중 데이터는 계속 누적될 수 있으니,
                // 필요하면 여기서도 flushMean() 호출 가능.
            }
        });
        heatFlushTimer_->start(100);
    }
}

float MainWindow::rssiToIntensity01(float rssi) const
{
    if (rssi < -90.f) rssi = -90.f;
    if (rssi > -30.f) rssi = -30.f;
    return (rssi + 90.f) / 60.f;
}

void MainWindow::updateSsidComboLive(const QString& ssid)
{
    if (!ui->cbSsid) return;
    if (ssid.isEmpty()) return;
    if (ssidSetLive_.contains(ssid)) return;

    ssidSetLive_.insert(ssid);
    ui->cbSsid->addItem(ssid);
}

void MainWindow::addLivePointMean(double x_m, double y_m, int rssi)
{
    if (!mapReady_ || !liveLayer_.isReady()) return;

    int px=0, py=0;
    if (!meterToPixel(x_m, y_m, px, py)) return;

    // HeatMapper는 1-based를 쓰는 기존 정책 유지
    liveLayer_.addPointMean(px + 1, py + 1, rssiToIntensity01((float)rssi));
}

// ======================
// Robot pose
// ======================
void MainWindow::onRobotPose(double x, double y, double yaw)
{
    if (!mapReady_ || !robotItem) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    robotItem->setPos(QPointF(px, py));

    //  180도 회전 보정
    robotItem->setRotation(viewRotateDeg_ - rad2deg(yaw));

    if (!poseReady_) {
        poseReady_ = true;
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);
    }
}

// ======================
// Live fused sample ->  전체 SSID 평균 히트맵
// ======================
void MainWindow::onFusedSample(double x_m, double y_m, const QString& ssid, int rssi)
{
    if (!mapReady_ || !liveLayer_.isReady()) return;

    // SSID 리스트 실시간 갱신
    updateSsidComboLive(ssid);

    // Start(누적 ON)일 때만 live heatmap에 반영
    if (!accumulating_) return;

    // live는 “전체 평균”이지만, 사용자가 필터를 걸어보고 싶다면 유지 가능(원하면 제거)
    if (!filterSsid_.isEmpty() && filterSsid_ != "ALL" && ssid != filterSsid_) return;
    if (filterThrEnable_ && rssi < filterThrRssi_) return;

    addLivePointMean(x_m, y_m, rssi);
}

// ======================
// Session Start/Stop (Trigger)
// ======================
// void MainWindow::onMeasureToggle()
// {
//     if (!rosThread) return;

//     // Start
//     if (!accumulating_) {
//         if (sessionPending_) return;
//         sessionPending_ = true;

//         //  즉시 UI를 Stop 상태처럼 보이게
//         startOptimistic_ = true;
//         if (ui->btnMeasureStart) {
//             ui->btnMeasureStart->setText("Stop");
//             ui->btnMeasureStart->setEnabled(false); // 응답 올 때까지 잠깐 잠금
//         }
//         statusBar()->showMessage("Starting session...");

//         rosThread->requestStartSession();
//         return;
//     }

//     // Stop
//     accumulating_ = false;
//     startOptimistic_ = false;
//     updateUiByContext();
//     applyLayersPolicy();

//     rosThread->requestStopSession();
//     statusBar()->showMessage("Stopping session...");
// }
void MainWindow::onMeasureToggle()
{
    if (!rosThread || !ui->btnMeasureStart) return;

    if (sessionPending_) return; // 중복 클릭 방지

    const QString curText = ui->btnMeasureStart->text().trimmed();

    // 1) Start 텍스트가 보이면 -> Start 기능 실행 + 즉시 Stop으로 바꾸기
    if (curText.compare("Start", Qt::CaseInsensitive) == 0) {

        sessionPending_ = true;
        pendingStartUi_ = true;
        pendingStopUi_  = false;

        //  즉시 UI 변경 (요구사항)
        ui->btnMeasureStart->setText("Stop");
        ui->btnMeasureStart->setEnabled(false);
        statusBar()->showMessage("Starting session...");

        // 실제 기능 실행
        rosThread->requestStartSession();
        return;
    }

    // 2) Stop 텍스트가 보이면 -> Stop 기능 실행 + 즉시 Start로 바꾸기
    if (curText.compare("Stop", Qt::CaseInsensitive) == 0) {

        sessionPending_ = true;
        pendingStopUi_  = true;
        pendingStartUi_ = false;

        //  즉시 UI 변경 (요구사항)
        ui->btnMeasureStart->setText("Start");
        ui->btnMeasureStart->setEnabled(false);
        statusBar()->showMessage("Stopping session...");

        // 실제 기능 실행
        rosThread->requestStopSession();
        return;
    }

    // 예상치 못한 텍스트면 안전하게 복구
    ui->btnMeasureStart->setText(accumulating_ ? "Stop" : "Start");
}


void MainWindow::onStartSessionReply(bool ok, const QString& session_id_or_msg)
{
    sessionPending_ = false;
    if (ui->btnMeasureStart) ui->btnMeasureStart->setEnabled(true);

    if (!pendingStartUi_) {
        // (혹시라도) start 응답이 왔는데 UI가 start-flow가 아니었다면 상태만 반영
        if (ok) accumulating_ = true;
        return;
    }
    pendingStartUi_ = false;

    if (!ok) {
        // ❌ start 실패 -> 즉시 바꿨던 텍스트를 원복
        accumulating_ = false;
        activeSessionId_.clear();
        if (ui->btnMeasureStart) ui->btnMeasureStart->setText("Start");
        statusBar()->showMessage("Start failed: " + session_id_or_msg, 3000);
        return;
    }

    const QString sid = session_id_or_msg.trimmed();
    if (sid.isEmpty()) {
        accumulating_ = false;
        activeSessionId_.clear();
        if (ui->btnMeasureStart) ui->btnMeasureStart->setText("Start");
        statusBar()->showMessage("Start failed: empty session_id", 3000);
        return;
    }

    //  start 성공
    accumulating_ = true;
    activeSessionId_ = sid;

    // (선택) 새 세션 시작 시 live heatmap 초기화
    liveLayer_.clear(true);

    statusBar()->showMessage("Session started: " + sid, 2000);

    // start 직후 세션/SSID 갱신 원하면
    onSessionRefresh();
}


void MainWindow::onStopSessionReply(bool ok, const QString& session_id_or_msg)
{
    sessionPending_ = false;
    if (ui->btnMeasureStart) ui->btnMeasureStart->setEnabled(true);

    if (!pendingStopUi_) {
        if (ok) accumulating_ = false;
        return;
    }
    pendingStopUi_ = false;

    if (!ok) {
        // stop 실패 -> 즉시 바꿨던 텍스트를 원복(Stop으로 돌아가야 함)
        accumulating_ = true; // stop이 실패했으면 측정중으로 보는 게 자연스러움
        if (ui->btnMeasureStart) ui->btnMeasureStart->setText("Stop");
        statusBar()->showMessage("Stop failed: " + session_id_or_msg, 3000);
        return;
    }

    //  stop 성공
    accumulating_ = false;
    activeSessionId_.clear();

    statusBar()->showMessage("Session stopped: " + session_id_or_msg, 2000);

    // (선택) stop 후 세션 목록 최신화
    onSessionRefresh();
}


// ======================
// Past(Query) heatmap load (GetHeatmap)
// ======================
void MainWindow::on_btnSessionLoad_clicked()
{
    if (!rosThread) return;
    if (!ui->cbSessionId) return;

    const QString sid = ui->cbSessionId->currentData().toString().isEmpty()
                            ? ui->cbSessionId->currentText().trimmed()
                            : ui->cbSessionId->currentData().toString().trimmed();

    if (sid.isEmpty() || sid.startsWith("(")) {
        statusBar()->showMessage("Load failed: invalid session_id");
        return;
    }

    loadedSessionId_ = sid;

    const uint32_t limit  = 200000;
    const uint32_t offset = 0;

    statusBar()->showMessage(QString("Loading past heatmap: sid=%1 ...").arg(sid));

    rosThread->requestHeatmap(
        sid,
        (filterSsid_.isEmpty() ? "ALL" : filterSsid_),
        filterThrEnable_,
        filterThrRssi_,
        limit,
        offset
        );
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
                                       const QVector<QString>&,
                                       const QVector<QString>&)
{
    if (!ok) {
        statusBar()->showMessage("GetHeatmap failed: " + message);
        return;
    }

    if (!queryLayer_.isReady() || !mapReady_) return;

    //  Query 평균 히트맵: 통계 초기화 후 sum/cnt로 다시 계산
    queryLayer_.clear(true);

    const int n = qMin(xs.size(), qMin(ys.size(), rssis.size()));
    for (int i=0; i<n; ++i) {
        // int px=0, py=0;
        // if (!meterToPixel(xs[i], ys[i], px, py)) continue;
        // queryLayer_.addPointMean(px + 1, py + 1, rssiToIntensity01((float)rssis[i]));
        // xs/ys는 grid/pixel 좌표라고 했으므로 그대로 사용
        const int n = qMin(xs.size(), qMin(ys.size(), rssis.size()));
        for (int i=0; i<n; ++i) {
            const int px = (int)std::lround(xs[i]);   // 또는 (int)xs[i]
            const int py = (int)std::lround(ys[i]);

            // 맵 이미지 범위 체크
            if (px < 0 || py < 0 || px >= mapImageSize_.width() || py >= mapImageSize_.height())
                continue;

            // HeatMapper가 1-based 좌표를 쓰는 정책이면 +1 유지
            queryLayer_.addPointMean(px + 1, py + 1, rssiToIntensity01((float)rssis[i]));
        }
        queryLayer_.flushMean();
        queryLayer_.setVisible(true);
    }

    queryLayer_.flushMean();
    queryLayer_.setVisible(true);

    loadedSessionId_ = session_id;

    // query가 live보다 우선
    applyLayersPolicy();

    statusBar()->showMessage(
        QString("Loaded past AVG heatmap rows=%1 (sid=%2, ssid=%3, thr=%4)")
            .arg(n).arg(session_id).arg(ssid).arg(thr_enable ? thr_rssi : 0)
        );
}

void MainWindow::onQueryClear()
{
    if (queryLayer_.isReady()) {
        queryLayer_.clear(true);
        queryLayer_.flushMean();
        queryLayer_.setVisible(false);
    }
    loadedSessionId_.clear();
    applyLayersPolicy();
    updateUiByContext();
    statusBar()->showMessage("Query cleared.");
}

// ======================
// Filter
// ======================
void MainWindow::onApplyFilter()
{
    filterSsid_ = ui->cbSsid ? ui->cbSsid->currentText() : "ALL";

    // Threshold UI가 숨김이지만, 기존 UX 유지: cbThr 읽어 적용
    filterThrEnable_ = true;
    filterThrRssi_ = ui->cbThr ? ui->cbThr->currentText().toInt() : -70;

    // Query 모드면 자동 재로드
    if (!loadedSessionId_.isEmpty()) {
        if (ui->cbSessionId) {
            for (int i=0; i<ui->cbSessionId->count(); ++i) {
                if (ui->cbSessionId->itemData(i).toString() == loadedSessionId_ ||
                    ui->cbSessionId->itemText(i) == loadedSessionId_) {
                    ui->cbSessionId->setCurrentIndex(i);
                    break;
                }
            }
        }
        on_btnSessionLoad_clicked();
    }

    updateUiByContext();
}

// ======================
// Layers policy
// ======================
MainWindow::Mode MainWindow::deriveModeFromContext() const
{
    if (!loadedSessionId_.isEmpty()) return Mode::Query;
    if (accumulating_) return Mode::Measurement;
    return Mode::View;
}

void MainWindow::updateUiByContext()
{
    const Mode m = deriveModeFromContext();

    if (ui->btnMeasureStart) {
        if (m == Mode::Measurement || startOptimistic_) ui->btnMeasureStart->setText("Stop");
        else                                           ui->btnMeasureStart->setText("Start");
        ui->btnMeasureStart->setEnabled(!sessionPending_);
    }

    if (robotItem)
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);

    applyLayersPolicy();
}


void MainWindow::applyLayersPolicy()
{
    const bool showHeat = ui->chkShowHeatmap->isChecked();

    liveLayer_.setVisible(false);
    if (queryLayer_.isReady()) queryLayer_.setVisible(false);
    if (simLayer_.isReady())   simLayer_.setVisible(false);

    if (!showHeat) return;

    const bool queryCtx = !loadedSessionId_.isEmpty();

    if (queryCtx && queryLayer_.isReady()) {
        queryLayer_.setVisible(true);
        return;
    }

    // 기본은 live
    liveLayer_.setVisible(true);
}

void MainWindow::onLayerHeatmap(bool)
{
    applyLayersPolicy();
}

void MainWindow::onLayerRobot(bool)
{
    if (robotItem)
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);
}

void MainWindow::onLayerPins(bool)
{
    // AP Pins/sim pins 정책은 기존 프로젝트 코드 유지 대상
}

// ======================
// Click handling (nav goal)
// ======================
bool MainWindow::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == ui->graphicsView->viewport() &&
        ev->type() == QEvent::MouseButtonPress) {

        auto* me = static_cast<QMouseEvent*>(ev);
        const QPointF scenePos = ui->graphicsView->mapToScene(me->pos());

        // Left click -> nav goal
        if (me->button() == Qt::LeftButton) {
            int px=0, py=0;
            if (!sceneToMapPixel(scenePos, px, py)) return true;

            double x_m=0.0, y_m=0.0;
            if (!pixelToMap(px, py, x_m, y_m)) return true;

            if (rosThread) rosThread->requestNavigateTo(x_m, y_m, 0.0);
            return true;
        }
        return true;
    }
    return false;
}

// ======================
// Legend overlay
// ======================
// void MainWindow::initLegendOverlay()
// {
//     if (!ui->graphicsView) return;
//     if (legendOverlay_) return;

//     legendOverlay_ = new LegendBarWidget(ui->graphicsView->viewport());
//     legendOverlay_->setRangeDbm(-80, 0);
//     legendOverlay_->setTitle("Voice + Data | dBm");
//     legendOverlay_->setValueDbm(-67);

//     legendOverlay_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
//     legendOverlay_->setStyleSheet("background: transparent;");
//     legendOverlay_->setFixedSize(360, 64);

//     updateLegendOverlayGeometry();
//     legendOverlay_->show();
// }

// void MainWindow::updateLegendOverlayGeometry()
// {
//     if (!legendOverlay_ || !ui->graphicsView) return;

//     const int margin = 12;
//     const QRect vp = ui->graphicsView->viewport()->rect();

//     const int w = legendOverlay_->width();
//     const int h = legendOverlay_->height();

//     const int x = vp.right() - w - margin;
//     const int y = vp.bottom() - h - margin;

//     legendOverlay_->move(x, y);
// }

void MainWindow::onListSsidsReply(bool ok,
                                  const QString& message,
                                  const QString& session_id,
                                  const QList<QString>& ssids)
{
    Q_UNUSED(session_id);

    if (!ok) {
        statusBar()->showMessage("ListSsids failed: " + message, 3000);
        // 실패 시 최소한 ALL만 남김
        if (ui->cbSsid) {
            ui->cbSsid->clear();
            ui->cbSsid->addItem("ALL");
        }
        return;
    }

    if (!ui->cbSsid) return;

    //  세션 선택 시 “해당 세션의 SSID만”으로 갱신
    ui->cbSsid->blockSignals(true);
    ui->cbSsid->clear();
    ui->cbSsid->addItem("ALL");
    for (const auto& s : ssids) {
        if (!s.isEmpty())
            ui->cbSsid->addItem(s);
    }
    ui->cbSsid->setCurrentIndex(0);
    ui->cbSsid->blockSignals(false);

    statusBar()->showMessage(QString("SSID list updated (%1)").arg(ssids.size()), 1500);
}

// ======================
// A) Session Refresh (ListSessions.srv)
// ======================
void MainWindow::onSessionRefresh()
{
    if (!rosThread) return;

    // DB 전체 목록 갱신
    statusBar()->showMessage("Refreshing sessions...");

    // limit/offset UI가 없으니 기본값
    const uint32_t limit = 200;
    const uint32_t offset = 0;

    rosThread->requestListSessions(limit, offset);
}

// ======================
// B) Session combo changed -> ListSsids.srv
// ======================
void MainWindow::onSessionComboChanged(int /*index*/)
{
    if (!rosThread) return;

    const QString sid = currentSessionIdText();
    if (sid.isEmpty() || sid.startsWith("(")) return;

    // 세션 선택 시 해당 세션의 SSID 목록만 가져오기
    const uint32_t limit = 200;
    const uint32_t offset = 0;

    rosThread->requestListSsid(sid, limit, offset);
}

// ======================
// ListSessions reply
// ======================
void MainWindow::onListSessionsReply(bool ok,
                                     const QString& message,
                                     const QList<QString>& session_ids,
                                     const QList<QString>& started_at,
                                     const QList<QString>& ended_at)
{
    Q_UNUSED(started_at);
    Q_UNUSED(ended_at);

    if (!ui->cbSessionId) return;

    if (!ok) {
        statusBar()->showMessage("ListSessions failed: " + message, 3000);
        return;
    }

    ui->cbSessionId->blockSignals(true);
    ui->cbSessionId->clear();
    ui->cbSessionId->setEditable(true);

    // 표시용: session_id만 넣음 (started/ended는 필요하면 itemText에 붙이기)
    for (const auto& sid : session_ids) {
        if (!sid.isEmpty())
            ui->cbSessionId->addItem(sid, sid);
    }

    if (ui->cbSessionId->count() == 0)
        ui->cbSessionId->addItem("(no sessions)", "");

    ui->cbSessionId->setCurrentIndex(0);
    ui->cbSessionId->blockSignals(false);

    statusBar()->showMessage(QString("Sessions updated (%1)").arg(session_ids.size()), 1500);

    // 첫 세션 기준으로 SSID 목록도 갱신
    onSessionComboChanged(ui->cbSessionId->currentIndex());
}

// ======================
// current session text helper (링커 에러 원인)
// ======================
QString MainWindow::currentSessionIdText() const
{
    if (!ui->cbSessionId) return "";

    const QString data = ui->cbSessionId->currentData().toString().trimmed();
    if (!data.isEmpty()) return data;

    return ui->cbSessionId->currentText().trimmed();
}


void MainWindow::initLegendOverlay()
{
    if (!ui->graphicsView) return;
    if (legendOverlay_) return;

    legendOverlay_ = new LegendBarWidget(ui->graphicsView->viewport());
    legendOverlay_->setRangeDbm(-80, 0);
    legendOverlay_->setTitle("Voice + Data | dBm");
    legendOverlay_->setValueDbm(-67);

    //  오버레이: 클릭/드래그 방해 금지
    legendOverlay_->setAttribute(Qt::WA_TransparentForMouseEvents, true);

    //  배경은 살짝만(겹쳐도 가독성 확보) - 원치 않으면 제거 가능
    legendOverlay_->setStyleSheet(
        "background: rgba(0,0,0,80);"
        "border-radius: 10px;"
        );

    legendOverlay_->setFixedSize(360, 64);
    legendOverlay_->show();

    //  항상 viewport 우하단으로 재배치
    updateLegendOverlayGeometry();

    //  viewport 리사이즈/레이아웃 변동을 놓치는 경우 대비: 주기적으로 위치 보정(가벼움)
    if (!legendPosTimer_) {
        legendPosTimer_ = new QTimer(this);
        connect(legendPosTimer_, &QTimer::timeout, this, [this]{
            updateLegendOverlayGeometry();
        });
        legendPosTimer_->start(200);  // 0.2초마다 위치 보정
    }
}

void MainWindow::updateLegendOverlayGeometry()
{
    if (!legendOverlay_ || !ui->graphicsView) return;

    const int margin = 12;
    const QRect vp = ui->graphicsView->viewport()->rect();

    const int w = legendOverlay_->width();
    const int h = legendOverlay_->height();

    const int x = vp.width()  - w - margin;
    const int y = vp.height() - h - margin;

    legendOverlay_->move(x, y);
    legendOverlay_->raise();   //  항상 위로
}

