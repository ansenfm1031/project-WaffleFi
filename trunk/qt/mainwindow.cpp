#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dbmanager.h"

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <cmath>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QShowEvent>
#include <QTimer>
#include <QDateTime>

    static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //======================12-29============================
    ui->leftPanel->setVisible(false);                 // 디폴트: 왼쪽 패널 숨김
    ui->leftStack->setCurrentWidget(ui->pageSession); // 기본 페이지(숨겨져 있어도 OK)

    auto showPanel = [&](QWidget* page) {
        ui->leftPanel->setVisible(true);
        ui->leftStack->setCurrentWidget(page);
    };

    auto hidePanel = [&]() {
        ui->leftPanel->setVisible(false);
    };

    // 탭 버튼 초기 설정 (Mode 탭은 UI에서 제거/숨김을 권장)
    auto initTab = [&](QToolButton* tabBtn, QWidget* pageWidget) {
        if (!tabBtn || !pageWidget) return;

        tabBtn->setCheckable(true);
        tabBtn->setAutoExclusive(false);

        connect(tabBtn, &QToolButton::clicked, this, [=]() {
            bool panelVisible = ui->leftPanel->isVisible();
            bool samePage = (ui->leftStack->currentWidget() == pageWidget);

            if (panelVisible && samePage) {
                hidePanel();
                tabBtn->setChecked(false);
                return;
            }

            showPanel(pageWidget);

            // 수동 exclusive
            if (ui->tbMode)    ui->tbMode->setChecked(tabBtn == ui->tbMode);
            if (ui->tbSession) ui->tbSession->setChecked(tabBtn == ui->tbSession);
            if (ui->tbSim)     ui->tbSim->setChecked(tabBtn == ui->tbSim);
            if (ui->tbMetric)  ui->tbMetric->setChecked(tabBtn == ui->tbMetric);
            if (ui->tbLayers)  ui->tbLayers->setChecked(tabBtn == ui->tbLayers);
        });
    };

    // tbMode는 UI에서 삭제/숨김 추천. 남아있으면 pageMode 대신 다른 페이지로 연결하거나 비활성.
    if (ui->tbMode) {
        ui->tbMode->setVisible(false);  // Mode 버튼 제거 UX
        ui->tbMode->setEnabled(false);
        ui->tbMode->setChecked(false);
    }

    initTab(ui->tbSession, ui->pageSession);
    initTab(ui->tbSim,     ui->pageSim);
    initTab(ui->tbMetric,  ui->pageMetric);
    initTab(ui->tbLayers,  ui->pageLayers);

    if (ui->tbSession) ui->tbSession->setChecked(false);
    if (ui->tbSim)     ui->tbSim->setChecked(false);
    if (ui->tbMetric)  ui->tbMetric->setChecked(false);
    if (ui->tbLayers)  ui->tbLayers->setChecked(false);
    //=======================================================

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->viewport()->installEventFilter(this);

    mapItem = scene->addPixmap(QPixmap());
    mapItem->setZValue(0);

    robotItem = scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
    robotItem->setZValue(10);
    robotItem->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    robotItem->setVisible(true);

    const QString yamlPath = "../../../../docs/maps/map.yaml";
    if (!loadStaticMap(yamlPath)) {
        qDebug() << "Failed to load map yaml:" << yamlPath;
    } else {
        mapReady_ = true;
    }

    if (mapReady_) {
        const double radius_m = 0.25; // 25cm
        const int radius_px = qMax(1, int(std::round(radius_m / mapMeta_.resolution)));
        simLayer_.init(scene, mapImageSize_, /*z=*/7, radius_px, /*opacity=*/160);
        simLayer_.setVisible(false);

    }

    initHeatmapLayer();

    // DB open/init
    db_.open(QDir::homePath() + "/wifi_qt.db");
    db_.initSchema();

    if (mapReady_) {
        const double radius_m = 0.25; // 25cm
        const int radius_px = qMax(1, int(std::round(radius_m / mapMeta_.resolution)));
        queryLayer_.init(scene, mapImageSize_, /*z=*/6, radius_px, /*opacity=*/160);
        queryLayer_.setVisible(false);
    }

    rosThread = new RosWorker(this);
    connect(rosThread, &RosWorker::statusChanged, this, &MainWindow::onRosStatus);
    connect(rosThread, &RosWorker::robotPose, this, &MainWindow::onRobotPose);
    connect(rosThread, &RosWorker::sample, this, &MainWindow::onSample);
    connect(rosThread, &RosWorker::sample, this, &MainWindow::onSampleToDb);
    rosThread->start();

    // Layers
    connect(ui->chkShowHeatmap, &QCheckBox::toggled, this, [this](bool){
        updateUiByContext();
    });
    connect(ui->chkShowRobot, &QCheckBox::toggled, this, &MainWindow::onLayerRobot);
    connect(ui->chkShowApPins, &QCheckBox::toggled, this, &MainWindow::onLayerPins);

    // AutoExplore
    connect(ui->chkAutoExplore, &QCheckBox::toggled,
            this, &MainWindow::onAutoExploreToggled);

    // Session / Measure / Filter
    connect(ui->btnSessionRefresh, &QPushButton::clicked, this, &MainWindow::onSessionRefresh);
    connect(ui->btnSessionLoad,    &QPushButton::clicked, this, &MainWindow::onSessionLoad);
    connect(ui->btnSessionDelete,  &QPushButton::clicked, this, &MainWindow::onSessionDelete);

    //connect(ui->btnMeasureStart, &QPushButton::clicked, this, &MainWindow::onMeasureStart);
    //connect(ui->btnMeasureStop,  &QPushButton::clicked, this, &MainWindow::onMeasureStop);
//========================
    // 1버튼 토글: checkable 쓰지 않음
    ui->btnMeasureStart->setCheckable(false);
    ui->btnMeasureStart->setText("Start");

    // Stop 버튼은 제거/숨김(1버튼만 사용)
    if (ui->btnMeasureStop) {
        ui->btnMeasureStop->setVisible(false);
        ui->btnMeasureStop->setEnabled(false);
    }

    connect(ui->btnMeasureStart, &QPushButton::clicked, this, [this]() {
        if (!measuringDb_) onMeasureStart();
        else               onMeasureStop();

        // ✅ 텍스트/상태 동기화는 updateUiByContext가 책임지게
        updateUiByContext();

        // ✅ 눌림/포커스 잔상 제거 (다음 이벤트 루프에서)
        QTimer::singleShot(0, this, [this]{
            ui->btnMeasureStart->setDown(false);
            ui->btnMeasureStart->clearFocus();
            ui->btnMeasureStart->repaint();
        });
    });


    // connect(ui->btnMeasureStart, &QPushButton::clicked, this, [this]() {
    //     if (!measuringDb_) {
    //         onMeasureStart();                 // DB 저장 시작 + 세션 생성
    //     } else {
    //         onMeasureStop();                  // DB 저장 종료
    //     }

    //     // 클릭 후 눌림 잔상 제거(테마/스타일에 따라 필요)
    //     ui->btnMeasureStart->setDown(false);
    //     ui->btnMeasureStart->setChecked(false);
    //     ui->btnMeasureStart->clearFocus();

    //     // 텍스트 동기화
    //     ui->btnMeasureStart->setText(measuringDb_ ? "Stop" : "Start");
    // });

    if (ui->btnMeasureStart) {
        ui->btnMeasureStart->setEnabled(true);
        ui->btnMeasureStart->setText(measuringDb_ ? "Stop" : "Start");
    }
    if (ui->btnMeasureStop) {
        ui->btnMeasureStop->setVisible(false);
    }

//========================

    connect(ui->btnApplyFilter, &QPushButton::clicked, this, &MainWindow::onApplyFilter);

    // Sim
    connect(ui->chkSimEnable, &QCheckBox::toggled, this, &MainWindow::onSimEnable);
    connect(ui->spTxPower, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onSimParamsChanged);
    connect(ui->spSimChannel, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onSimParamsChanged);
    connect(ui->cbSimBandwidth, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onSimParamsChanged);
    connect(ui->btnClearPins, &QPushButton::clicked, this, &MainWindow::onClearPins);

    // Metric
    connect(ui->cbMetric, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onMetricChanged);

    // AutoExplorer integration
    connect(rosThread, &RosWorker::goalStatus,
            &autoExplorer_, &AutoExplorer::onGoalStatus);
    connect(&autoExplorer_, &AutoExplorer::newGoal,
            this, &MainWindow::onAutoGoal);

    // init
    onMetricChanged(ui->cbMetric->currentIndex());
    onSessionRefresh();

    // 최종 UI/레이어 정책은 컨텍스트 기반으로 단일 진입점
    updateUiByContext();
    initLegendOverlay();
}

MainWindow::~MainWindow()
{
    if (rosThread) {
        rosThread->requestInterruption();
        rosThread->wait();
    }

    delete heatFlushTimer_;
    heatFlushTimer_ = nullptr;

    delete heatMapper_;
    heatMapper_ = nullptr;

    delete palette_;
    palette_ = nullptr;

    delete ui;
}

void MainWindow::onRosStatus(const QString &msg)
{
    qDebug() << msg;
}

//======================
// 컨텍스트 기반 Mode 결정 (Mode 버튼 제거 핵심)
//======================
MainWindow::Mode MainWindow::deriveModeFromContext() const
{
    // Sim이 클릭 의미를 바꾸므로 최우선
    if (ui->chkSimEnable->isChecked())
        return Mode::SimAP;

    // 세션이 로드되어 있으면 Query 컨텍스트
    if (!currentLoadedSession_.isEmpty())
        return Mode::Query;

    // 측정 중이면 Measurement
    if (measuringDb_)
        return Mode::Measurement;

    // 기본 View
    return Mode::View;
}

//======================
// UI/레이어/상태를 단일 진입점으로 갱신
//======================
void MainWindow::updateUiByContext()
{
    currentMode_ = deriveModeFromContext();

    // Measure: 항상 접근 가능, 상태에 따라 Start/Stop enable만 변경
    if (ui->gbMeasure) ui->gbMeasure->setEnabled(true);
    // if (ui->btnMeasureStart) ui->btnMeasureStart->setEnabled(!measuringDb_);
    // if (ui->btnMeasureStop)  ui->btnMeasureStop->setEnabled(measuringDb_);
    if (ui->btnMeasureStart) {
        ui->btnMeasureStart->setEnabled(true);
        ui->btnMeasureStart->setText(measuringDb_ ? "Stop" : "Start");
    }

    if (ui->btnMeasureStop) {
        ui->btnMeasureStop->setEnabled(false);
        ui->btnMeasureStop->setVisible(false);
    }
    // Session/Filter: 항상 접근 가능
    if (ui->gbSession) ui->gbSession->setEnabled(true);
    if (ui->gbFilter)  ui->gbFilter->setEnabled(true);

    // Sim: Sim Enable ON일 때만 파라미터 조작 가능
    simEnabled_ = ui->chkSimEnable->isChecked();
    if (ui->gbSim) ui->gbSim->setEnabled(simEnabled_);

    // Metric: 항상 선택 가능
    if (ui->gbMetric) ui->gbMetric->setEnabled(true);

    // 로봇 표시
    if (robotItem)
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);

    // 레이어 정책 반영(겹침 방지 + 우선순위)
    applyLayersPolicy();

    // 상태바 안내(선택)
    if (simEnabled_) {
        statusBar()->showMessage("Sim: Shift+Click or Right Click to drop AP pins");
    } else if (!currentLoadedSession_.isEmpty()) {
        statusBar()->showMessage("Query: loaded session. Filter/Metric changes apply.");
    } else if (measuringDb_) {
        statusBar()->showMessage("Measuring: samples are being stored.");
    } else {
        statusBar()->showMessage("View: live heatmap.");
    }
}

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

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    applyViewTransform();
    updateLegendOverlayGeometry();   // ✅ 추가
}

void MainWindow::showEvent(QShowEvent* e)
{
    QMainWindow::showEvent(e);
    QTimer::singleShot(0, this, [this]{
        applyViewTransform();
        updateLegendOverlayGeometry(); // ✅ 추가
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

    // 기존과 동일한 회전/fit 정책 유지
    QTransform rot;
    rot.rotate(-90.0);
    v->setTransform(rot, true);

    v->resetTransform();
    v->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

    QTransform t = v->transform();
    t.rotate(-90.0);
    v->setTransform(t, false);

    v->centerOn(scene->sceneRect().center());
    updateLegendOverlayGeometry();
}

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

void MainWindow::onRobotPose(double x, double y, double yaw)
{
    qDebug() << "[POSE]" << x << y << yaw;
    if (!mapReady_) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    robotItem->setPos(QPointF(px, py));
    robotItem->setRotation(-rad2deg(yaw));

    if (!poseReady_) {
        poseReady_ = true;
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);
    }
}

void MainWindow::initHeatmapLayer()
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !scene) return;

    const int w = mapImageSize_.width();
    const int h = mapImageSize_.height();

    heatCanvas_ = QImage(w, h, QImage::Format_ARGB32);
    heatCanvas_.fill(Qt::transparent);

    delete palette_;
    palette_ = new GradientPalette(256);
    palette_->setColorAt(0.0, QColor(255, 0, 0));
    palette_->setColorAt(0.5, QColor(255, 255, 0));
    palette_->setColorAt(1.0, QColor(0, 255, 0));

    // ===== 25cm 반경 설정 =====
    const double radius_m = 0.25; // 25cm
    const int radius_px = qMax(1, int(std::round(radius_m / mapMeta_.resolution)));
    const int opacity = 160;
    const bool absoluteMode = true;
    const bool cap01 = true;

    delete heatMapper_;
    heatMapper_ = new HeatMapper(&heatCanvas_, palette_, radius_px, 160,
                                 /*absoluteMode=*/true,
                                 /*cap01=*/true,
                                 /*useSaturCurve=*/true,
                                 /*saturK=*/1.5);


    if (!heatItem_) {
        heatItem_ = scene->addPixmap(QPixmap::fromImage(heatCanvas_));
        heatItem_->setZValue(5);
        heatItem_->setPos(0, 0);
    } else {
        heatItem_->setPixmap(QPixmap::fromImage(heatCanvas_));
    }

    if (!heatFlushTimer_) {
        heatFlushTimer_ = new QTimer(this);
        connect(heatFlushTimer_, &QTimer::timeout, this, &MainWindow::flushHeatmap);
        heatFlushTimer_->start(100);
    }
}


float MainWindow::rssiToIntensity01(float rssi) const
{
    // clamp [-90, -30]
    if (rssi < -90.f) rssi = -90.f;
    if (rssi > -30.f) rssi = -30.f;

    // -90 -> 0 (bad), -30 -> 1 (good)
    return (rssi + 90.f) / 60.f;
}

void MainWindow::onSample(double x, double y, double /*yaw*/, float rssi)
{
    if (!mapReady_ || !heatMapper_) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    const float intensity = rssiToIntensity01(rssi);
    heatMapper_->addPoint(px + 1, py + 1, intensity);
}

void MainWindow::flushHeatmap()
{
    if (!heatMapper_ || !heatItem_) return;

    heatMapper_->colorize();
    heatItem_->setPixmap(QPixmap::fromImage(heatCanvas_));
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

bool MainWindow::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == ui->graphicsView->viewport() &&
        ev->type() == QEvent::MouseButtonPress) {

        auto* me = static_cast<QMouseEvent*>(ev);
        const QPointF scenePos = ui->graphicsView->mapToScene(me->pos());

        // Mode 버튼 제거 후: Sim은 chkSimEnable 하나로 컨텍스트 결정
        const bool simCtx = ui->chkSimEnable->isChecked();
        const bool simGesture = (me->button() == Qt::RightButton) ||
                                (me->button() == Qt::LeftButton && (me->modifiers() & Qt::ShiftModifier));

        if (simCtx && simGesture) {
            int px=0, py=0;
            if (!sceneToMapPixel(scenePos, px, py)) return true;

            double x_m=0.0, y_m=0.0;
            if (!pixelToMap(px, py, x_m, y_m)) return true;

            addSimPinAt(x_m, y_m);
            return true;
        }

        // 기본 동작: Navigate
        onMapClicked(scenePos);
        return true;
    }
    return false;
}

void MainWindow::onMapClicked(const QPointF& scenePos)
{
    int px=0, py=0;
    if (!sceneToMapPixel(scenePos, px, py)) {
        qDebug() << "Clicked outside map";
        return;
    }

    double x_m=0.0, y_m=0.0;
    if (!pixelToMap(px, py, x_m, y_m)) {
        qDebug() << "pixelToMap failed";
        return;
    }

    rosThread->requestNavigateTo(x_m, y_m, 0.0);
    qDebug() << "NavigateTo:" << x_m << y_m;
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

//======================
// Layer toggles
//======================
void MainWindow::onLayerRobot(bool /*on*/)
{
    if (robotItem)
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);
}

void MainWindow::onLayerPins(bool on)
{
    for (auto* pin : apPins_) {
        if (pin) pin->setVisible(on);
    }
}

//======================
// AutoExplore (강화확습으로 대체)
//======================
void MainWindow::onAutoExploreToggled(bool on)
{
    autoExplorer_.setMapBounds(-3.0, 3.0, -3.0, 3.0);
    autoExplorer_.setEnabled(on);
    statusBar()->showMessage(QString("Auto Explore %1").arg(on ? "ON" : "OFF"));
}

void MainWindow::onGoalStatusMsg(const QString& msg)
{
    statusBar()->showMessage(msg);

    if (msg.contains("Sending goal") || msg.contains("Goal accepted")) {
        navBusy_ = true;
    }
    if (msg.contains("Goal SUCCEEDED") || msg.contains("Goal ABORTED") || msg.contains("Goal CANCELED")) {
        navBusy_ = false;
    }
}

void MainWindow::onAutoGoal(double x, double y, double yaw)
{
    if (navBusy_) return;
    if (!ui->chkAutoExplore->isChecked()) return;

    rosThread->requestNavigateTo(x, y, yaw);
    navBusy_ = true;
}

//======================
// Session
//======================
void MainWindow::onSessionRefresh()
{
    ui->cbSessionId->clear();

    const auto list = db_.listSessions();
    for (const auto& p : list) {
        const QString id = p.first;
        const QString at = p.second;
        ui->cbSessionId->addItem(id + " (" + at + ")", id);
    }

    statusBar()->showMessage(QString("Sessions: %1").arg(ui->cbSessionId->count()));
}

void MainWindow::onSessionLoad()
{
    const QString sid = ui->cbSessionId->currentData().toString();
    if (sid.isEmpty()) {
        statusBar()->showMessage("Load failed: no session selected");
        return;
    }
    currentLoadedSession_ = sid;
    reloadQueryLayer(sid);

    updateUiByContext();
}

void MainWindow::onSessionDelete()
{
    const QString sid = ui->cbSessionId->currentData().toString();
    if (sid.isEmpty()) return;

    db_.deleteSession(sid);

    queryLayer_.clear();
    queryLayer_.flush();
    queryLayer_.setVisible(false);

    if (currentLoadedSession_ == sid) currentLoadedSession_.clear();

    onSessionRefresh();
    updateUiByContext();
}

//======================
// Measure
//======================
void MainWindow::onMeasureStart()
{
    if (measuringDb_) return;

    const QString sid = db_.beginSession();
    if (sid.isEmpty()) {
        statusBar()->showMessage("Measure Start failed: cannot create session");
        return;
    }

    activeSessionId_ = sid;
    measuringDb_ = true;

    onSessionRefresh();

    // 방금 만든 세션 선택
    for (int i=0; i<ui->cbSessionId->count(); ++i) {
        if (ui->cbSessionId->itemData(i).toString() == sid) {
            ui->cbSessionId->setCurrentIndex(i);
            break;
        }
    }

    updateUiByContext();
}

void MainWindow::onMeasureStop()
{
    if (!measuringDb_) return;

    measuringDb_ = false;
    db_.endSession(activeSessionId_);
    activeSessionId_.clear();

    updateUiByContext();
}
//======================
// Filter
//======================
void MainWindow::onApplyFilter()
{
    filterSsid_ = ui->cbSsid->currentText();
    filterThrEnable_ = ui->chkThrEnable->isChecked();
    filterThrRssi_ = ui->cbThr->currentText().toInt();

    if (!currentLoadedSession_.isEmpty()) {
        reloadQueryLayer(currentLoadedSession_);
    }

    updateUiByContext();
}

void MainWindow::onSampleToDb(double x, double y, double /*yaw*/, float rssi)
{
    if (!measuringDb_) return;
    if (activeSessionId_.isEmpty()) return;
    if (filterThrEnable_ && rssi < filterThrRssi_) return;

    SampleRow s;
    s.x = x;
    s.y = y;
    s.ssid = filterSsid_.isEmpty() ? "ALL" : filterSsid_;
    s.rssi = rssi;
    s.ts = QDateTime::currentDateTime().toString(Qt::ISODate);

    db_.insertSample(activeSessionId_, s);
}

void MainWindow::reloadQueryLayer(const QString& sessionId)
{
    if (!mapReady_) return;
    if (!queryLayer_.isReady()) {
        const double radius_m = 0.25; // 25cm
        const int radius_px = qMax(1, int(std::round(radius_m / mapMeta_.resolution)));
        queryLayer_.init(scene, mapImageSize_, 6, radius_px, 160);
    }


    const auto rows = db_.loadSamples(sessionId, filterSsid_, filterThrEnable_, filterThrRssi_);

    queryLayer_.clear();
    for (const auto& r : rows) {
        int px=0, py=0;
        if (!meterToPixel(r.x, r.y, px, py)) continue;

        const float intensity = valueToIntensity01(r);
        queryLayer_.addPoint(px + 1, py + 1, intensity);
    }
    queryLayer_.flush();
    queryLayer_.setVisible(true);
}

//======================
// Sim
//======================
float MainWindow::simRssiAt(double d_m) const
{
    const double tx = ui->spTxPower->value(); // dBm (예: -40)
    const double rssi = tx - 20.0 * std::log10(d_m + 1.0);
    return static_cast<float>(rssi);
}

float MainWindow::simIntensityAt(double d_m) const
{
    return rssiToIntensity01(simRssiAt(d_m));
}

void MainWindow::addSimPinAt(double x_m, double y_m)
{
    simPins_.push_back({x_m, y_m});

    int px=0, py=0;
    if (meterToPixel(x_m, y_m, px, py)) {
        auto* pin = scene->addEllipse(-4, -4, 8, 8, QPen(Qt::blue), QBrush(Qt::blue));
        pin->setPos(QPointF(px, py));
        pin->setZValue(20);
        pin->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
        pin->setVisible(ui->chkShowApPins->isChecked());
        apPins_.push_back(pin);
    }

    rebuildSimLayer();
    updateUiByContext();
}

void MainWindow::rebuildSimLayer()
{
    if (!mapReady_) return;
    if (!simLayer_.isReady()) {
        const double radius_m = 0.25; // 25cm
        const int radius_px = qMax(1, int(std::round(radius_m / mapMeta_.resolution)));
        simLayer_.init(scene, mapImageSize_, 7, radius_px, 160);
    }

    simLayer_.clear();

    const double maxR_m = 6.0;
    const int rings = 10;
    const int angles = 24;

    for (const auto& pin : simPins_) {
        int cpx=0, cpy=0;
        if (meterToPixel(pin.x_m, pin.y_m, cpx, cpy)) {
            simLayer_.addPoint(cpx+1, cpy+1, simIntensityAt(0.0));
        }

        for (int ri=1; ri<=rings; ++ri) {
            const double r = maxR_m * (static_cast<double>(ri) / rings);
            const float inten = simValueToIntensity01(r);

            for (int ai=0; ai<angles; ++ai) {
                const double a = (2.0 * M_PI) * (static_cast<double>(ai) / angles);
                const double x = pin.x_m + r * std::cos(a);
                const double y = pin.y_m + r * std::sin(a);

                int px=0, py=0;
                if (!meterToPixel(x, y, px, py)) continue;
                simLayer_.addPoint(px+1, py+1, inten);
            }
        }
    }

    simLayer_.flush();
}

void MainWindow::onSimEnable(bool on)
{
    simEnabled_ = on;

    // ON이면: 핀이 있으면 재계산/표시, OFF면: sim layer 숨김
    if (on) {
        if (!simPins_.isEmpty()) rebuildSimLayer();
    } else {
        if (simLayer_.isReady()) simLayer_.setVisible(false);
    }

    updateUiByContext();
}

void MainWindow::onSimParamsChanged()
{
    simTxPower_ = ui->spTxPower->value();
    simChannel_ = ui->spSimChannel->value();
    simBandwidth_ = ui->cbSimBandwidth->currentText().toInt();

    if (ui->chkSimEnable->isChecked() && !simPins_.isEmpty()) {
        rebuildSimLayer();
    }

    updateUiByContext();
}

void MainWindow::onClearPins()
{
    for (auto* it : apPins_) {
        if (!it) continue;
        scene->removeItem(it);
        delete it;
    }
    apPins_.clear();

    simPins_.clear();

    simLayer_.clear();
    simLayer_.flush();
    simLayer_.setVisible(false);

    updateUiByContext();
}

//======================
// Metric
//======================
void MainWindow::onMetricChanged(int)
{
    const QString m = ui->cbMetric->currentText();

    if (m == "RSSI") currentMetric_ = Metric::RSSI;
    else if (m == "SNR") currentMetric_ = Metric::SNR;
    else if (m == "AP Count") currentMetric_ = Metric::ApCount;
    else if (m == "Interference/Noise") currentMetric_ = Metric::Noise;
    else if (m == "Throughput(Max)") currentMetric_ = Metric::ThroughputMax;
    else currentMetric_ = Metric::RSSI;

    // 현재 지원 상태 안내
    if (currentMetric_ != Metric::RSSI) {
        statusBar()->showMessage("Metric not implemented yet (only RSSI available).");
    }

    if (!currentLoadedSession_.isEmpty()) {
        reloadQueryLayer(currentLoadedSession_);
    }
    if (ui->chkSimEnable->isChecked() && !simPins_.isEmpty()) {
        rebuildSimLayer();
    }

    updateUiByContext();
}

float MainWindow::valueToIntensity01(const SampleRow& s) const
{
    switch (currentMetric_) {
    case Metric::RSSI:
        return rssiToIntensity01(s.rssi);
    case Metric::SNR:
    case Metric::ApCount:
    case Metric::Noise:
    case Metric::ThroughputMax:
    default:
        return rssiToIntensity01(s.rssi);
    }
}

float MainWindow::simValueToIntensity01(double d_m) const
{
    const float rssi = simRssiAt(d_m);

    switch (currentMetric_) {
    case Metric::RSSI:
    default:
        return rssiToIntensity01(rssi);
    }
}

//======================
// 히트맵 겹침 방지 + 컨텍스트 우선순위(Sim > Query > Live)
//======================
void MainWindow::applyLayersPolicy()
{
    const bool showHeat = ui->chkShowHeatmap->isChecked();

    // 모두 끄고 시작(겹침 방지)
    if (heatItem_) heatItem_->setVisible(false);
    if (queryLayer_.isReady()) queryLayer_.setVisible(false);
    if (simLayer_.isReady())   simLayer_.setVisible(false);

    if (!showHeat) return;

    const bool simCtx   = ui->chkSimEnable->isChecked();
    const bool queryCtx = !currentLoadedSession_.isEmpty();

    // 1) Sim
    if (simCtx && simEnabled_ && !simPins_.isEmpty() && simLayer_.isReady()) {
        simLayer_.setVisible(true);
        return;
    }

    // 2) Query
    if (queryCtx && queryLayer_.isReady()) {
        queryLayer_.setVisible(true);
        return;
    }

    // 3) Live
    if (heatItem_) heatItem_->setVisible(true);
}

void MainWindow::on_btnSessionLoad_clicked()
{
    // (Qt Designer auto-slot이 남아있으면 여기서 onSessionLoad로 연결해도 됨)
    onSessionLoad();
}

void MainWindow::onLayerHeatmap(bool /*on*/)
{
    // Mode 버튼 제거 후에는
    // 히트맵 on/off는 컨텍스트 정책으로 통합
    updateUiByContext();
}

void MainWindow::initLegendOverlay()
{
    if (!ui->graphicsView) return;
    if (legendOverlay_) return;

    // ✅ graphicsView의 viewport 위에 직접 올리면 오버레이처럼 동작
    legendOverlay_ = new LegendBarWidget(ui->graphicsView->viewport());
    legendOverlay_->setRangeDbm(-80, 0);
    legendOverlay_->setTitle("Voice + Data | dBm");
    legendOverlay_->setValueDbm(-67);

    // 보기 좋게 테두리/배경이 필요하면(선택)
    legendOverlay_->setAttribute(Qt::WA_TransparentForMouseEvents, true); // 클릭을 맵에 통과
    legendOverlay_->setStyleSheet("background: transparent;");           // 위젯 자체 배경은 paintEvent가 그림

    // 크기(첨부 이미지 느낌)
    legendOverlay_->setFixedSize(360, 64);

    updateLegendOverlayGeometry();
    legendOverlay_->show();
}

void MainWindow::updateLegendOverlayGeometry()
{
    if (!legendOverlay_ || !ui->graphicsView) return;

    const int margin = 12; // 우하단 여백
    const QRect vp = ui->graphicsView->viewport()->rect();

    const int w = legendOverlay_->width();
    const int h = legendOverlay_->height();

    const int x = vp.right() - w - margin;
    const int y = vp.bottom() - h - margin;

    legendOverlay_->move(x, y);
}

