#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QPixmap>
#include <QDebug>
#include <cmath>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QShowEvent>
#include <QDateTime>
#include <QTimer>
#include <QSet>

#include <sqlite3.h>

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // =======================
    // Left panel tab behavior
    // =======================
    ui->leftPanel_2->setVisible(false);
    ui->leftStack->setCurrentWidget(ui->pageSession);

    auto showPanel = [&](QWidget* page) {
        ui->leftPanel_2->setVisible(true);
        ui->leftStack->setCurrentWidget(page);
    };
    auto hidePanel = [&]() {
        ui->leftPanel_2->setVisible(false);
    };

    auto initTab = [&](QToolButton* tabBtn, QWidget* pageWidget) {
        if (!tabBtn || !pageWidget) return;

        tabBtn->setCheckable(true);
        tabBtn->setAutoExclusive(false);

        connect(tabBtn, &QToolButton::clicked, this, [=]() {
            const bool panelVisible = ui->leftPanel_2->isVisible();
            const bool samePage = (ui->leftStack->currentWidget() == pageWidget);

            if (panelVisible && samePage) {
                hidePanel();
                tabBtn->setChecked(false);
                return;
            }

            showPanel(pageWidget);

            // manual exclusive
            if (ui->tbSession) ui->tbSession->setChecked(tabBtn == ui->tbSession);
            if (ui->tbSim)     ui->tbSim->setChecked(tabBtn == ui->tbSim);
            if (ui->tbLayers)  ui->tbLayers->setChecked(tabBtn == ui->tbLayers);
        });
    };

    initTab(ui->tbSession, ui->pageSession);
    initTab(ui->tbSim,     ui->pageSim);
    initTab(ui->tbLayers,  ui->pageLayers);

    if (ui->tbSession) ui->tbSession->setChecked(false);
    if (ui->tbSim)     ui->tbSim->setChecked(false);
    if (ui->tbLayers)  ui->tbLayers->setChecked(false);

    // Admin은 DB 직접 접근 금지 → 비활성화
    if (ui->btnAdmin) {
        ui->btnAdmin->setEnabled(false);
        connect(ui->btnAdmin, &QAbstractButton::clicked, this, &MainWindow::onAdminClicked);
    }

    // =======================
    // Graphics scene
    // =======================
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

    // =======================
    // Live heatmap init
    // =======================
    initHeatmapLayer();

    // =======================
    // Sim layer init
    // =======================
    if (!simLayer_.isReady() && mapReady_) {
        simLayer_.init(scene, mapImageSize_, 7, simBrushRadiusPx(), 160);
        simLayer_.setVisible(false);
    }

    // =======================
    // Query layer init (스냅샷 Load 결과 표시용)
    // =======================
    if (mapReady_) {
        if (!queryLayer_.isReady()) {
            queryLayer_.init(scene, mapImageSize_, 6, radiusPxLive(), 160);
            queryLayer_.setVisible(false);
        }
    }

    // =======================
    // ROS worker thread
    // =======================
    rosThread = new RosWorker(this);
    connect(rosThread, &RosWorker::statusChanged, this, &MainWindow::onRosStatus);
    connect(rosThread, &RosWorker::robotPose,     this, &MainWindow::onRobotPose);

    // dummy sample은 test모드가 아니면 불필요 (남겨도 무방)
    connect(rosThread, &RosWorker::sample,        this, &MainWindow::onSample);

    // Live fused (진짜 데이터)
    connect(rosThread, &RosWorker::fusedSample,   this, &MainWindow::onFusedSample);

    // Robot marker (gridPose는 현재 px로 처리 중이므로 쓰지 않으면 연결 빼도 됨)
    connect(rosThread, &RosWorker::gridPose,      this, &MainWindow::onGridPose);
    connect(rosThread, &RosWorker::rlStateArrived,this, &MainWindow::onRlStateArrived);

    rosThread->start();

    // =======================
    // Layer toggles
    // =======================
    connect(ui->chkShowHeatmap, &QCheckBox::toggled, this, &MainWindow::onLayerHeatmap);
    connect(ui->chkShowRobot,   &QCheckBox::toggled, this, &MainWindow::onLayerRobot);
    connect(ui->chkShowApPins,  &QCheckBox::toggled, this, &MainWindow::onLayerPins);

    // Auto Explore
    connect(ui->chkAutoExplore, &QCheckBox::toggled, this, &MainWindow::onAutoExploreToggled);

    // =======================
    // Session (Snapshot)
    // =======================
    connect(ui->btnSessionRefresh, &QPushButton::clicked, this, &MainWindow::onSessionRefresh);
    // btnSessionLoad는 오토슬롯 on_btnSessionLoad_clicked로만 진입
    connect(ui->btnSessionDelete,  &QPushButton::clicked, this, &MainWindow::onSessionDelete);

    // Query Clear
    if (ui->btnQueryClear) {
        connect(ui->btnQueryClear, &QPushButton::clicked, this, &MainWindow::onQueryClear);
    }

    // =======================
    // Measure: Start/Stop = live 누적 ON/OFF (+ Start 시 snapshot 자동 생성)
    // =======================
    if (ui->btnMeasureStart) {
        ui->btnMeasureStart->setCheckable(false);
        ui->btnMeasureStart->setText("Start");
        connect(ui->btnMeasureStart, &QPushButton::clicked, this, [this]() {
            if (!accumulating_) onMeasureStart();
            else                onMeasureStop();

            updateUiByContext();

            QTimer::singleShot(0, this, [this]{
                ui->btnMeasureStart->setDown(false);
                ui->btnMeasureStart->clearFocus();
                ui->btnMeasureStart->repaint();
            });
        });
    }

    // Filter apply
    connect(ui->btnApplyFilter, &QPushButton::clicked, this, &MainWindow::onApplyFilter);

    // =======================
    // Sim controls
    // =======================
    connect(ui->chkSimEnable, &QCheckBox::toggled, this, &MainWindow::onSimEnable);
    connect(ui->spTxPower,    QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onSimParamsChanged);
    connect(ui->spSimChannel, QOverload<int>::of(&QSpinBox::valueChanged),          this, &MainWindow::onSimParamsChanged);
    connect(ui->cbSimBandwidth, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onSimParamsChanged);
    connect(ui->btnClearPins, &QPushButton::clicked, this, &MainWindow::onClearPins);

    // AutoExplorer integration
    connect(rosThread, &RosWorker::goalStatus, &autoExplorer_, &AutoExplorer::onGoalStatus);
    connect(&autoExplorer_, &AutoExplorer::newGoal, this, [this](double x, double y, double yaw){
        if (navBusy_) return;
        if (!ui->chkAutoExplore->isChecked()) return;
        rosThread->requestNavigateTo(x, y, yaw);
        navBusy_ = true;
    });

    // ===== init =====
    // SSID 콤보 초기화(ALL 보장)
    if (ui->cbSsid) {
        ui->cbSsid->clear();
        ui->cbSsid->addItem("ALL");
        ssidSetLive_.clear();
        ssidSetLive_.insert("ALL");
    }

    // Snapshot 목록 초기 로드
    onSessionRefresh();
    onSimParamsChanged();
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

// ======================
// Context-based Mode
// ======================
MainWindow::Mode MainWindow::deriveModeFromContext() const
{
    if (ui->chkSimEnable->isChecked())
        return Mode::SimAP;

    if (!loadedSnapshotPath_.isEmpty())
        return Mode::Query;

    if (accumulating_)
        return Mode::Measurement;

    return Mode::View;
}

void MainWindow::updateUiByContext()
{
    currentMode_ = deriveModeFromContext();

    // Start/Stop button
    if (ui->btnMeasureStart) {
        ui->btnMeasureStart->setEnabled(true);
        ui->btnMeasureStart->setText(accumulating_ ? "Stop" : "Start");
    }

    // Sim group enable/disable
    simEnabled_ = ui->chkSimEnable->isChecked();
    if (ui->gbSim) ui->gbSim->setEnabled(true);
    if (ui->chkSimEnable) ui->chkSimEnable->setEnabled(true);

    if (ui->spTxPower)      ui->spTxPower->setEnabled(simEnabled_);
    if (ui->spSimChannel)   ui->spSimChannel->setEnabled(simEnabled_);
    if (ui->cbSimBandwidth) ui->cbSimBandwidth->setEnabled(simEnabled_);
    if (ui->btnClearPins)   ui->btnClearPins->setEnabled(simEnabled_);

    // Robot visibility
    if (robotItem)
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);

    // Layer policy
    applyLayersPolicy();

    // Status
    if (simEnabled_) {
        statusBar()->showMessage("Sim: Shift+Click or Right Click to drop AP pins");
    } else if (!loadedSnapshotPath_.isEmpty()) {
        statusBar()->showMessage("Query: loaded snapshot. Use Clear to remove.");
    } else if (accumulating_) {
        statusBar()->showMessage("Measuring: live heatmap accumulating.");
    } else {
        statusBar()->showMessage("View: live heatmap.");
    }
}

int MainWindow::radiusPx() const
{
    const double radius_m = 0.25;
    return qMax(1, int(std::round(radius_m / mapMeta_.resolution)));
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

    v->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
    v->setResizeAnchor(QGraphicsView::AnchorViewCenter);
    v->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->setRenderHint(QPainter::SmoothPixmapTransform, true);

    v->resetTransform();
    v->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

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

    if (!poseReady_) {
        poseReady_ = true;
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);
    }
}

// ======================
// Live heatmap
// ======================
void MainWindow::initHeatmapLayer()
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !scene) return;

    heatCanvas_ = QImage(mapImageSize_, QImage::Format_ARGB32);
    heatCanvas_.fill(Qt::transparent);

    delete palette_;
    palette_ = new GradientPalette(256);
    palette_->setColorAt(0.0, QColor(255, 0, 0));
    palette_->setColorAt(0.5, QColor(255, 255, 0));
    palette_->setColorAt(1.0, QColor(0, 255, 0));

    delete heatMapper_;
    heatMapper_ = new HeatMapper(&heatCanvas_, palette_, radiusPx(), 160, true, true);

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

// 색 3단계 양자화
float MainWindow::rssiToIntensity01(float rssi) const
{
    if (rssi >= -50.f) return 1.0f;
    if (rssi >= -70.f) return 0.5f;
    return 0.0f;
}

void MainWindow::onSample(double x, double y, double /*yaw*/, float rssi)
{
    // (테스트용 dummy sample)
    if (!mapReady_ || !heatMapper_) return;
    if (!accumulating_) return; // live 누적 정책 동일 적용

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    heatMapper_->addPoint(px + 1, py + 1, rssiToIntensity01(rssi));
}

void MainWindow::flushHeatmap()
{
    if (!heatMapper_ || !heatItem_) return;
    heatMapper_->colorize();
    heatItem_->setPixmap(QPixmap::fromImage(heatCanvas_));
}

// ======================
// Click handling
// ======================
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

        const bool simCtx = ui->chkSimEnable->isChecked();
        const bool simGesture =
            (me->button() == Qt::RightButton) ||
            (me->button() == Qt::LeftButton && (me->modifiers() & Qt::ShiftModifier));

        if (simCtx && simGesture) {
            int px=0, py=0;
            if (!sceneToMapPixel(scenePos, px, py)) return true;

            double x_m=0.0, y_m=0.0;
            if (!pixelToMap(px, py, x_m, y_m)) return true;

            addSimPinAt(x_m, y_m);
            return true;
        }

        onMapClicked(scenePos);
        return true;
    }
    return false;
}

void MainWindow::onMapClicked(const QPointF& scenePos)
{
    int px=0, py=0;
    if (!sceneToMapPixel(scenePos, px, py)) return;

    double x_m=0.0, y_m=0.0;
    if (!pixelToMap(px, py, x_m, y_m)) return;

    rosThread->requestNavigateTo(x_m, y_m, 0.0);
}

// ======================
// Layer toggles
// ======================
void MainWindow::onLayerHeatmap(bool)
{
    updateUiByContext();
}

void MainWindow::onLayerRobot(bool)
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

// ======================
// AutoExplore
// ======================
void MainWindow::onAutoExploreToggled(bool on)
{
    autoExplorer_.setMapBounds(-3.0, 3.0, -3.0, 3.0);
    autoExplorer_.setEnabled(on);
    statusBar()->showMessage(QString("Auto Explore %1").arg(on ? "ON" : "OFF"));
}

// ======================
// Snapshot helpers
// ======================
QString MainWindow::makeDbSnapshot()
{
    QDir().mkpath(snapshotDir_);

    if (!QFile::exists(sourceDbPath_)) {
        statusBar()->showMessage("Snapshot failed: source DB not found");
        return {};
    }

    const QString dst = snapshotDir_ + "/snap_" +
                        QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".db";

    if (QFile::exists(dst)) QFile::remove(dst);

    if (!QFile::copy(sourceDbPath_, dst)) {
        statusBar()->showMessage("Snapshot failed: copy error");
        return {};
    }

    return dst;
}

void MainWindow::updateSsidComboLive(const QString& ssid)
{
    if (!ui->cbSsid) return;
    if (ssid.isEmpty()) return;
    if (ssidSetLive_.contains(ssid)) return;

    ssidSetLive_.insert(ssid);
    ui->cbSsid->addItem(ssid);
}

// ======================
// Sessions (Snapshot list)
// ======================
void MainWindow::onSessionRefresh()
{
    ui->cbSessionId->clear();

    QDir dir(snapshotDir_);
    if (!dir.exists()) {
        ui->cbSessionId->addItem("(no snapshots yet)");
        statusBar()->showMessage("Snapshots: 0");
        return;
    }

    QStringList files = dir.entryList(QStringList() << "*.db", QDir::Files, QDir::Time);
    if (files.isEmpty()) {
        ui->cbSessionId->addItem("(no snapshots yet)");
        statusBar()->showMessage("Snapshots: 0");
        return;
    }

    for (const auto& f : files) {
        const QString full = dir.absoluteFilePath(f);
        ui->cbSessionId->addItem(f, full); // userData = full path
    }

    statusBar()->showMessage(QString("Snapshots: %1").arg(ui->cbSessionId->count()));
}

void MainWindow::onSessionLoad()
{
    const QString snapPath = ui->cbSessionId->currentData().toString();
    if (snapPath.isEmpty() || !QFile::exists(snapPath)) {
        statusBar()->showMessage("Load failed: snapshot not found");
        return;
    }

    loadedSnapshotPath_ = snapPath;
    reloadQueryLayerFromSnapshot(snapPath);
    updateUiByContext();
}

void MainWindow::onSessionDelete()
{
    const QString snapPath = ui->cbSessionId->currentData().toString();
    if (snapPath.isEmpty()) return;

    if (loadedSnapshotPath_ == snapPath) {
        onQueryClear();
        loadedSnapshotPath_.clear();
    }

    QFile::remove(snapPath);
    onSessionRefresh();
    updateUiByContext();
}

// 오토슬롯
void MainWindow::on_btnSessionLoad_clicked()
{
    onSessionLoad();
}

void MainWindow::onQueryClear()
{
    if (queryLayer_.isReady()) {
        queryLayer_.clear(true);
        queryLayer_.flush();
        queryLayer_.setVisible(false);
    }
    loadedSnapshotPath_.clear();
    updateUiByContext();
}

// ======================
// Measure: Start/Stop (Live 누적 + Start 시 snapshot 생성)
// ======================
void MainWindow::onMeasureStart()
{
    if (accumulating_) return;
    accumulating_ = true;

    // Start 시점 스냅샷 1개 자동 생성(원본 DB는 절대 open하지 않음)
    const QString snap = makeDbSnapshot();
    if (!snap.isEmpty()) {
        onSessionRefresh();
        for (int i = 0; i < ui->cbSessionId->count(); ++i) {
            if (ui->cbSessionId->itemData(i).toString() == snap) {
                ui->cbSessionId->setCurrentIndex(i);
                break;
            }
        }
    }

    updateUiByContext();
}

void MainWindow::onMeasureStop()
{
    if (!accumulating_) return;
    accumulating_ = false;
    updateUiByContext();
}

// ======================
// Filter
// ======================
void MainWindow::onApplyFilter()
{
    filterSsid_ = ui->cbSsid ? ui->cbSsid->currentText() : "ALL";

    // chkThrEnable은 UI에서 숨김/비활성이라 사실상 항상 true로 쓰는 게 UX상 자연스럽습니다.
    filterThrEnable_ = true;

    // "-70 dBm" -> toInt() 하면 -70 파싱됨
    filterThrRssi_ = ui->cbThr ? ui->cbThr->currentText().toInt() : -70;

    if (!loadedSnapshotPath_.isEmpty()) {
        reloadQueryLayerFromSnapshot(loadedSnapshotPath_);
    }
    updateUiByContext();
}

// ======================
// Query (Snapshot DB SELECT only)
// ======================
void MainWindow::reloadQueryLayerFromSnapshot(const QString& snapshotPath)
{
    if (!mapReady_) return;

    // 겹침/착시 방지: query 갱신 중 live 잠시 끄기
    if (heatItem_) heatItem_->setVisible(false);

    if (queryLayer_.isReady()) {
        queryLayer_.clear(true);
        queryLayer_.flush();
        queryLayer_.setVisible(false);
    }

    sqlite3* db = nullptr;
    if (sqlite3_open_v2(snapshotPath.toStdString().c_str(),
                        &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
        statusBar()->showMessage("Snapshot open failed");
        if (db) sqlite3_close(db);
        return;
    }

    QString sql = "SELECT ssid, grid_id_x, grid_id_y, rssi_value FROM wifi_data WHERE 1=1";
    if (!filterSsid_.isEmpty() && filterSsid_ != "ALL") {
        sql += " AND ssid = ?";
    }
    if (filterThrEnable_) {
        sql += " AND rssi_value >= ?";
    }
    sql += " ORDER BY id ASC;";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql.toStdString().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        statusBar()->showMessage("Snapshot query prepare failed");
        sqlite3_close(db);
        return;
    }

    int idx = 1;
    if (!filterSsid_.isEmpty() && filterSsid_ != "ALL") {
        sqlite3_bind_text(stmt, idx++, filterSsid_.toStdString().c_str(), -1, SQLITE_TRANSIENT);
    }
    if (filterThrEnable_) {
        sqlite3_bind_double(stmt, idx++, (double)filterThrRssi_);
    }

    int count = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        const double x_m  = sqlite3_column_double(stmt, 1);
        const double y_m  = sqlite3_column_double(stmt, 2);
        const double rssi = sqlite3_column_double(stmt, 3);

        int px=0, py=0;
        if (!meterToPixel(x_m, y_m, px, py)) continue;

        queryLayer_.addPoint(px + 1, py + 1, rssiToIntensity01((float)rssi));
        ++count;
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);

    queryLayer_.flush();
    queryLayer_.setVisible(true);

    statusBar()->showMessage(QString("Loaded %1 rows from snapshot").arg(count));
}

// ======================
// Sim
// ======================
float MainWindow::simRssiAt(double d_m) const
{
    const double tx = simTxPower_;
    const double rssi = tx - 20.0 * std::log10(d_m + 1.0);
    return static_cast<float>(rssi);
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
    if (!mapReady_ || !simLayer_.isReady()) return;

    simLayer_.clear(true);

    for (const auto& pin : simPins_) {
        int px=0, py=0;
        if (!meterToPixel(pin.x_m, pin.y_m, px, py)) continue;

        const float inten = rssiToIntensity01(static_cast<float>(simTxPower_));
        simLayer_.addPoint(px + 1, py + 1, inten);
    }

    simLayer_.flush();
}

void MainWindow::onSimEnable(bool on)
{
    simEnabled_ = on;

    if (!on) {
        if (simLayer_.isReady()) simLayer_.setVisible(false);
        updateUiByContext();
        return;
    }

    if (!simPins_.isEmpty()) rebuildSimLayer();
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

    if (simLayer_.isReady()) {
        simLayer_.clear(true);
        simLayer_.flush();
        simLayer_.setVisible(false);
    }

    updateUiByContext();
}

// Layer policy: Sim > Query > Live
void MainWindow::applyLayersPolicy()
{
    const bool showHeat = ui->chkShowHeatmap->isChecked();

    if (heatItem_) heatItem_->setVisible(false);
    if (queryLayer_.isReady()) queryLayer_.setVisible(false);
    if (simLayer_.isReady())   simLayer_.setVisible(false);

    if (!showHeat) return;

    const bool simCtx   = ui->chkSimEnable->isChecked();
    const bool queryCtx = !loadedSnapshotPath_.isEmpty();

    if (simCtx && simEnabled_ && !simPins_.isEmpty() && simLayer_.isReady()) {
        simLayer_.setVisible(true);
        return;
    }

    if (queryCtx && queryLayer_.isReady()) {
        queryLayer_.setVisible(true);
        return;
    }

    if (heatItem_) heatItem_->setVisible(true);
}

// ======================
// Legend overlay
// ======================
void MainWindow::initLegendOverlay()
{
    if (!ui->graphicsView) return;
    if (legendOverlay_) return;

    legendOverlay_ = new LegendBarWidget(ui->graphicsView->viewport());
    legendOverlay_->setRangeDbm(-80, 0);
    legendOverlay_->setTitle("Voice + Data | dBm");
    legendOverlay_->setValueDbm(-67);

    legendOverlay_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    legendOverlay_->setStyleSheet("background: transparent;");
    legendOverlay_->setFixedSize(360, 64);

    updateLegendOverlayGeometry();
    legendOverlay_->show();
}

void MainWindow::updateLegendOverlayGeometry()
{
    if (!legendOverlay_ || !ui->graphicsView) return;

    const int margin = 12;
    const QRect vp = ui->graphicsView->viewport()->rect();

    const int w = legendOverlay_->width();
    const int h = legendOverlay_->height();

    const int x = vp.right() - w - margin;
    const int y = vp.bottom() - h - margin;

    legendOverlay_->move(x, y);
}

// 25cm live 반경
int MainWindow::radiusPxLive() const
{
    const double r_m = 0.25;
    return qMax(1, int(std::round(r_m / mapMeta_.resolution)));
}

int MainWindow::simBrushRadiusPx() const
{
    int rpx = 10;
    switch (simBandwidth_) {
    case 20:  rpx = 6;  break;
    case 40:  rpx = 8;  break;
    case 80:  rpx = 10; break;
    case 160: rpx = 12; break;
    default:  rpx = 10; break;
    }
    return qMax(6, rpx);
}

void MainWindow::onSimParamsChanged()
{
    simTxPower_   = ui->spTxPower->value();
    simChannel_   = ui->spSimChannel->value();
    simBandwidth_ = ui->cbSimBandwidth->currentText().toInt();

    if (simLayer_.isReady()) {
        simLayer_.resetBrush(simBrushRadiusPx(), 200);
        rebuildSimLayer();
    }

    if (ui->chkSimEnable->isChecked() && !simPins_.isEmpty()) {
        rebuildSimLayer();
    }

    updateUiByContext();
}

void MainWindow::onAdminClicked()
{
    statusBar()->showMessage("Admin disabled: Qt does not open DB directly.");
}

// ======================
// Live fused sample
// ======================
void MainWindow::onFusedSample(double x_m, double y_m, const QString& ssid, int rssi)
{
    if (!mapReady_ || !heatMapper_ || mapImageSize_.isEmpty()) return;

    // SSID 리스트 실시간 갱신
    updateSsidComboLive(ssid);

    // Start/Stop 정책: Start(누적 ON)에서만 live heatmap 누적
    if (!accumulating_) return;

    // 필터 적용 (Live에도 동일 적용)
    if (!filterSsid_.isEmpty() && filterSsid_ != "ALL" && ssid != filterSsid_) return;
    if (filterThrEnable_ && rssi < filterThrRssi_) return;

    int px=0, py=0;
    if (!meterToPixel(x_m, y_m, px, py)) return;

    heatMapper_->addPoint(px + 1, py + 1, rssiToIntensity01((float)rssi));
}

// gridPose는 현재 구현이 "픽셀 좌표" 가정이라, 실제로 쓰지 않는다면 호출 안 해도 됩니다.
void MainWindow::onGridPose(double gx, double gy)
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !robotItem) return;

    int px = (int)std::lround(gx);
    int gy_i = (int)std::lround(gy);
    int py = (mapImageSize_.height() - 1) - gy_i;

    if (px < 0 || py < 0 || px >= mapImageSize_.width() || py >= mapImageSize_.height())
        return;

    robotItem->setPos(QPointF(px, py));
    poseReady_ = true;
    robotItem->setVisible(ui->chkShowRobot->isChecked());
}

void MainWindow::onRlStateArrived()
{
    statusBar()->showMessage("RL state arrived");
}
