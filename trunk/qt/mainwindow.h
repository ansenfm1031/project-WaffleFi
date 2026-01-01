#pragma once

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QTimer>
#include <QImage>
#include <QResizeEvent>
#include <QShowEvent>
#include <QMouseEvent>
#include <QSet>


#include "rosworker.h"
#include "legendbarwidget.h"

// QHeatMap(HeatMapper 방식)
#include "heatmapper.h"
#include "gradientpalette.h"

// HeatLayer를 이미 쓰고 있다면 include 유지
#include "heatlayer.h"
#include "autoexplorer.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct MapMeta {
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
    QString imagePath;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent=nullptr);
    ~MainWindow();

protected:
    void resizeEvent(QResizeEvent* e) override;
    void showEvent(QShowEvent* e) override;
    bool eventFilter(QObject* obj, QEvent* ev) override;

private slots:
    void onRosStatus(const QString& msg);
    void onRobotPose(double x, double y, double yaw);

    // live fused sample -> live heatmap
    void onFusedSample(double x_m, double y_m, const QString& ssid, int rssi);

    // past heatmap service reply -> query heatmap
    void onHeatmapReplyArrived(bool ok,
                               const QString& message,
                               const QString& session_id,
                               const QString& ssid,
                               bool thr_enable,
                               int thr_rssi,
                               const QVector<double>& xs,
                               const QVector<double>& ys,
                               const QVector<int>& rssis,
                               const QVector<QString>& ssids,
                               const QVector<QString>& stamps);

    // UI actions
    void on_btnSessionLoad_clicked();   // autoconnect
    void onSessionRefresh();            // (여기서는 서비스 기반으로 간단히)
    void onQueryClear();
    void onApplyFilter();
    void onLayerHeatmap(bool);
    void onLayerRobot(bool);

private:
    // ===== map load =====
    bool loadStaticMap(const QString& yamlPath);
    bool parseMapYaml(const QString& yamlPath, MapMeta& out);

    // ===== view =====
    void applyViewTransform();

    // ===== coord transforms =====
    bool meterToPixel(double x, double y, int& px, int& py) const;
    bool pixelToMap(int px, int py, double& x_m, double& y_m) const;
    bool sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const;

    // ===== heatmaps =====
    int brushRadiusPx() const;
    float rssiToIntensity01(float rssi) const;

    void initLiveHeatmap();
    void initQueryHeatmap();
    void flushLiveHeatmap();
    void flushQueryHeatmap();
    void clearQueryHeatmap();

    // query(service) request helper
    void requestPastHeatmap();

    // policy: Query 우선(세션 로드 중) / 아니면 Live
    void applyLayersPolicy();

private:
    Ui::MainWindow* ui = nullptr;

    RosWorker* rosThread = nullptr;

    QGraphicsScene* scene = nullptr;
    QGraphicsPixmapItem* mapItem = nullptr;
    QGraphicsPixmapItem* liveHeatItem = nullptr;
    QGraphicsPixmapItem* queryHeatItem = nullptr;
    QGraphicsEllipseItem* robotItem = nullptr;

    bool mapReady_ = false;
    bool poseReady_ = false;

    MapMeta mapMeta_;
    QSize mapImageSize_;

    // live heatmap
    QImage liveCanvas_;
    GradientPalette* livePalette_ = nullptr;
    HeatMapper* liveMapper_ = nullptr;
    QTimer* liveFlushTimer_ = nullptr;

    // query heatmap
    QImage queryCanvas_;
    GradientPalette* queryPalette_ = nullptr;
    HeatMapper* queryMapper_ = nullptr;

    // UI state
    bool accumulating_ = true;          // Start/Stop 정책(원하면 연결)
    QString currentSessionId_;          // “과거” 모드 진입 여부
    QString filterSsid_ = "ALL";
    bool filterThrEnable_ = false;
    int filterThrRssi_ = -70;

    uint32_t queryLimit_ = 200000;
    uint32_t queryOffset_ = 0;
};
