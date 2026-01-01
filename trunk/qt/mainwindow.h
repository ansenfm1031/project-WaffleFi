#pragma once

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QImage>
#include <QTimer>
#include <QPointF>
#include <QSet>
#include <QString>
#include <QList>

#include "heatlayer.h"

// forward
namespace Ui { class MainWindow; }
class RosWorker;
class LegendBarWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* e) override;
    bool eventFilter(QObject* obj, QEvent* ev) override;

private slots:
    void onRosStatus(const QString &msg);

    void onRobotPose(double x, double y, double yaw);
    void onFusedSample(double x_m, double y_m, const QString& ssid, int rssi);

    void onMeasureToggle();
    void onStartSessionReply(bool ok, const QString& session_id_or_msg);
    void onStopSessionReply(bool ok, const QString& session_id_or_msg);

    void on_btnSessionLoad_clicked();   // auto-slot (btnSessionLoad)
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

    void onQueryClear();

    void onApplyFilter();

    void onLayerHeatmap(bool);
    void onLayerRobot(bool);
    void onLayerPins(bool);
    void onSessionRefresh();
    void onSessionComboChanged(int index);

    void onListSessionsReply(bool ok,
                             const QString& message,
                             const QList<QString>& session_ids,
                             const QList<QString>& started_at,
                             const QList<QString>& ended_at);

    void onListSsidsReply(bool ok,
                          const QString& message,
                          const QString& session_id,
                          const QList<QString>& ssids);
    QString currentSessionIdText() const;

private:
    double viewRotateDeg_ = -180.0;
    bool simEnable_ = false;
    double simTxPower_ = -40.0;
    int simChannel_ = 36;
    int simBandDiameterPx_ = 20;

    // ===== Types =====
    struct MapMeta {
        QString imagePath;
        double resolution = 0.05;
        double origin_x = 0.0;
        double origin_y = 0.0;
        double origin_yaw = 0.0;
    };

    enum class Mode {
        View,
        Measurement,
        Query
    };

    // ===== Setup helpers =====
    void initGraphics();
    bool loadStaticMap(const QString &yamlPath);
    bool parseMapYaml(const QString &yamlPath, MapMeta &out);

    void applyViewTransform();

    bool meterToPixel(double x, double y, int& px, int& py) const;
    bool pixelToMap(int px, int py, double& x_m, double& y_m) const;
    bool sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const;

    // ===== Heatmap helpers =====
    void initHeatLayers();              // live/query/sim 레이어 init
    float rssiToIntensity01(float rssi) const;

    void updateSsidComboLive(const QString& ssid);
    void addLivePointMean(double x_m, double y_m, int rssi);

    // ===== UI / policy =====
    Mode deriveModeFromContext() const;
    void updateUiByContext();
    void applyLayersPolicy();

    // ===== Legend overlay =====
    void initLegendOverlay();
    void updateLegendOverlayGeometry();

private:
    Ui::MainWindow *ui = nullptr;

    // Graphics
    QGraphicsScene* scene = nullptr;
    QGraphicsPixmapItem* mapItem = nullptr;
    QGraphicsEllipseItem* robotItem = nullptr;

    bool mapReady_ = false;
    bool poseReady_ = false;

    MapMeta mapMeta_;
    QSize mapImageSize_;

    // ROS worker thread
    RosWorker* rosThread = nullptr;

    // Layers ( live도 HeatLayer로 통일)
    HeatLayer liveLayer_;
    HeatLayer queryLayer_;
    HeatLayer simLayer_;

    QTimer* heatFlushTimer_ = nullptr;

    // Session / Query state
    bool accumulating_ = false;
    bool sessionPending_ = false;

    QString activeSessionId_;
    QString loadedSessionId_;

    // Live SSID list
    QSet<QString> ssidSetLive_;

    // Filter state
    QString filterSsid_ = "ALL";
    bool filterThrEnable_ = false;
    int  filterThrRssi_ = -70;

    // Legend
    LegendBarWidget* legendOverlay_ = nullptr;
    QTimer* legendPosTimer_ = nullptr;

    bool startOptimistic_ = false;

    // "즉시 텍스트 변경"을 했을 때, 실패 시 롤백하기 위한 플래그
    bool pendingStartUi_   = false;
    bool pendingStopUi_    = false;

};
