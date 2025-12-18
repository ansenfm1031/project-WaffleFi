#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsPolygonItem>
#include <vector>
#include <cstdint>
#include <QGraphicsProxyWidget>
#include <QImage>
#include <QGraphicsPixmapItem>
#include <QTimer>
#include "rosworker.h"
#include "heatmapper.h"
#include "gradientpalette.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct MapMeta
{
    QString imagePath;
    double resolution = 0.05;  // m/px
    double origin_x = 0.0;     // m
    double origin_y = 0.0;     // m
    double origin_yaw = 0.0;   // rad (보통 0)
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onRosStatus(const QString &msg);
    void onRobotPose(double x, double y, double yaw);
    void onSample(double x, double y, double yaw, float rssi);
    void flushHeatmap();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    bool loadStaticMap(const QString& yamlPath);
    bool parseMapYaml(const QString& yamlPath, MapMeta& out);
    QPointF mapMeterToPixel(double x, double y) const;
    void applyViewTransform();
    void initHeatmap();
    bool meterToGrid(double x, double y, int& gx, int& gy) const;
    QColor rssiToColor(float rssi) const;
    // heatmap core
    QImage heatCanvas_;                 // HeatMapper가 칠할 캔버스(ARGB32)
    GradientPalette* palette_ = nullptr;
    HeatMapper* heatMapper_ = nullptr;

    // scene overlay
    QGraphicsPixmapItem* heatItem_ = nullptr;

    // flush timer (10~15Hz)
    QTimer* heatFlushTimer_ = nullptr;
    void initHeatmapLayer();            // 지도 로드 후 1회 호출
    bool meterToPixel(double x, double y, int& px, int& py) const;
    int  rssiToAddCount(float rssi) const;  // rssi -> addPoint 반복 횟수(강도)
    void refitView();
    void onMapClicked(const QPointF& scenePos);
    bool pixelToMap(int px, int py, double& x_m, double& y_m) const;
    bool eventFilter(QObject* obj, QEvent* ev);



private:
    Ui::MainWindow *ui;
    RosWorker *rosThread = nullptr;

    QGraphicsScene *scene = nullptr;
    QGraphicsPixmapItem *mapItem = nullptr;
    QGraphicsEllipseItem *robotItem = nullptr;
    QGraphicsPixmapItem* heatItem = nullptr;
    QImage heatImg;

    MapMeta mapMeta_;
    QSize mapImageSize_;


    std::vector<uint16_t> heatCount;
    std::vector<float> heatMean;
};

#endif // MAINWINDOW_H
