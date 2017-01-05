#ifndef SCANMAPWIDGET_H
#define SCANMAPWIDGET_H

class QHBoxLayout;
class CustomQGraphicsView;
class MergeMapListWidget;

#include <QWidget>
#include <QGraphicsScene>
#include "Model/robots.h"
#include "Model/position.h"

#define LIST_WIDGET_HEIGHT 120

class ScanMapWidget : public QWidget {
    Q_OBJECT
public:
    ScanMapWidget(QSharedPointer<Robots> _robots, QWidget* parent = Q_NULLPTR);

protected:
    void initializeMenu();
    void initializeMap();
    void addMap(QString name);
    void closeEvent(QCloseEvent *event);
    void refreshIds();

private slots:
    void cancelSlot();
    void saveSlot();
    void addImageRobotSlot();
    void robotMenuSlot(QAction* action);
    void startedScanningSlot(QString robotName, bool scanning);
    void robotDisconnectedSlot(QString robotName);
    void deleteMapSlot(int id, QString robotName);
    void robotReconnectedSlot(QString robotName);
    void playScanSlot(bool scan, QString robotName);
    void robotScanningSlot(bool scan, QString robotName, bool success);
    void receivedScanMapSlot(QString robotName, QImage map);
    void robotGoToSlot(QString robotName, double x, double y);
    void scanRobotPosSlot(QString robotName, double x, double y, double ori);

signals:
    /**
     * @brief startScanning
     * To Start from the beggining a scan
     */
    void startScanning(QString);

    /**
     * @brief stopScanning
     * To completely stop a scan
     */
    void stopScanning(QStringList);

    /**
     * @brief playScan
     * To play/pause a scan that is already launched
     */
    void playScan(bool, QString);
    void robotGoTo(QString, double, double);

private:
    QSharedPointer<Robots> robots;
    QHBoxLayout* layout;
    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    MergeMapListWidget* listWidget;
};

#endif // SCANMAPWIDGET_H
