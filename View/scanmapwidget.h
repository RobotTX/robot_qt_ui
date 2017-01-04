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

private slots:
    void cancelSlot();
    void saveSlot();
    void addImageRobotSlot();
    void robotMenuSlot(QAction* action);
    void startedScanningSlot(QString robotName, bool scanning);

signals:
    void startScanning(QString);

private:
    QSharedPointer<Robots> robots;
    QHBoxLayout* layout;
    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    MergeMapListWidget* listWidget;
};

#endif // SCANMAPWIDGET_H
