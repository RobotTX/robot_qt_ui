#ifndef SCANMAPLISTITEMWIDGET_H
#define SCANMAPLISTITEMWIDGET_H

class QGraphicsScene;
class QPushButton;
class QLabel;
class QLineEdit;
class ScanMapGraphicsItem;
class CustomPushButton;

#include <QWidget>
#include <QSlider>

class ScanMapListItemWidget : public QWidget {
    Q_OBJECT
public:
    ScanMapListItemWidget(int _id, QString name, QGraphicsScene *_scene);

    /// Setter
    void setId(const int _id){ id = _id; }

    /// Getters
    //ScanMapGraphicsItem* getPixmapItem(void) const { return pixmapItem; }
    //int getRotation(void) const { return slider->value(); }
    QString getRobotName(void) const { return robotName; }
    void robotConnected(bool connected);
    ScanMapGraphicsItem* getPixmapItem(void) const { return pixmapItem; }
    void robotScanning(bool scanning);
    void updateMap(QImage map);
    void updateRobotPos(double x, double y, double ori);

protected:
    void initializeMenu();

private slots:
    void sliderSlot(int value);
    void closeBtnSlot();
    void rotLineEditSlot(QString text);
    void scanningBtnSlot(bool checked);
    void robotGoToSlot(double x, double y);

signals:
    void deleteMap(int, QString);
    void playScan(bool, QString);
    void robotGoTo(QString, double, double);

private:
    int id;
    ScanMapGraphicsItem* pixmapItem;
    QString robotName;
    QPushButton* closeBtn;
    QSlider* slider;
    QLineEdit* rotLineEdit;
    QLabel* fileNameLabel;
    QGraphicsScene* scene;
    QPushButton* discoIcon;
    QPushButton* warningIcon;
    CustomPushButton* scanningBtn;
    QLabel* scanningLabel;

};


#endif // SCANMAPLISTITEMWIDGET_H
