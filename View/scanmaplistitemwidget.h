#ifndef SCANMAPLISTITEMWIDGET_H
#define SCANMAPLISTITEMWIDGET_H

class QGraphicsScene;
class QPushButton;
class QLabel;
class QLineEdit;
class ScanMapGraphicsItem;
class QGraphicsItem;

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
    QImage cropImage(QImage image);
    void mouseDoubleClickEvent(QMouseEvent *);

private slots:
    void sliderSlot(int value);
    void closeBtnSlot();
    void rotLineEditSlot(QString text);
    void scanningBtnSlot();
    void robotGoToSlot(double x, double y);

signals:
    void deleteMap(int, QString);
    void playScan(bool, QString);
    void robotGoTo(QString, double, double);
    void centerOn(QGraphicsItem*);

private:
    int id;
    QString robotName;
    QGraphicsScene* scene;
    ScanMapGraphicsItem* pixmapItem;
    QPushButton* closeBtn;
    QSlider* slider;
    QLineEdit* rotLineEdit;
    QLabel* fileNameLabel;
    QPushButton* discoIcon;
    QPushButton* warningIcon;
    QPushButton* scanningBtn;
    QLabel* scanningLabel;
    int oriWidth;
    int oriHeight;
    int newWidth;
    int newHeight;
    int top;
    int left;
};


#endif // SCANMAPLISTITEMWIDGET_H
