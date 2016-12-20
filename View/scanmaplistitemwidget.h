#ifndef SCANMAPLISTITEMWIDGET_H
#define SCANMAPLISTITEMWIDGET_H

class QGraphicsScene;
class QPushButton;
class QLabel;
class QLineEdit;
class MergeMapGraphicsItem;

#include <QWidget>
#include <QSlider>

class ScanMapListItemWidget : public QWidget {
    Q_OBJECT
public:
    ScanMapListItemWidget(int _id, QString fileName, QGraphicsScene *scene, bool _fromRobot);

    /// Setter
    void setId(const int _id){ id = _id; }

    /// Getters
    //ScanMapGraphicsItem* getPixmapItem(void) const { return pixmapItem; }
    int getRotation(void) const { return slider->value(); }

protected:
    void initializeMap(QString fileName, QGraphicsScene* scene);
    void initializeMenu(QString fileName);

private slots:
    /*void closeBtnSlot();
    void rotLineEditSlot(QString text);
    void sliderSlot(int value);
    void pixmapClickedSlot();*/

signals:
    /*void deleteMap(int);
    void gotOrigin(int);
    void pixmapClicked(int);*/

private:
    QPushButton* closeBtn;
    QSlider* slider;
    QLineEdit* rotLineEdit;
    int id;
    QLabel* fileNameLabel;
    bool fromRobot;
    MergeMapGraphicsItem* pixmapItem;
};

#endif // SCANMAPLISTITEMWIDGET_H
