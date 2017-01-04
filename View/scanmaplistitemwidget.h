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
    ScanMapListItemWidget(int _id, QString name, QGraphicsScene *scene);

    /// Setter
    void setId(const int _id){ id = _id; }

    /// Getters
    //ScanMapGraphicsItem* getPixmapItem(void) const { return pixmapItem; }
    //int getRotation(void) const { return slider->value(); }

protected:
    void initializeMap(QGraphicsScene* scene);
    void initializeMenu();

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
    int id;
    MergeMapGraphicsItem* pixmapItem;
    QString robotName;
};

#endif // SCANMAPLISTITEMWIDGET_H
