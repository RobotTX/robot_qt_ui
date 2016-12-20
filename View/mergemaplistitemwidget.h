#ifndef MERGEMAPLISTITEMWIDGET_H
#define MERGEMAPLISTITEMWIDGET_H

class QGraphicsScene;
class QPushButton;
class QLineEdit;
class QGraphicsPixmapItem;
class MergeMapGraphicsItem;
class QCheckBox;

#include <QWidget>
#include <QSlider>
#include <QLabel>

class MergeMapListItemWidget : public QWidget {
    Q_OBJECT
public:
    MergeMapListItemWidget(int _id, QString fileName, QGraphicsScene *scene, bool _fromRobot, QImage image = QImage(), double _resolution = -1, double _originX = -1, double _originY = -1);

    /// Setter
    void setId(const int _id){ id = _id; }

    /// Getters
    MergeMapGraphicsItem* getPixmapItem(void) const { return pixmapItem; }
    int getRotation(void) const { return slider->value(); }
    double getResolution(void) const { return resolution; }
    QPointF getOrigin(void) const { return origin; }
    QString getFileNameLabel(void) const { return fileNameLabel->text(); }

protected:
    void initializeMap(QString fileName, QGraphicsScene* scene, QImage image);
    void initializeMenu(QString fileName);

private slots:
    void closeBtnSlot();
    void rotLineEditSlot(QString text);
    void sliderSlot(int value);
    void pixmapClickedSlot();

signals:
    void deleteMap(int);
    void gotOrigin(int);
    void pixmapClicked(int);

private :
    QCheckBox* box;
    QPushButton* closeBtn;
    QSlider* slider;
    QLineEdit* rotLineEdit;
    MergeMapGraphicsItem* pixmapItem;
    int id;
    QLabel* fileNameLabel;
    QPointF origin;
    QPoint originInPixel;
    double resolution;
    bool fromRobot;
};

#endif // MERGEMAPLISTITEMWIDGET_H
