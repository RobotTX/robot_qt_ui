#ifndef MERGEMAPLISTITEMWIDGET_H
#define MERGEMAPLISTITEMWIDGET_H

class QGraphicsScene;
class QPushButton;
class QLineEdit;
class MergeMapGraphicsItem;
class QCheckBox;

#include <QWidget>
#include <QSlider>
#include <QLabel>

class MergeMapListItemWidget : public QWidget {
    Q_OBJECT
public:
    MergeMapListItemWidget(int _id, QString _fileName, QGraphicsScene *scene, bool _fromRobot, QImage image = QImage(), double _resolution = -1, double _originX = -1, double _originY = -1);

    /// Setter
    void setId(const int _id){ id = _id; }

    /// Getters
    MergeMapGraphicsItem* getPixmapItem(void) const { return pixmapItem; }
    int getRotation(void) const { return slider->value(); }
    double getResolution(void) const { return resolution; }
    QPointF getOrigin(void) const { return origin; }
    QString getFileNameLabel(void) const { return fileNameLabel->text(); }
    QString getFileName(void) const { return fileName; }

    static int mod(const int a, const int b);

protected:
    void initializeMap(QString fileName, QGraphicsScene* scene, QImage image);
    void initializeMenu(QString fileName);

private slots:
    void closeBtnSlot();
    void rotLineEditSlot(QString text);
    void pixmapClickedSlot();

public slots:
    void sliderSlot(int value);

signals:
    void deleteMap(int);
    void gotOrigin(int);
    void pixmapClicked(int);

private :
    QCheckBox* box;
    QPushButton* closeBtn;
    QSlider* slider;
    QLineEdit* rotLineEdit;
    int id;
    QLabel* fileNameLabel;
    bool fromRobot;
    QPointF origin;
    double resolution;
    MergeMapGraphicsItem* pixmapItem;
    QPoint originInPixel;
    QString fileName;
};


#endif /// MERGEMAPLISTITEMWIDGET_H
