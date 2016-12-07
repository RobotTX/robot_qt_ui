#ifndef MERGEMAPLISTITEMWIDGET_H
#define MERGEMAPLISTITEMWIDGET_H

class QGraphicsScene;
class QPushButton;
class QSlider;
class QLineEdit;
class QGraphicsPixmapItem;
class QLabel;

#include <QWidget>

class MergeMapListItemWidget : public QWidget {
    Q_OBJECT
public:
    MergeMapListItemWidget(int _id, QString fileName, QGraphicsScene *scene);

    void setId(const int _id){ id = _id; }

    QGraphicsPixmapItem* getPixmapItem(void) const { return pixmapItem; }

protected:
    void initializeMap(QString fileName, QGraphicsScene* scene);
    void initializeMenu(QString fileName);

private slots:
    void closeBtnSlot();
    void rotLineEditSlot(QString text);
    void sliderSlot(int value);

signals:
    void deleteMap(int);

private :
    QPushButton* closeBtn;
    QSlider* slider;
    QLineEdit* rotLineEdit;
    QGraphicsPixmapItem* pixmapItem;
    int id;
    QLabel* fileNameLabel;
};

#endif // MERGEMAPLISTITEMWIDGET_H
