#ifndef MERGEMAPWIDGET_H
#define MERGEMAPWIDGET_H

class QHBoxLayout;
class CustomQGraphicsView;
class QButtonGroup;
class QListWidget;
class MergeMapListItemWidget;


#include <QWidget>
#include <QGraphicsScene>

#define MERGE_WIDGET_HEIGHT 120

class MergeMapWidget : public QWidget {
    Q_OBJECT
public:
    MergeMapWidget(QWidget* parent = Q_NULLPTR);
    ~MergeMapWidget();

protected:
    void initializeMenu();
    void initializeMap();
    void refreshIds();

private slots:
    void cancelSlot();
    void saveSlot();
    void resetSlot();
    void undoSlot();
    void redoSlot();
    void addImageFileSlot();
    void addImageRobotSlot();
    void deleteMapSlot(int itemId);

private:
    QListWidget* listWidget;
    QHBoxLayout* layout;
    QGraphicsScene* scene;
    CustomQGraphicsView* graphicsView;
    QButtonGroup* sizeGroup;
};

#endif // MERGEMAPWIDGET_H
