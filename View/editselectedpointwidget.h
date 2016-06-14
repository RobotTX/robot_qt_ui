#ifndef EDITSELECTEDPOINTWIDGET_H
#define EDITSELECTEDPOINTWIDGET_H

class PointsView;
class PointView;
class QVBoxLayout;
class QPushButton;
class QLabel;
class QMainWindow;
class QHBoxLayout;
class QLineEdit;

#include <QListWidget>
#include "View/groupmenu.h"

/**
 * @brief The EditSelectedPointWidget class
 *
 */
class EditSelectedPointWidget: public QWidget{
    Q_OBJECT
public:
    EditSelectedPointWidget(QMainWindow* parent, PointsView *points);
    ~EditSelectedPointWidget();

    void setSelectedPoint(PointView* _pointView, const bool isTemporary);

    bool isTemporary(void) const { return _isTemporary; }
    void setPoints(PointsView* const _points){ points = _points;}
    GroupMenu* getGroupMenu(void) const { return groupMenu; }
    int getCurrentGroupIndex(void) const { return groupMenu->getWidgetsList()->currentIndex().row(); }
    QLabel* getPosXLabel(void) const { return posXLabel; }
    QLabel* getPosYLabel(void) const { return posYLabel; }
    QLineEdit* getNameEdit(void) const { return nameEdit; }

signals:
    void pointSaved(void);

private:
    QMainWindow* parent;
    QVBoxLayout* layout;
    PointView* pointView;
    QLineEdit* nameEdit;
    QLabel* posXLabel;
    QLabel* posYLabel;
    PointsView* points;
    QPushButton* saveBtn;
    GroupMenu* groupMenu;
    bool _isTemporary;

private slots:
    void saveEditSelecPointBtnEvent();
    void checkPointName();
};

#endif // EDITSELECTEDPOINTWIDGET_H
