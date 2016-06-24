#ifndef EDITSELECTEDPOINTWIDGET_H
#define EDITSELECTEDPOINTWIDGET_H

class PointsView;
class PointView;
class QHBoxLayout;
class QVBoxLayout;
class QPushButton;
class QLabel;
class QMainWindow;
class QHBoxLayout;
class QLineEdit;
class QComboBox;
class SpaceWidget;

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

    void setSelectedPoint(PointView* const& _pointView, const bool isTemporary);

    bool isTemporary(void) const { return _isTemporary; }
    void setPoints(PointsView* const _points){ points = _points;}
    //GroupMenu* getGroupMenu(void) const { return groupMenu; }
    //int getCurrentGroupIndex(void) const { return groupMenu->getWidgetsList()->currentIndex().row(); }
    QLabel* getPosXLabel(void) const { return posXLabel; }
    QLabel* getPosYLabel(void) const { return posYLabel; }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    QPushButton* getPlusButton(void) const { return plusButton; }

signals:
    void pointSaved(int, double, double, QString);

private slots:
    void saveEditSelecPointBtnEvent();
    /// check whether or not a point with the same name already exists
    void checkPointName(void) const;
    void print(int id) const;
    void showGroupLayout() const;

public slots:
    void hideGroupLayout() const;

private:
    QPushButton* plusButton;
    QPushButton* minusButton;
    QPushButton* mapButton;
    QPushButton* eyeButton;
    QPushButton* editButton;
    QHBoxLayout* topButtonsLayout;
    QHBoxLayout* eyeMapLayout;
    QHBoxLayout* cancelSaveLayout;
    QMainWindow* parent;
    QVBoxLayout* layout;
    PointView* pointView;
    QLineEdit* nameEdit;
    QLabel* posXLabel;
    QLabel* posYLabel;
    PointsView* points;
    QPushButton* saveBtn;
    QPushButton* cancelBtn;
    QComboBox* groupBox;
    bool _isTemporary;
    QHBoxLayout* groupLayout;
    QLabel* groupLabel;
    SpaceWidget* separator;
};

#endif // EDITSELECTEDPOINTWIDGET_H
