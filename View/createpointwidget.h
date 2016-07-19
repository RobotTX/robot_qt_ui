#ifndef CreatePointWidget_H
#define CreatePointWidget_H

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
class TopLeftMenu;

#include <QListWidget>
#include "Model/points.h"

/**
 * @brief The CreatePointWidget class
 *
 */
class CreatePointWidget: public QWidget{
    Q_OBJECT
public:

    enum Error { ContainsSemicolon, EmptyName, AlreadyExists, NoError };
    CreatePointWidget(QMainWindow* parent, PointsView *points);

    void setSelectedPoint(PointView* const& _pointView, const bool isTemporary);

    bool isTemporary(void) const { return _isTemporary; }
    void setPoints(PointsView* const _points){ points = _points;}
    QLabel* getPosXLabel(void) const { return posXLabel; }
    QLabel* getPosYLabel(void) const { return posYLabel; }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    void updateGroupBox(const Points &_points);
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }

private:
    QString formatName(const QString name) const;

protected:
    void keyPressEvent(QKeyEvent* event);

signals:
    void pointSaved(int, double, double, QString);
    void invalidName(CreatePointWidget::Error);

private slots:
    void saveEditSelecPointBtnEvent();
    /// check whether or not a point with the same name already exists
    void checkPointName(void);
    void showGroupLayout(void) const;

public slots:
    void hideGroupLayout() const;

private:

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
    TopLeftMenu* actionButtons;

};

#endif // CreatePointWidget_H
