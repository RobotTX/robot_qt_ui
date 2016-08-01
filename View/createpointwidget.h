#ifndef CreatePointWidget_H
#define CreatePointWidget_H

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

    /// to display an appropriate message to the end user when he tries to create a point
    enum Error { ContainsSemicolon, EmptyName, AlreadyExists, NoError };
    CreatePointWidget(QMainWindow* parent, std::shared_ptr<Points> points);

    void setSelectedPoint(std::shared_ptr<PointView> const& _pointView);

    QLabel* getPosXLabel(void) const { return posXLabel; }
    QLabel* getPosYLabel(void) const { return posYLabel; }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    void updateGroupBox();
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }

private:
    /// this prevents a user to type names like " a                stupidname       " by removing extra spaces
    QString formatName(const QString name) const;

protected:
    void keyPressEvent(QKeyEvent* event);
    void showEvent(QShowEvent* event);

signals:
    void pointSaved(QString, double, double, QString);
    void invalidName(QString, CreatePointWidget::Error);
    void errorCreationPoint(QString, QString);

private slots:
    void saveEditSelecPointBtnEvent();
    /// check whether or not a point is valid
    /// a point is valid if it's not empty, already taken and if it does not contain ";" or "}"
    int checkPointName(void);
    void showGroupLayout(void) const;

public slots:
    void hideGroupLayout() const;

private:

    QHBoxLayout* cancelSaveLayout;
    QMainWindow* parent;
    QVBoxLayout* layout;
    std::shared_ptr<PointView> pointView;
    QLineEdit* nameEdit;
    QLabel* posXLabel;
    QLabel* posYLabel;
    std::shared_ptr<Points> points;
    QPushButton* saveBtn;
    QPushButton* cancelBtn;
    QComboBox* groupBox;
    QHBoxLayout* groupLayout;
    QLabel* groupLabel;
    SpaceWidget* separator;
    TopLeftMenu* actionButtons;

};

#endif // CreatePointWidget_H
