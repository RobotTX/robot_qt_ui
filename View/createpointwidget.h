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

    CreatePointWidget(QMainWindow* parent, QSharedPointer<Points> points);

    void setSelectedPoint(QSharedPointer<PointView> _pointView);

    QLabel* getPosXLabel(void) const { return posXLabel; }
    QLabel* getPosYLabel(void) const { return posYLabel; }
    QLineEdit* getNameEdit(void) const { return nameEdit; }
    void updateGroupBox();
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QComboBox* getGroupBox(void) const { return groupBox; }
    QLabel* getGroupLabel(void) const { return groupLabel; }

private:
    /// this prevents a user to type names like " a                stupidname       " by removing extra spaces
    QString formatName(const QString name) const;

protected:
    void keyPressEvent(QKeyEvent* event);
    void showEvent(QShowEvent* event);

signals:
    /// emitted when the 'save' button is clicked
    void pointSaved(QString, double, double, QString);
    /// emitted every time the input field changes to allow (no error) or not the creation of the point with the indicated name
    void invalidName(QString, CreatePointWidget::Error);
    /// emitted after the user clicks the plus button to explain him what to do to create his point
    void displayMessageCreation(QString);

private slots:
    void saveEditSelecPointBtnEvent();
    /// check whether or not a point is valid
    /// a point is valid if it's not empty, already taken and if it does not contain ";" or "}" or "pathpoint" (case insensitive)
    int checkPointName(void);
    /// shows the widgets necessary to the creation of a point, cancel and save buttons and checkbox
    void showGroupLayout(void);

public slots:
    /// hides the widgets necessary to the creation of a point, cancel and save buttons and checkbox
    void hideGroupLayout() const;

private:

    QHBoxLayout* cancelSaveLayout;
    QMainWindow* parent;
    QVBoxLayout* layout;
    QSharedPointer<PointView> pointView;
    QLineEdit* nameEdit;
    QLabel* posXLabel;
    QLabel* posYLabel;
    QSharedPointer<Points> points;
    QPushButton* saveBtn;
    QPushButton* cancelBtn;
    QComboBox* groupBox;
    QHBoxLayout* groupLayout;
    QLabel* groupLabel;
    SpaceWidget* separator;
    TopLeftMenu* actionButtons;

};

#endif // CreatePointWidget_H
