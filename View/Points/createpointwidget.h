#ifndef CreatePointWidget_H
#define CreatePointWidget_H

class PointView;
class QHBoxLayout;
class QVBoxLayout;
class CustomPushButton;
class QLabel;
class MainWindow;
class QHBoxLayout;
class CustomLineEdit;
class QComboBox;
class SpaceWidget;
class TopLeftMenu;

#include <QListWidget>
#include "Model/Points/points.h"

/**
 * @brief The CreatePointWidget class
 * Class that provides a widget that allows a user to create a permanent point upon clicking the map
 */
class CreatePointWidget: public QWidget{
    Q_OBJECT
public:

    CreatePointWidget(MainWindow* mainWindow);

    void setSelectedPoint(QSharedPointer<PointView> _pointView);

    QLabel* getPosXLabel(void) const { return posXLabel; }
    QLabel* getPosYLabel(void) const { return posYLabel; }
    CustomLineEdit* getNameEdit(void) const { return nameEdit; }
    CustomPushButton* getSaveBtn(void) const { return saveBtn; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QComboBox* getGroupBox(void) const { return groupBox; }
    QLabel* getGroupLabel(void) const { return groupLabel; }

    /// called when a new group is created to add it to the box
    void updateGroupBox(QSharedPointer<Points> points);

protected:
    void keyPressEvent(QKeyEvent* event);
    void showEvent(QShowEvent* event);
    void resizeEvent(QResizeEvent *event);

signals:
    /// emitted when the 'save' button is clicked
    void pointSaved(QString, double, double, QString);
    /// emitted after the user clicks the plus button to explain him what to do to create his point
    void setMessageTop(QString, QString);

private slots:
    /**
     * @brief saveEditSelecPointBtnEvent
     * emit a signal to save the point
     */
    void saveEditSelecPointBtnEvent(void);
    /// shows the widgets necessary to the creation of a point, cancel and save buttons and checkbox
    void showGroupLayout(void);


public slots:
    /// hides the widgets necessary to the creation of a point, cancel and save buttons and checkbox, resetTopMessage holds whether
    /// it's been called by the cancel button by opposition to ENTER key pressed or save button and therefore if the message in the lop layout should be reset or changed to
    /// "you have created a new point"
    void hideGroupLayout(const bool pointAdded);

private:

    CustomLineEdit* nameEdit;
    CustomPushButton* saveBtn;
    CustomPushButton* cancelBtn;
    QComboBox* groupBox;
    QLabel* groupLabel;
    QLabel* messageCreationLabel;
    QLabel* posXLabel;
    QLabel* posYLabel;
    QSharedPointer<PointView> pointView;
    TopLeftMenu* actionButtons;
};

#endif /// CreatePointWidget_H
