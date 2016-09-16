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
#include "Model/points.h"

/**
 * @brief The CreatePointWidget class
 * Class that provides a widget that allows a user to create a permanent point upon clicking the map
 */
class CreatePointWidget: public QWidget{
    Q_OBJECT
public:

    /// to display an appropriate message to the end user when he tries to create a point
    enum Error { ContainsSemicolon, EmptyName, AlreadyExists, NoError };

    CreatePointWidget(QWidget* parent, MainWindow* mainWindow, QSharedPointer<Points> points);

    void setSelectedPoint(QSharedPointer<PointView> _pointView);

    QLabel* getPosXLabel(void) const { return posXLabel; }
    QLabel* getPosYLabel(void) const { return posYLabel; }
    CustomLineEdit* getNameEdit(void) const { return nameEdit; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QComboBox* getGroupBox(void) const { return groupBox; }
    QLabel* getGroupLabel(void) const { return groupLabel; }

    /// called when a new group is created to add it to the box
    void updateGroupBox();

private:
    /// this prevents a user to type names like " a                stupidname       " by removing extra spaces
    QString formatName(const QString name) const;

protected:
    void keyPressEvent(QKeyEvent* event);
    void showEvent(QShowEvent* event);
    void resizeEvent(QResizeEvent *event);

signals:
    /// emitted when the 'save' button is clicked
    void pointSaved(QString, double, double, QString);
    /// emitted every time the input field changes to allow (no error) or not the creation of the point with the indicated name
    void invalidName(QString, CreatePointWidget::Error);
    /// emitted after the user clicks the plus button to explain him what to do to create his point
    void displayMessageCreation(QString);
    /// to signify the mainWindow that it has to display "Click the + button to save the point" in the top layout
    void resetMessageTop(QString, QString);

private slots:
    /**
     * @brief saveEditSelecPointBtnEvent
     * emit a signal to save the point
     */
    void saveEditSelecPointBtnEvent();
    /// check whether or not a point is valid
    /// a point is valid if it's not empty, already taken and if it does not contain ";" or "}" or "pathpoint" (case insensitive)
    int checkPointName(void);
    /// shows the widgets necessary to the creation of a point, cancel and save buttons and checkbox
    void showGroupLayout(void);


public slots:
    /// hides the widgets necessary to the creation of a point, cancel and save buttons and checkbox, resetTopMessage holds whether
    /// it's been called by the cancel button by opposition to ENTER key pressed or save button and therefore if the message in the lop layout should be reset or changed to
    /// "you have created a new point"
    void hideGroupLayout(const bool pointAdded);

private:

    QHBoxLayout* cancelSaveLayout;
    QVBoxLayout* layout;
    QSharedPointer<PointView> pointView;
    CustomLineEdit* nameEdit;
    QLabel* posXLabel;
    QLabel* posYLabel;
    QSharedPointer<Points> points;
    CustomPushButton* saveBtn;
    CustomPushButton* cancelBtn;
    QComboBox* groupBox;
    QHBoxLayout* groupLayout;
    QLabel* groupLabel;
    SpaceWidget* separator;
    TopLeftMenu* actionButtons;
    QLabel* messageCreationLabel;
};

#endif // CreatePointWidget_H
