#ifndef PATHCREATIONWIDGET_H
#define PATHCREATIONWIDGET_H

class MainWindow;
class Points;
class Robot;
class TopLeftMenu;
class QMenu;
class PathPointList;
class QListWidgetItem;
class PathPointCreationWidget;
class PathPoint;
class CustomLineEdit;
class QLabel;
class CustomPushButton;

#include <QWidget>
#include <QSharedPointer>
#include <QVBoxLayout>
#include "Model/paths.h"
#include "Model/graphicitemstate.h"
#include "Model/pathpoint.h"

#define WIDGET_HEIGHT 120

/**
 * @brief The PathCreationWidget class
 * The widget which display the left menu of the creation of a path
 * Display a few buttons and the pathpointlist
 */
class PathCreationWidget: public QWidget{
    Q_OBJECT

enum CheckState { NO_STATE, CREATE, EDIT };

public:
    struct PointInfo {
        QString name;
        float posX;
        float posY;
    };

    PathCreationWidget(QWidget *parent, const QSharedPointer<Points>& points, const QSharedPointer<Paths>& _paths, const bool associatedToRobot);

    QString getCurrentPathName(void) const { return currentPathName; }
    PathPointList* getPathPointList(void) const { return pathPointsList; }
    QString getCurrentGroupName(void) const { return currentGroupName; }
    CustomPushButton* getCancelButton(void) const { return cancelBtn; }
    CustomLineEdit* getNameEdit(void) const { return nameEdit; }
    CustomPushButton* getSaveButton(void) const { return saveBtn; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }

    void setCurrentPathName(const QString name);
    void setCurrentGroupName(const QString name) { currentGroupName = name; }

public:
    /**
     * @brief updatePath
     * @param _currentPath
     * recreates the path points list from scratch using <_currentPath>
     */
    void updatePath(const QVector<QSharedPointer<PathPoint> >& _currentPath);
    /**
     * @brief updatePointsList
     * updates the menu from which we can choose permanent points to add to a path
     */
    void updatePointsList(void);
    /**
     * @brief deleteItem
     * @param item
     * to remove a point from the path point list
     */
    void deleteItem(QListWidgetItem* item);
    /**
     * @brief editPathPoint
     * @param name
     * @param x
     * @param y
     * updates the currently selected path point in the list
     */
    void editPathPoint(const QString name, const double x, const double y);

protected:
    void showEvent(QShowEvent* event);
    void keyPressEvent(QKeyEvent* event);

signals:
    /// emitted when the waiting times of points have not been set properly so that a message is displayed to the user
    void setMessage(QString, QString );
    /// emitted when a new path point is added to the list
    void addPathPoint(QString, double, double, int, int);
    /// emitted when a path point is deleted
    void deletePathPoint(int);
    /// emitted when the order of the path has changed
    void orderPathPointChanged(int, int);
    /// emitted when the widget is reset
    void resetPath();
    /// emitted when a waiting time is changed
    void actionChanged(int, int, QString);
    /// emitted when a path point is edited
    void editPathPoint(int, QString, double, double);
    void editTmpPathPoint(int, QString, double, double);
    /// emitted when the path is saved
    void saveEditPathPoint();
    /// emitted when the cancel button is clicked
    void cancelEditPathPoint();
    /// emitted when the save button is clicked
    void savePath();
    /// emitted when the name of a path is being edited to notify whether or not this is a valid name
    void codeEditPath(int codeError);
    /// emitted when the button clean is clicked to clear the temporary path of all its points
    void resetWidgetSignal();

private slots:
    /**
     * @brief resetWidgetRelaySlot
     * relays a signal through the click of the clean button (because the button signal can only provide a bool and we need to send a state)
     */
    void resetWidgetRelaySlot();
    /**
     * @brief resetWidget
     * clear the path point lists of all its path points upon clicking the clean button
     */
    void resetWidget();
    /**
     * @brief addPathPointByMenuSlot
     * adds a permanent point to the path
     */
    void addPathPointByMenuSlot(void);
    /**
     * @brief deletePathPointSlot
     * deletes the path point using the minus button
     */
    void deletePathPointSlot();
    /**
     * @brief editPathPointSlot
     * Prepares the edition of the selected path point
     */
    void editPathPointSlot();
    /**
     * @brief itemClicked
     * @param item
     * Called when an item of the path point list is called
     */
    void itemClicked(QListWidgetItem* item);
    /**
     * @brief itemMovedSlot
     * @param start
     * @param row
     * when a path point is moved within the path points list
     */
    void itemMovedSlot(const QModelIndex& , int start, int , const QModelIndex& , int row);
    /**
     * @brief savePathClicked
     * called when the save button is called
     */
    void savePathClicked(void);
    /**
     * @brief clicked
     * called to open the menu of points
     */
    void clicked(void);
    /**
     * @brief pointClicked
     * @param action
     * called when a point is clicked in the menu
     */
    void pointClicked(QAction *action);
    /**
     * @brief addPathPointSlot
     * @param name
     * @param x
     * @param y
     * @param action
     * @param waitTime
     * adds a slot in the path point list
     */
    void addPathPointSlot(QString name, double x, double y, PathPoint::Action action = PathPoint::Action::WAIT, int waitTime = 0);
    /**
     * @brief saveEditSlot
     * @param
     * called when changes must be applied to an edited path point
     */
    void saveEditSlot(PathPointCreationWidget* pathPointCreationWidget);
    /**
     * @brief cancelEditSlot
     * @param pathPointCreationWidget
     * called when changes made to a path point must be cancelled
     */
    void cancelEditSlot(PathPointCreationWidget* pathPointCreationWidget);
    /**
     * @brief actionChangedSlot
     * @param id
     * @param action
     * @param waitTime
     * called when the waiting time of a path point is updated
     */
    void actionChangedSlot(int id, int action, QString waitTime);
    /**
     * @brief checkPathName
     * @param name
     * checks that the name of a path point is valid (not already taken or empty
     */
    void checkPathName(const QString name);
    /**
     * @brief deletePathPointWithCross
     * @param pathPointCreationWidget
     * deletes a path point using the cross button to the right of the label describing the point
     */
    void deletePathPointWithCross(PathPointCreationWidget* pathPointCreationWidget);

private:
    QSharedPointer<Points> points;
    QSharedPointer<Paths> paths;
    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;
    QMenu* pointsMenu;
    PathPointList* pathPointsList;
    CheckState checkState;
    QLabel* nameLabel;
    CustomLineEdit* nameEdit;
    QString currentGroupName;
    QString currentPathName;
    CustomPushButton* cleanBtn;
    CustomPushButton* cancelBtn;
    CustomPushButton* saveBtn;
};

#endif // PATHCREATIONWIDGET_H

