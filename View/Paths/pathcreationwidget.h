#ifndef PATHCREATIONWIDGET_H
#define PATHCREATIONWIDGET_H

class MainWindow;
class Points;
class Robot;
class TopLeftMenu;
class QMenu;
class QListWidgetItem;
class PathPointCreationWidget;
class PathPoint;
class CustomLineEdit;
class QLabel;
class CustomPushButton;

#include <QWidget>
#include <QSharedPointer>
#include <QVBoxLayout>
#include "Model/Paths/paths.h"
#include "Model/Other/graphicitemstate.h"
#include "Model/Paths/pathpoint.h"
#include "View/Paths/pathpointlist.h"

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

    PathCreationWidget(QWidget *parent, const bool associatedToRobot);

    QString getCurrentPathName(void) const { return currentPathName; }
    PathPointList* getPathPointList(void) const { return pathPointsList; }
    QString getCurrentGroupName(void) const { return currentGroupName; }
    CustomPushButton* getCancelButton(void) const { return cancelBtn; }
    CustomLineEdit* getNameEdit(void) const { return nameEdit; }
    CustomPushButton* getSaveButton(void) const { return saveBtn; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QMenu* getPointsMenu(void) const { return pointsMenu; }

    void setCurrentPathName(const QString name);
    void setCurrentGroupName(const QString name) { currentGroupName = name; }
    void setCanSave(const bool _canSave){ canSave = _canSave; }

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
    void updatePointsList(QSharedPointer<Points> points);

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

    /**
     * @brief pointClicked
     * @param action
     * called when a point is clicked in the menu
     */
    void pointClicked(Position pos, QString pointName);

    /**
     * @brief editPathPoint
     * Prepares the edition of the selected path point
     */
    void editPathPoint(const QSharedPointer<Points> points);

public slots:
    /**
     * @brief resetWidget
     * clear the path point lists of all its path points upon clicking the clean button
     */
    void resetWidget();

private slots:
    /**
     * @brief resetWidgetRelaySlot
     * relays a signal through the click of the clean button (because the button signal can only provide a bool and we need to send a state)
     */
    void resetWidgetRelaySlot();

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
     * @brief addPathPointSlot
     * @param name
     * @param x
     * @param y
     * @param action
     * @param waitTime
     * adds a slot in the path point list
     */
    void addPathPointSlot(QString name, double x, double y, int waitTime = 0);

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
    void actionChangedSlot(int id, QString waitTime);

    /**
     * @brief deletePathPointWithCross
     * @param pathPointCreationWidget
     * deletes a path point using the cross button to the right of the label describing the point
     */
    void deletePathPointWithCross(PathPointCreationWidget* pathPointCreationWidget);

protected:
    void showEvent(QShowEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void hideEvent(QHideEvent *event);

signals:
    /// emitted when the waiting times of points have not been set properly so that a message is displayed to the user
    void setMessage(QString, QString );
    /// emitted when a new path point is added to the list
    void addPathPoint(QString, double, double, int);
    /// emitted when a path point is deleted
    void deletePathPoint(int);
    /// emitted when the order of the path has changed
    void orderPathPointChanged(int, int);
    /// emitted when the widget is reset
    void resetPath();
    /// emitted when a waiting time is changed
    void actionChanged(int, QString);
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
    void codeEditPath(int);
    /// emitted when the button clean is clicked to clear the temporary path of all its points
    void resetWidgetSignal();
    void updatePointsList();
    void updatePathPointCreationWidget(PathPointCreationWidget*);

private:
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
    bool canSave;
};

#endif // PATHCREATIONWIDGET_H

