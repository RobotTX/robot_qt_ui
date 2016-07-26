#ifndef PATHCREATIONWIDGET_H
#define PATHCREATIONWIDGET_H

class PathPointList;
class QMainWindow;
class QVBoxLayout;
class QListWidgetItem;
class PathPointCreationWidget;
class Robot;
class QMenu;
class TopLeftMenu;
class Point;

#include "Model/points.h"
#include "Model/point.h"
#include <QWidget>
#include <QPushButton>

#define WIDGET_HEIGHT 100

/**
 * @brief The PathCreationWidget class
 * The widget which display the left menu of the creation of a path
 * Display a few buttons and the pathpointlist
 */
class PathCreationWidget: public QWidget{
    Q_OBJECT

enum CheckState { NO_STATE, SUPPR, EDIT };

public:
    struct PointInfo{
        QString name;
        float posX;
        float posY;
    };

    PathCreationWidget(QMainWindow* parent, const std::shared_ptr<Points>& point);

    PathPointCreationWidget* getPathPointCreationWidget(void) const { return editedPathPointCreationWidget; }
    PathPointList* getPathPointList(void) const { return pathPointsList; }
    QVector<Point> getPointList(void) const { return pointList; }


    void updateMenu();
    void updateList();

    /**
     * @brief initialisationPathPoint
     * @param pathPoint
     * Initialize a given PathPointCreationWidget
     */
    void initialisationPathPoint(PathPointCreationWidget* pathPoint);

    /**
     * @brief resetWidget
     * Reset the widget (when you stop or start creating a path)
     */
    void resetWidget(void);

    /**
     * @brief supprItem
     * @param item
     * Delete the given pathPoint from the list
     */
    void supprItem(QListWidgetItem* item);

    /**
     * @brief editItem
     * @param item
     * Edit the given pathPoint from the list
     */
    void editItem(QListWidgetItem* item);

    /**
     * @brief addPathPoint
     * add a path point to the list from a given point
     */
    void addPathPoint(Point* const point);

    /**
     * @brief hideEvent
     * @param event
     * When the menu is hidden we clean its content
     */
    void hideEvent(QHideEvent *event);

    /**
     * @brief applySavePathPoint
     * @param posX
     * @param posY
     * Called when we click to save a temporary pathPoint we were editing/dragging
     */
    void applySavePathPoint(const float posX, const float posY, const bool save = false);

    /**
     * @brief moveEditPathPoint
     * @param posX
     * @param posY
     * Called when we drag/drop the temporary button we are editing (used to redraw the path lines)
     */
    void moveEditPathPoint(float posX, float posY);

    /**
     * @brief clicked
     * Called to open the Menu to select a Point from a list
     */
    void clicked(void);

    /**
     * @brief savePath
     * @return
     * Called when we try to save the path
     */
    bool savePath(void);

    /// Setters
    void setSelectedRobot(std::shared_ptr<Robot> const& _selectedRobot){ selectedRobot = _selectedRobot; }
    ///getters
    std::shared_ptr<Robot> getSelectedRobot(void){ return selectedRobot ;}


    TopLeftMenu* getActionButtons(void) const {return actionButtons;}

    /// redefinition of show event
    void showEvent(QShowEvent *);

private slots:
    /**
     * @brief itemClicked
     * @param item
     * Manage when the user click on the path list
     */
    void itemClicked(QListWidgetItem* item);

    /**
     * @brief supprPathPoint
     * Slot called when we click on the minus btn
     */
    void supprPathPoint(void);

    /**
     * @brief editPathPoint
     * Slot called when we click on the edit btn
     */
    void editPathPoint(void);

    /**
     * @brief saveNoExecPath
     * Slot called when we click on the save btn
     */
    void saveNoExecPath(void);

    /**
     * @brief saveExecPath
     * Slot called when we click on the save and play btn
     */
    void saveExecPath(void);

    /**
     * @brief addPathPoint
     * Slot called when we click on the plus btn
     */
    void addPathPoint(void);

    /**
     * @brief updatePointPainter
     * Function which emit the updatePathPointToPainter signal to update the painter
     */
    void updatePointPainter(const bool save = false);

    /**
     * @brief itemMovedSlot
     * @param from
     * @param to
     * Edit the vector of Point when we dit/move a tmp point
     */
    void itemMovedSlot(const QModelIndex&, int start, int, const QModelIndex&, int row);

    /**
     * @brief saveEditSlot
     * @param pathPointCreationWidget
     * Slot called when we click on the Save changes btn while editing a tmp pathPoint
     */
    void saveEditSlot(PathPointCreationWidget* pathPointCreationWidget);

    /**
     * @brief pointClicked
     * @param action
     * Sot called when a Point is selected from the Menu of points
     */
    void pointClicked(QAction *action);
    void pointClicked(QString name);

signals:
    /**
     * @brief pathSaved
     * Signal emitted when the path is complete and ready to be saved
     */
    void pathSaved(bool);

    /**
     * @brief updatePathPointToPainter
     * @param pointVector
     * Signal emitted to tell the path painter to update its drawing of the path
     */
    void updatePathPointToPainter(QVector<Point>& pointVector, bool save);

    /**
     * @brief hidePathCreationWidget
     * Signal to hide the widget
     */
    void hidePathCreationWidget();

    /**
     * @brief editTmpPathPoint
     * @param id
     * @param point
     * @param nbWidget
     * Signal emitted to tell the main window which Point we want to edit
     */
    void editTmpPathPoint(int id, Point* point, int nbWidget);

    /**
     * @brief saveEditPathPoint
     * Signal emitted when we save the pathPoint we were editing
     */
    void saveEditPathPoint();
    void setMessage(QString msgType, QString msg);
    void addPointEditPath(Point pt);
    void deletePointView(Point pt);

    /// to notify the path painter that the order of the points has changed
    void orderPointsChanged(int, int);

    /// to notify the mapView that a permanent point has been added to a path
    void addMapPathPoint(Point*);

    /// to update the path known to the mapView ( old point's name and new point's name given )
    void changePermanentPoint(QString, QString);
private:
    QVBoxLayout* layout;

    /**
     * @brief idPoint
     * id of the next point to create
     */
    int idPoint;

    /**
     * @brief pathPointsList
     * The widget displaying the list
     */
    PathPointList* pathPointsList;

    /**
     * @brief points
     * List of all the permanent points
     */
    std::shared_ptr<Points> points;
    std::shared_ptr<Robot> selectedRobot;

    /**
     * @brief state
     * State of the widget (which btn is checked, delete, edit or none)
     */
    CheckState state;


    /**
     * @brief previousItem
     * Last clicked item on the list of pathpoint
     */
    QListWidgetItem* previousItem;
    QVector<Point> pointList;

    /**
     * @brief editedPathPointCreationWidget
     * The item in the list we are currently editting
     */
    PathPointCreationWidget* editedPathPointCreationWidget;

    /**
     * @brief creatingNewPoint
     * Wether or not we are creating a new pathPoint or editing one,
     * used for the menu of Point to now what to do after selecting a point
     */
    bool creatingNewPoint;

    /**
     * @brief pointsMenu
     * The menu containing the list of points
     */
    QMenu* pointsMenu;
    QVector<PointInfo> pointInfos;

    /**
     * @brief actionButtons
     * The menu containing + - edit view and map buttons
     */
    TopLeftMenu* actionButtons ;

};

#endif // PATHCREATIONWIDGET_H
