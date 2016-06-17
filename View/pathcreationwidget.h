#ifndef PATHCREATIONWIDGET_H
#define PATHCREATIONWIDGET_H

class PathPointList;
class QMainWindow;
class QVBoxLayout;
class QListWidgetItem;
class PathPointCreationWidget;
class Robot;
class QMenu;

#include "Model/points.h"
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

    PathCreationWidget(QMainWindow* parent, const Points& point);
    ~PathCreationWidget();

    /**
     * @brief initialisationPathPoint
     * @param pathPoint
     * Initialise a given PathPointCreationWidget
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
     * Called when we click to save a pathPoint
     */
    void applySavePathPoint(float posX, float posY);
    void moveEditPathPoint(float posX, float posY);
    void clicked(void);
    bool savePath(void);

    /// Setters
    void setSelectedRobot(std::shared_ptr<Robot> const& _selectedRobot){ selectedRobot = _selectedRobot; }



private slots:
    /**
     * @brief itemClicked
     * @param item
     * Manage when the user click on the path list
     */
    void itemClicked(QListWidgetItem* item);

    void supprPathPoint(void);
    void editPathPoint(void);
    void saveNoExecPath(void);
    void saveExecPath(void);

    /**
     * @brief addPathPoint
     * add a path point to the list
     */
    void addPathPoint(void);

    /**
     * @brief updatePointPainter
     * Function which emit the updatePathPointToPainter signal to update the painter
     */
    void updatePointPainter(void);

    void itemMovedSlot(const int from, const int to);
    void saveEditSlot(PathPointCreationWidget* pathPointCreationWidget);
    void pointClicked(QAction *action);

signals:
    void pathSaved(bool);
    void updatePathPointToPainter(QVector<Point>* pointVector);
    void hidePathCreationWidget();

    void editTmpPathPoint(int id, Point* point, int nbWidget);
    void saveEditPathPoint();


private:
    QVBoxLayout* layout;
    int idPoint;
    PathPointList* pathPointsList;
    Points points;
    std::shared_ptr<Robot> selectedRobot;
    CheckState state;
    QPushButton* newBtn;
    QPushButton* supprBtn;
    QPushButton* editBtn;
    QListWidgetItem* previousItem;
    QVector<Point> pointList;
    PathPointCreationWidget* editedPathPointCreationWidget;
    bool creatingNewPoint;
    QMenu* pointsMenu;
    QVector<PointInfo> pointInfos;

};

#endif // PATHCREATIONWIDGET_H
