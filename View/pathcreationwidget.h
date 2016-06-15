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
    void initialisationPathPoint(PathPointCreationWidget* pathPoint);

    void setSelectedRobot(Robot* const _selectedRobot){ selectedRobot = _selectedRobot; }
    void resetWidget(void);
    void supprItem(QListWidgetItem* item);

    void editItem(QListWidgetItem* item);

    /**
     * @brief addPathPoint
     * add a path point to the list from a given point
     */
    void addPathPoint(Point* const point);
    void hideEvent(QHideEvent *event);
    void applySavePathPoint(float posX, float posY);
    void moveEditPathPoint(float posX, float posY);
    void clicked(void);


private slots:
    /**
     * @brief itemClicked
     * @param item
     * Manage when the user click on the path list
     */
    void itemClicked(QListWidgetItem* item);

    void supprPathPoint(void);
    void editPathPoint(void);
    void savePath(void);

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
    void pathSaved();
    void updatePathPointToPainter(QVector<Point>* pointVector);
    void hidePathCreationWidget();

    void editTmpPathPoint(int id, Point* point, int nbWidget);
    void saveEditPathPoint();


private:
    QVBoxLayout* layout;
    int idPoint;
    PathPointList* pathPointsList;
    Points points;
    Robot* selectedRobot;
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
