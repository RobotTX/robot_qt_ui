#ifndef POINTSCONTROLLER_H
#define POINTSCONTROLLER_H

class MainWindow;
class PointsLeftWidget;
class CreatePointWidget;
class DisplaySelectedPoint;
class DisplaySelectedGroup;
class Robots;
class Map;
class PathPoint;
class MapController;

#include <QObject>
#include <QSharedPointer>
#include "Model/Points/points.h"

class PointsController : public QObject {
    Q_OBJECT
public:
    /// to display an appropriate message to the end user when he tries to create a point
    enum PointNameError { ContainsSemicolon, EmptyName, AlreadyExists, NoError };

    PointsController(MainWindow* mainWindow);
    void initializePoints(void);
    void initializeMenus(MainWindow *mainWindow, const QSharedPointer<Robots> &robots, const QSharedPointer<Map> &_map);

    /// Getters
    QSharedPointer<Points> getPoints(void) const { return points; }
    PointsLeftWidget* getPointsLeftWidget(void) const {return pointsLeftWidget;}
    CreatePointWidget* getCreatePointWidget(void) const {return createPointWidget;}
    DisplaySelectedPoint* getDisplaySelectedPoint(void) const { return displaySelectedPoint; }
    DisplaySelectedGroup* getDisplaySelectedGroup(void) const { return displaySelectedGroup; }
    QSharedPointer<PointView> getEditedPointView(void) const { return editedPointView; }

    /// Setters
    void setEditedPointView(const QSharedPointer<PointView> _editedPointView){ editedPointView = _editedPointView; }
    void resetEditedPointView(){ editedPointView = QSharedPointer<PointView>(); }

    void savePoints(const QString fileName);
    void loadPoints(const QString fileName);


    /**
     * @brief updateGroupDisplayed
     * @param groupName
     * updates the group displayed identified by its name
     */
    void updateGroupDisplayed(const QString groupName);

    void hidePointViewsToDisplayButPath(QVector<QSharedPointer<PathPoint>> currentPath);
    void hidePointViewsToDisplay(void);
    void showPointViewsToDisplay(void);
    void editTmpPathPoint(const int id, const int nbWidget);
    void setSelectedTmpPoint(MainWindow *mainWindow);
    void openInterdictionOfPointRemovalMessage(const QString pointName, const QString robotName);
    void pointSavedEvent(QString groupName, double x, double y, QString name);
    void showHomeFromRobotName(QString robotName);
    void showHomeFromHomeName(QString homeName);

private:
    void askForDeleteGroupConfirmation(const QString group);
    void askForDeleteDefaultGroupPointConfirmation(const QString index);
    void askForDeletePointConfirmation(QString pointName);

private slots:
    void replacePoint(int id, QString name);
    void plusGroupBtnEvent(void);
    void minusGroupBtnEvent(void);
    void editPointButtonEvent(void);
    void editGroupBtnEvent(void);
    void removePointFromGroupMenu(void);
    void displayPointEvent(QString name, double x, double y);
    void displayGroupMapEvent(void);
    void displayPointMapEvent(void);
    void displayPointsInGroup(void);
    void removePointFromInformationMenu(void);
    void editPointFromGroupMenu(void);
    void displayPointInfoFromGroupMenu(void);
    void updatePoint(void);
    void cancelUpdatePoint(void);
    void updateCoordinates(double x, double y);
    void displayPointFromGroupMenu(void);
    void doubleClickOnPoint(QString checkedId);
    void doubleClickOnGroup(QString checkedId);
    void createGroup(QString name);
    void modifyGroupWithEnter(QString name);
    void modifyGroupAfterClick(QString name);
    void reestablishConnectionsGroups(void);
    void reestablishConnectionsPoints(void);
    void resetPointViewsSlot(void);
    void checkPointName(QString name);

protected:
    /// this prevents a user to type names like " a                stupidname       " by removing extra spaces
    QString formatName(const QString name) const;

signals:
    void setMessageTop(QString msgType, QString msg);
    void setTemporaryMessageTop(QString type, QString message, int ms);
    void backEvent();
    void setSelectedTmpPoint(void);
    void setSelectedRobot(QPointer<RobotView> robotView);
    /// emitted every time the input field changes to allow (no error) or not the creation of the point with the indicated name
    void invalidName(QString, PointsController::PointNameError);

private:
    QSharedPointer<Points> points;
    QSharedPointer<PointView> editedPointView;
    QVector<QSharedPointer<PointView>> pointViewsToDisplay;

    PointsLeftWidget* pointsLeftWidget;
    CreatePointWidget* createPointWidget;
    DisplaySelectedPoint* displaySelectedPoint;
    DisplaySelectedGroup* displaySelectedGroup;
};

#endif // POINTSCONTROLLER_H
