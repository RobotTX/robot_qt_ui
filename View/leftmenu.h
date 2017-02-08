

#ifndef LEFTMENU_H
#define LEFTMENU_H

class Points;
class Robots;
class LeftMenuWidget;
class RobotsLeftWidget;
class MapLeftWidget;
class EditSelectedRobotWidget;
class QVBoxLayout;
class MainWindow;
class Map;
class PathPainter;
class CustomPushButton;


#include <QWidget>
#include <QSharedPointer>

/**
 * @brief The LeftMenu class
 * The main menu class that initialize all the other menu
 */
class LeftMenu: public QWidget{
    Q_OBJECT
public:
    LeftMenu(MainWindow* mainWindow, const QSharedPointer<Points> &points, QSharedPointer<Robots> const& robots,
             QSharedPointer<Map> const& _map);

    /// Getters
    LeftMenuWidget* getLeftMenuWidget(void) const {return leftMenuWidget;}
    RobotsLeftWidget* getRobotsLeftWidget(void) const {return robotsLeftWidget;}
    MapLeftWidget* getMapLeftWidget(void) const {return mapLeftWidget;}
    EditSelectedRobotWidget* getEditSelectedRobotWidget(void) const {return editSelectedRobotWidget;}
    CustomPushButton* getReturnButton(void) const { return returnButton; }
    CustomPushButton* getCloseButton(void) const { return closeBtn; }
    QWidget* getLastWidget() const {return lastWidget; }


    /**
     * @brief showBackButton
     * @param name
     * to set the label appropriately on the back button
     */
    void showBackButton(const QString name);
    /**
     * @brief hideBackButton
     * hides the back button
     */
    void hideBackButton();

    /**
     * @brief setEnableReturnCloseButtons
     * @param enable
     * enables or disables the return and close buttons
     */
    void setEnableReturnCloseButtons(const bool enable);

private:
    CustomPushButton* closeBtn;
    QVBoxLayout* leftLayout;
    QWidget* lastWidget;
    LeftMenuWidget* leftMenuWidget;
    RobotsLeftWidget* robotsLeftWidget;
    MapLeftWidget* mapLeftWidget;
    EditSelectedRobotWidget* editSelectedRobotWidget;
    CustomPushButton * returnButton;
};

#endif /// LEFTMENU_H

