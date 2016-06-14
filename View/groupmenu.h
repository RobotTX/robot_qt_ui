#ifndef GROUPMENU_H
#define GROUPMENU_H

class Points;
class QVBoxLayout;
class QListWidget;

#include <QWidget>
/**
 * @brief The GroupMenu class
 * The purpose of this class is to display a list of points to the user
 */

class GroupMenu: public QWidget
{
public:
    GroupMenu(const Points& points, bool _editPoint);
    ~GroupMenu();

    QListWidget* getWidgetsList(void) const { return widgetsList; }

    /**
     * @brief displayReverse
     * to display the groups in the reverse order in which they are stored
     */
    void displayReverse(void);

    /**
     * @brief updateList
     * @param points
     * to update the list of points
     */
    void updateList(const Points &points);

private:
    QVBoxLayout* layout;
    QListWidget* widgetsList;
    // if we choose a group for a point we want the "no group" to be displayed
    bool editPoint;
};

#endif // GROUPMENU_H

