#ifndef DISPLAYSELECTEDGROUP_H
#define DISPLAYSELECTEDGROUP_H

class CustomScrollArea;
class QVBoxLayout;
class QHBoxLayout;
class QMainWindow;
class QLabel;
class Points;

#include "Model/points.h"
#include <QWidget>
#include "View/pointbuttongroup.h"
#include <QSharedPointer>
#include "topleftmenu.h"
/**
 * @brief The DisplaySelectedGroup class
 * The purpose of this class is to provide a QWidget that displays the name and the points
 * contained by the group that was just clicked
 */

class DisplaySelectedGroup: public QWidget {
    Q_OBJECT
public:
    DisplaySelectedGroup(QWidget *_parent, const QSharedPointer<Points> &_points);

    PointButtonGroup* getPointButtonGroup(void) const { return pointButtonGroup; }
    TopLeftMenu* getActionButtons(void) const { return actionButtons; }
    QLabel* getNameLabel(void) const { return name; }
    QSharedPointer<Points> getPoints(void) const { return points; }

    /**
     * @brief setName
     * @param _name
     * sets the text of the label displaying the name
     */
    void setName(const QString _name);

    /**
     * @brief uncheck
     * unchecks all the buttons of the group
     */
    void uncheck(void) { pointButtonGroup->uncheck(); }

    /**
     * @brief disableButtons
     * resets the enability, checkability, tooltips of the buttons (called after an operation is performed and
     * no point is selected
     */
    void disableButtons();

private slots:
    /**
     * @brief buttonClickedSlot
     * called when a button is clicked in the group to either unselect it (if it was already selected before being clicked)
     * or enable other buttons
     */
    void buttonClickedSlot(QAbstractButton*);

protected:
    void showEvent(QShowEvent* event);
    void resizeEvent(QResizeEvent *event);
    void keyPressEvent(QKeyEvent* event);

signals:
    /// emitted when a button is clicked to reset the colors of the path points
    void resetPathPointViews();
    /// emitted when the delete key is pressed
    void removePoint();

private:

    PointButtonGroup* pointButtonGroup;

    /// this is the graphical object that allows a user to scroll the points if they can't fit in the screen
    CustomScrollArea* scrollArea;

    QVBoxLayout* layout;
    TopLeftMenu* actionButtons;

    QLabel* name;
    QSharedPointer<Points> points;
    /// to remember which group is selected
    QString lastCheckedButton;
};

#endif // DISPLAYSELECTEDGROUP_H
