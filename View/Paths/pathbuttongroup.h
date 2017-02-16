#ifndef PATHBUTTONGROUP_H
#define PATHBUTTONGROUP_H


#include <QWidget>
#include <QButtonGroup>
#include "Model/Paths/paths.h"

class QVBoxLayout;

/**
 * @brief The PathButtonGroup class
 * provides a widget to display a group of paths
 */

class PathButtonGroup: public QWidget
{
public:
    PathButtonGroup(QWidget *_parent);
    ~PathButtonGroup(){}

    QButtonGroup* getButtonGroup(void) const { return buttonGroup; }
    QVBoxLayout* getLayout(void) const { return layout; }

public:
    /**
     * @brief deleteButtons
     * delete the buttons to prepare an update
     */
    void deleteButtons(void);
    /**
     * @brief uncheck
     * unchecks all the buttons
     */
    void uncheck(void);
    /**
     * @brief setCheckable
     * @param checkable
     * sets the checkable property of all the buttons to <checkable>
     */
    void setCheckable(const bool checkable);

protected:
    void resizeEvent(QResizeEvent *event);

private:
    QVBoxLayout* layout;
    QButtonGroup* buttonGroup;
};

#endif // PATHBUTTONGROUP_H
