#ifndef PATHPOINTCREATIONWIDGET_H
#define PATHPOINTCREATIONWIDGET_H

class QVBoxLayout;
class QHBoxLayout;
class QLabel;
class CustomPushButton;
class CustomLineEdit;
class QComboBox;
class QGridLayout;

#include "Model/Points/points.h"
#include "Model/Points/point.h"
#include "Model/Paths/pathpoint.h"
#include <QWidget>

/**
 * @brief The PathPointCreationWidget class
 * Create the widget that display a pathpoint informations
 * on the left menu and allow the user to choose an action for this pathpoint
 */
class PathPointCreationWidget: public QWidget{
    Q_OBJECT
public:

    /**
     * @brief PathPointCreationWidget
     * @param id
     * @param point
     * The construcor of the widget when we clicked on the map and selected a point
     */
    PathPointCreationWidget(const int ids, const QString name, const double x, const double y, QWidget *parent);

    /// Setters
    void setName(const QString name);
    void setId(const int id);
    void setPos(const float _posX, const float y);
    void setActionWidget(const int waitTime);

    /// Getters
    QString getName(void) const { return name; }
    int getId(void) const { return id; }
    QComboBox* getAction(void) const { return actionBtn; }
    CustomLineEdit* getTimeEdit(void) const { return timeEdit; }
    int getPosX(void) const { return posX; }
    int getPosY(void) const { return posY; }
    CustomPushButton* getSaveEditBtn(void) const { return saveEditBtn; }
    QWidget* getEditWidget(void) const { return editWidget; }
    QWidget* getPathWidget(void) const { return pathWidget; }

public:

    /**
     * @brief displayActionWidget
     * @param show
     * Display or not the action widget with the choice between "Human Action" or "Wait time"
     */
    void displayActionWidget(const bool show);

    /**
     * @brief resetAction
     * Reset the action widget
     */
    void resetAction(void);

    /**
     * @brief setPointLabel
     * @param _posX
     * @param _posY
     * Set the displayed text
     */
    void setPointLabel(const float _posX, const float _posY);

private slots:
    /**
     * @brief actionClicked
     * @param action
     * When an action is clicked in the action widget, we display or not the QLineEdit for the time to wait
     */
    void actionClicked(QString action);

    /**
     * @brief saveEdit
     * To save the edition of the point
     */
    void saveEdit();

    /**
     * @brief cancelEdit
     * To cancel the edition of the point
     */
    void cancelEdit();

    /**
     * @brief timeChanged
     * When the time has been changed in the action widget
     */
    void timeChanged(QString);
    /**
     * @brief removePathPoint
     * @param checked
     * called when the cross is clicked within an item of the list
     */
    void removePathPoint();

signals:
    /// emitted when changes must be saved
    void saveEditSignal(PathPointCreationWidget*);
    /// emitted when changes must be canceled
    void cancelEditSignal(PathPointCreationWidget*);
    /// emitted when action has changed
    void actionChanged(int, QString);
    /// emitted when the cross button is clicked and the point must be removed
    void removePathPoint(PathPointCreationWidget*);

private:
    int id;
    QString name;
    float posX;
    float posY;
    QLabel* pointLabel;
    QComboBox* actionBtn;
    CustomPushButton* saveEditBtn;
    CustomPushButton* cancelBtn;
    CustomLineEdit* timeEdit;
    QWidget* timeWidget;
    QWidget* actionWidget;
    QWidget* editWidget;
    QWidget* pathWidget;
};

#endif // PATHPOINTCREATIONWIDGET_H
