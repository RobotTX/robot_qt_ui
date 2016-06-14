#ifndef PATHPOINTCREATIONWIDGET_H
#define PATHPOINTCREATIONWIDGET_H

class QVBoxLayout;
class QLabel;
class QPushButton;
class QMenu;
class QLineEdit;

#include "Model/points.h"
#include "Model/point.h"
#include <QWidget>

/**
 * @brief The PathPointCreationWidget class
 * Create the widget that display a pathpoint informations
 * on the left menu and allow the user to choose an action for this pathpoint
 */
class PathPointCreationWidget: public QWidget{
    Q_OBJECT
public:
    struct PointInfo{
        QString name;
        float posX;
        float posY;
    };

    /**
     * @brief PathPointCreationWidget
     * @param id
     * @param points
     * The construcor of the widget when we havn't selected a point yet
     */
    PathPointCreationWidget(const int id, const Points points, QString name = "Select a point");

    /**
     * @brief PathPointCreationWidget
     * @param id
     * @param points
     * @param point
     * The construcor of the widget when we clicked on the map and selected a point
     */
    PathPointCreationWidget(int id, Points points, Point point);
    ~PathPointCreationWidget();
    void initialisation(const int _id, const Points _points, QString _name);

    /**
     * @brief PathPointCreationWidget::clicked
     * called when we want to show the menu displaying the list of points
     */
    void clicked(void);
    void displayActionWidget(const bool show);
    void resetAction(void);

    /// Setters
    void setName(const QString name);
    void setId(const int id);

    /// Getters
    QString getName(void) const { return name; }
    int getId(void) const { return id; }
    QPushButton* getActionBtn(void) const { return actionBtn; }
    QLineEdit* getTimeEdit(void) const { return timeEdit; }
    int getPosX(void) const { return posX; }
    int getPosY(void) const { return posY; }
    bool isTemporary(void) const { return (name.compare("tmpPoint") == 0); }

private:
    QVBoxLayout* layout;
    QLabel* pointLabel;
    float posX;
    float posY;
    QString name;
    QMenu* pointsMenu;
    int id;
    Points points;
    Point point;
    bool waitHuman;
    QPushButton* actionBtn;
    QLineEdit* timeEdit;
    QWidget* timeWidget;
    QWidget* actionWidget;
    QVector<PointInfo> pointInfos;

signals:
    /**
     * @brief pointSelected
     * @param i
     * @param name
     * Signal emitted when a point is selected in the menu that display the list of points
     */
    void pointSelected(int i, QString name);

private slots:
    /**
     * @brief PathPointCreationWidget::pointClicked
     * @param action
     * when we clicked on a point in the list
     */
    void pointClicked(QAction *action);
    void actionClicked();
};

#endif // PATHPOINTCREATIONWIDGET_H
