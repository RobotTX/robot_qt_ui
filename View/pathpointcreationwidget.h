#ifndef PATHPOINTCREATIONWIDGET_H
#define PATHPOINTCREATIONWIDGET_H

class QVBoxLayout;
class QLabel;
class QPushButton;
class QLineEdit;
class QComboBox;

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

    /**
     * @brief PathPointCreationWidget
     * @param id
     * @param points
     * @param point
     * The construcor of the widget when we clicked on the map and selected a point
     */
    PathPointCreationWidget(const int id, const Points& points, const Point& point, QWidget *parent);


    void displayActionWidget(const bool show);
    void displaySaveEditBtn(const bool show, const int count);
    void resetAction(void);
    void updatePointLabel(const float _posX, const float _posY);

    /// Setters
    void setName(const QString name);
    void setId(const int id);
    void setPos(const float _posX, const float _posY);

    /// Getters
    QString getName(void) const { return name; }
    int getId(void) const { return id; }
    QComboBox* getAction(void) const { return actionBtn; }
    QLineEdit* getTimeEdit(void) const { return timeEdit; }
    int getPosX(void) const { return posX; }
    int getPosY(void) const { return posY; }
    bool isTemporary(void) const { return (name.compare("tmpPoint") == 0); }
    Point getPoint(void) const { return point; }
    void setPointLabel(const float _posX, const float _posY);

private:
    QVBoxLayout* layout;
    QLabel* pointLabel;
    float posX;
    float posY;
    QString name;
    int id;
    Points points;
    Point point;
    QComboBox* actionBtn;
    QPushButton* saveEditBtn;
    QLineEdit* timeEdit;
    QWidget* timeWidget;
    QWidget* actionWidget;

signals:
    void saveEditSignal(PathPointCreationWidget*);

private slots:
    void actionClicked(QString action);
    void saveEdit();
};

#endif // PATHPOINTCREATIONWIDGET_H
