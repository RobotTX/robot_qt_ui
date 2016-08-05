#ifndef PATHPOINTCREATIONWIDGET_H
#define PATHPOINTCREATIONWIDGET_H

class QVBoxLayout;
class QHBoxLayout;
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
     * @param point
     * The construcor of the widget when we clicked on the map and selected a point
     */
    PathPointCreationWidget(const int ids, const QString name, const double x, const double y, QWidget *parent);


    void displayActionWidget(const bool show);
    void displaySaveEditBtn(const bool show, const int count);
    void resetAction(void);
    void updatePointLabel(const float x, const float y);
    void setPointLabel(const float _posX, const float _posY);

    /// Setters
    void setName(const QString name);
    void setId(const int id);
    void setPos(const float _posX, const float y);

    /// Getters
    QString getName(void) const { return name; }
    int getId(void) const { return id; }
    QComboBox* getAction(void) const { return actionBtn; }
    QLineEdit* getTimeEdit(void) const { return timeEdit; }
    int getPosX(void) const { return posX; }
    int getPosY(void) const { return posY; }
    bool isTemporary(void) const { return (name.compare(TMP_POINT_NAME) == 0); }
    QPushButton* getCancelBtn(void) const { return cancelBtn; }
    QPushButton* getSaveEditBtn(void) const { return saveEditBtn; }
    QWidget* getTimeWidget(void) const { return timeWidget; }


private:
    QHBoxLayout* layout;
    QVBoxLayout* rightLayout ;
    QLabel* pointLabel;
    float posX;
    float posY;
    QString name;
    int id;
    QComboBox* actionBtn;
    QPushButton* saveEditBtn;
    QPushButton* cancelBtn;
    QLineEdit* timeEdit;
    QWidget* timeWidget;
    QWidget* actionWidget;
    QHBoxLayout* editLayout;

signals:
    void saveEditSignal(PathPointCreationWidget*);
    void actionChanged(int, QString);

private slots:
    void actionClicked(QString action);
    void saveEdit();
    void timeChanged(QString);
};

#endif // PATHPOINTCREATIONWIDGET_H
