#ifndef SELECTEDPOINTWIDGET_H
#define SELECTEDPOINTWIDGET_H

class PointView;
class QVBoxLayout;
class QLabel;
class QPushButton;
class QMainWindow;

#include <QWidget>

/**
 * @brief The SelectedPointWidget class
 * Widget of the left menu which is displayed when a point is selected
 * (from the map or the list of points)
 */
class SelectedPointWidget: public QWidget{
public:
    SelectedPointWidget(QMainWindow* parent);
    ~SelectedPointWidget();

    /**
     * @brief setSelectedPoint
     * @param _pointView
     * Updates the widget with the selected point
     */
    void setSelectedPoint(PointView* const& _pointView);

private:
    QVBoxLayout* layout;
    QPushButton* backBtn;
    QLabel* posXLabel;
    QLabel* posYLabel;
};

#endif // SELECTEDPOINTWIDGET_H
