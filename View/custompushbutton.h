#ifndef CUSTOMPUSHBUTTON_H
#define CUSTOMPUSHBUTTON_H

class QMouseEvent;
class QLabel;

#include <QPushButton>

/**
 * @brief The CustomPushButton class
 * class that provides a custom button that handles for example double clicks
 */
class CustomPushButton: public QPushButton{
    Q_OBJECT
public:
    enum ButtonType { TOP, BOTTOM, LEFT_MENU, TOP_LEFT_MENU };
    CustomPushButton(const QIcon& icon, const QString text = "", QWidget *parent = Q_NULLPTR, const bool customTooltipEnable = false, const ButtonType type = LEFT_MENU, const QString align = "left",
                     const bool checkable = false, const bool enable = true);
    CustomPushButton(const QString text = "", QWidget *parent = Q_NULLPTR, const bool _customTooltipEnable = false, const ButtonType type = LEFT_MENU, const QString align = "left",
                     const bool checkable = false, const bool enable = true);
    void initialize(const bool checkable, const bool enable, const QString align);

    /**
     * @brief addStyleSheet
     * @param style
     * To add some styleSheet without deleting the current one
     */
    void addStyleSheet(const QString style);

    /**
     * @brief setText
     * @param text
     * Set the text of the button
     */
    void setText(const QString &text);

    /**
     * @brief moveLabel
     * Move the label "..." to the correct position in the button
     */
    void moveLabel();

    /**
     * @brief setEnabled
     * @param checked
     * Set whether or not this button is enabled
     */
    void setEnabled(bool checked);

private slots:
    /**
     * @brief toggledSlot
     * @param checked
     * Called when the button is toggled
     */
    void toggledSlot(bool checked);

signals:
    void doubleClick(QString);

protected:
    /**
     * @brief mouseDoubleClickEvent
     * @param event
     * To handle double click
     */
    void mouseDoubleClickEvent(QMouseEvent *event);

    /**
     * @brief resizeEvent
     * @param event
     * on resize event we set the size of the button depending on the size of its parent
     */
    void resizeEvent(QResizeEvent *event);
    void showEvent(QShowEvent* event);

    /**
     * @brief enterEvent
     * @param event
     * We change the style of the label as if it was hovered
     */
    void enterEvent(QEvent *event);

    /**
     * @brief enterEvent
     * @param event
     * We change the style of the label as if it was normal
     */
    void leaveEvent(QEvent *event);

private:
    ButtonType buttonType;
    QLabel* label;
    /// holds whether or not we set the tooltip manually
    bool customTooltipEnable;
};

#endif /// CUSTOMPUSHBUTTON_H
