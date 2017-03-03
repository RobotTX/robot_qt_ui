#ifndef MAINMENUCONTROLLER_H
#define MAINMENUCONTROLLER_H

#include <QObject>
#include <QVariant>

class MainMenuController : public QObject {
    Q_OBJECT
public:
    MainMenuController(QObject *applicationWindow, QObject* parent);

private slots:
    /**
     * @brief menuClicked
     * @param txt
     * @param checked
     * Called when a button in the left menu is clicked,
     * we send a signal to the mainMenuViewsFrame to display the selected page
     */
    void menuClicked(QString txt, bool checked);

    /**
     * @brief closeMenuClicked
     * @param txt
     * Called when a close button is clicked in the mainMenuViewsFrame,
     * we send a signal to the mainMenuFrame to uncheck the corresponding menu button
     */
    void closeMenuClicked(QString txt);

signals:
    /**
     * @brief showMenu
     * signal to the mainMenuViewsFrame to display the selected page
     */
    void showMenu(QVariant);

    /**
     * @brief closeMenu
     * signal to the mainMenuFrame to uncheck the corresponding menu button
     */
    void closeMenu(QVariant);

};

#endif // MAINMENUCONTROLLER_H
