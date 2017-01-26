#ifndef SETTINGSCONTROLLER_H
#define SETTINGSCONTROLLER_H

#include <QPointer>
#include <QObject>
#include <QWidget>

class SettingsWidget;
class Settings;

/**
 * @brief The SettingsController class
 * This class ensures that the settings chosen by the user are consisten within the application
 * both in the model and in the view
 */
class SettingsController : public QObject
{
    Q_OBJECT

public:

    SettingsController(QWidget* parent = 0);
    ~SettingsController();

    QPointer<Settings> getSettings(void) const { return settings; }
    SettingsWidget* getSettingsView(void) const { return view; }

    /// for the laser feedbacks
    void removeRobot(const QString name);
    void addRobot(const QString name);

    void createSettingsView(void);
    void hideView(void);
    void showView(void);

public slots:
    void setHelpNeeded(const bool needed);
    void updateBatteryThreshold(const int value);
    void setMapChoice(const int choice);
    void updateLaserStatus(const int id, const bool status);

signals:
    void activateLaser(QString, bool);

private:
    QPointer<Settings> settings;
    SettingsWidget* view;
};

#endif /// SETTINGSCONTROLLER_H
