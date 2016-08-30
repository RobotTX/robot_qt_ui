#ifndef ENUMGRAPHICSTATE_H
#define ENUMGRAPHICSTATE_H

/**
 * @brief The GraphicItemState enum
 * Used to determine in which state a particular widget is
 * For example the map can be draggable or not depending on its current state
 */
enum GraphicItemState { NO_STATE, ROBOT_CREATING_PATH, NO_EVENT, NO_ROBOT_EDITING_PATH, ROBOT_EDITING_PATH, EDITING_PERM, SELECTING_HOME, EDITING_HOME, NO_ROBOT_CREATING_PATH };

#endif // ENUMGRAPHICSTATE_H
