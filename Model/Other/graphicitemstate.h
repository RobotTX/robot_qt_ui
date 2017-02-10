#ifndef ENUMGRAPHICSTATE_H
#define ENUMGRAPHICSTATE_H

/**
 * @brief The GraphicItemState enum
 * Used to determine in which state a particular widget is
 * For example the map can be draggable or not depending on its current state
 */
enum GraphicItemState { NO_STATE, CREATING_PATH, NO_EVENT, EDITING_PATH, EDITING_PERM, SELECTING_HOME, EDITING_HOME, SCANNING };

#endif /// ENUMGRAPHICSTATE_H
