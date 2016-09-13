/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../Controller/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QSharedPointer>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[180];
    char stringdata0[2843];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 11), // "sendCommand"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 11), // "nameChanged"
QT_MOC_LITERAL(4, 36, 24), // "changeCmdThreadRobotName"
QT_MOC_LITERAL(5, 61, 12), // "addPathPoint"
QT_MOC_LITERAL(6, 74, 4), // "name"
QT_MOC_LITERAL(7, 79, 1), // "x"
QT_MOC_LITERAL(8, 81, 1), // "y"
QT_MOC_LITERAL(9, 83, 16), // "GraphicItemState"
QT_MOC_LITERAL(10, 100, 19), // "addNoRobotPathPoint"
QT_MOC_LITERAL(11, 120, 17), // "updatePathPainter"
QT_MOC_LITERAL(12, 138, 26), // "updatePathPainterPointView"
QT_MOC_LITERAL(13, 165, 9), // "resetPath"
QT_MOC_LITERAL(14, 175, 23), // "resetPathCreationWidget"
QT_MOC_LITERAL(15, 199, 11), // "updateRobot"
QT_MOC_LITERAL(16, 211, 9), // "ipAddress"
QT_MOC_LITERAL(17, 221, 4), // "posX"
QT_MOC_LITERAL(18, 226, 4), // "posY"
QT_MOC_LITERAL(19, 231, 3), // "ori"
QT_MOC_LITERAL(20, 235, 14), // "updateMetadata"
QT_MOC_LITERAL(21, 250, 5), // "width"
QT_MOC_LITERAL(22, 256, 6), // "height"
QT_MOC_LITERAL(23, 263, 10), // "resolution"
QT_MOC_LITERAL(24, 274, 7), // "originX"
QT_MOC_LITERAL(25, 282, 7), // "originY"
QT_MOC_LITERAL(26, 290, 9), // "updateMap"
QT_MOC_LITERAL(27, 300, 8), // "mapArray"
QT_MOC_LITERAL(28, 309, 14), // "connectToRobot"
QT_MOC_LITERAL(29, 324, 4), // "quit"
QT_MOC_LITERAL(30, 329, 16), // "setSelectedRobot"
QT_MOC_LITERAL(31, 346, 10), // "RobotView*"
QT_MOC_LITERAL(32, 357, 9), // "robotView"
QT_MOC_LITERAL(33, 367, 17), // "editSelectedRobot"
QT_MOC_LITERAL(34, 385, 16), // "QAbstractButton*"
QT_MOC_LITERAL(35, 402, 6), // "button"
QT_MOC_LITERAL(36, 409, 24), // "setSelectedRobotNoParent"
QT_MOC_LITERAL(37, 434, 29), // "setSelectedRobotFromPointSlot"
QT_MOC_LITERAL(38, 464, 9), // "robotName"
QT_MOC_LITERAL(39, 474, 13), // "robotBtnEvent"
QT_MOC_LITERAL(40, 488, 13), // "pointBtnEvent"
QT_MOC_LITERAL(41, 502, 11), // "mapBtnEvent"
QT_MOC_LITERAL(42, 514, 12), // "pathBtnEvent"
QT_MOC_LITERAL(43, 527, 17), // "plusGroupBtnEvent"
QT_MOC_LITERAL(44, 545, 18), // "minusGroupBtnEvent"
QT_MOC_LITERAL(45, 564, 17), // "editGroupBtnEvent"
QT_MOC_LITERAL(46, 582, 12), // "openLeftMenu"
QT_MOC_LITERAL(47, 595, 22), // "backSelecRobotBtnEvent"
QT_MOC_LITERAL(48, 618, 22), // "editSelecRobotBtnEvent"
QT_MOC_LITERAL(49, 641, 25), // "addPathSelecRobotBtnEvent"
QT_MOC_LITERAL(50, 667, 17), // "backRobotBtnEvent"
QT_MOC_LITERAL(51, 685, 17), // "editRobotBtnEvent"
QT_MOC_LITERAL(52, 703, 22), // "checkRobotBtnEventMenu"
QT_MOC_LITERAL(53, 726, 24), // "checkRobotBtnEventSelect"
QT_MOC_LITERAL(54, 751, 18), // "checkRobotBtnEvent"
QT_MOC_LITERAL(55, 770, 15), // "saveMapBtnEvent"
QT_MOC_LITERAL(56, 786, 15), // "loadMapBtnEvent"
QT_MOC_LITERAL(57, 802, 28), // "cancelEditSelecRobotBtnEvent"
QT_MOC_LITERAL(58, 831, 15), // "robotSavedEvent"
QT_MOC_LITERAL(59, 847, 23), // "minusSelecPointBtnEvent"
QT_MOC_LITERAL(60, 871, 22), // "editSelecPointBtnEvent"
QT_MOC_LITERAL(61, 894, 16), // "setSelectedPoint"
QT_MOC_LITERAL(62, 911, 15), // "pointSavedEvent"
QT_MOC_LITERAL(63, 927, 9), // "groupName"
QT_MOC_LITERAL(64, 937, 10), // "deletePath"
QT_MOC_LITERAL(65, 948, 7), // "robotNb"
QT_MOC_LITERAL(66, 956, 17), // "playSelectedRobot"
QT_MOC_LITERAL(67, 974, 29), // "askForDeleteGroupConfirmation"
QT_MOC_LITERAL(68, 1004, 5), // "group"
QT_MOC_LITERAL(69, 1010, 29), // "askForDeletePointConfirmation"
QT_MOC_LITERAL(70, 1040, 5), // "index"
QT_MOC_LITERAL(71, 1046, 17), // "displayPointEvent"
QT_MOC_LITERAL(72, 1064, 41), // "askForDeleteDefaultGroupPoint..."
QT_MOC_LITERAL(73, 1106, 20), // "displayGroupMapEvent"
QT_MOC_LITERAL(74, 1127, 12), // "savePathSlot"
QT_MOC_LITERAL(75, 1140, 5), // "state"
QT_MOC_LITERAL(76, 1146, 14), // "cancelPathSlot"
QT_MOC_LITERAL(77, 1161, 16), // "addPointPathSlot"
QT_MOC_LITERAL(78, 1178, 20), // "displayPointsInGroup"
QT_MOC_LITERAL(79, 1199, 30), // "removePointFromInformationMenu"
QT_MOC_LITERAL(80, 1230, 20), // "displayPointMapEvent"
QT_MOC_LITERAL(81, 1251, 20), // "editPointButtonEvent"
QT_MOC_LITERAL(82, 1272, 20), // "editTmpPathPointSlot"
QT_MOC_LITERAL(83, 1293, 2), // "id"
QT_MOC_LITERAL(84, 1296, 22), // "editPointFromGroupMenu"
QT_MOC_LITERAL(85, 1319, 21), // "saveEditPathPointSlot"
QT_MOC_LITERAL(86, 1341, 23), // "cancelEditPathPointSlot"
QT_MOC_LITERAL(87, 1365, 23), // "moveEditedPathPointSlot"
QT_MOC_LITERAL(88, 1389, 29), // "displayPointInfoFromGroupMenu"
QT_MOC_LITERAL(89, 1419, 11), // "updatePoint"
QT_MOC_LITERAL(90, 1431, 17), // "updateCoordinates"
QT_MOC_LITERAL(91, 1449, 24), // "removePointFromGroupMenu"
QT_MOC_LITERAL(92, 1474, 25), // "displayPointFromGroupMenu"
QT_MOC_LITERAL(93, 1500, 18), // "doubleClickOnPoint"
QT_MOC_LITERAL(94, 1519, 9), // "checkedId"
QT_MOC_LITERAL(95, 1529, 18), // "doubleClickOnGroup"
QT_MOC_LITERAL(96, 1548, 23), // "doubleClickOnPathsGroup"
QT_MOC_LITERAL(97, 1572, 13), // "checkedButton"
QT_MOC_LITERAL(98, 1586, 28), // "reestablishConnectionsGroups"
QT_MOC_LITERAL(99, 1615, 28), // "reestablishConnectionsPoints"
QT_MOC_LITERAL(100, 1644, 11), // "createGroup"
QT_MOC_LITERAL(101, 1656, 20), // "modifyGroupWithEnter"
QT_MOC_LITERAL(102, 1677, 21), // "modifyGroupAfterClick"
QT_MOC_LITERAL(103, 1699, 27), // "enableReturnAndCloseButtons"
QT_MOC_LITERAL(104, 1727, 18), // "doubleClickOnRobot"
QT_MOC_LITERAL(105, 1746, 22), // "setMessageCreationPath"
QT_MOC_LITERAL(106, 1769, 7), // "message"
QT_MOC_LITERAL(107, 1777, 21), // "updateEditedPathPoint"
QT_MOC_LITERAL(108, 1799, 9), // "centerMap"
QT_MOC_LITERAL(109, 1809, 23), // "setMessageCreationPoint"
QT_MOC_LITERAL(110, 1833, 4), // "type"
QT_MOC_LITERAL(111, 1838, 24), // "CreatePointWidget::Error"
QT_MOC_LITERAL(112, 1863, 5), // "error"
QT_MOC_LITERAL(113, 1869, 15), // "choosePointName"
QT_MOC_LITERAL(114, 1885, 12), // "saveMapState"
QT_MOC_LITERAL(115, 1898, 11), // "cancelEvent"
QT_MOC_LITERAL(116, 1910, 13), // "setMessageTop"
QT_MOC_LITERAL(117, 1924, 7), // "msgType"
QT_MOC_LITERAL(118, 1932, 3), // "msg"
QT_MOC_LITERAL(119, 1936, 14), // "setLastMessage"
QT_MOC_LITERAL(120, 1951, 23), // "setMessageCreationGroup"
QT_MOC_LITERAL(121, 1975, 14), // "goHomeBtnEvent"
QT_MOC_LITERAL(122, 1990, 21), // "viewPathSelectedRobot"
QT_MOC_LITERAL(123, 2012, 7), // "checked"
QT_MOC_LITERAL(124, 2020, 13), // "editHomeEvent"
QT_MOC_LITERAL(125, 2034, 9), // "closeSlot"
QT_MOC_LITERAL(126, 2044, 20), // "setGraphicItemsState"
QT_MOC_LITERAL(127, 2065, 8), // "showHome"
QT_MOC_LITERAL(128, 2074, 12), // "showEditHome"
QT_MOC_LITERAL(129, 2087, 12), // "showAllHomes"
QT_MOC_LITERAL(130, 2100, 9), // "backEvent"
QT_MOC_LITERAL(131, 2110, 10), // "updateView"
QT_MOC_LITERAL(132, 2121, 16), // "robotIsAliveSlot"
QT_MOC_LITERAL(133, 2138, 8), // "hostname"
QT_MOC_LITERAL(134, 2147, 2), // "ip"
QT_MOC_LITERAL(135, 2150, 5), // "mapId"
QT_MOC_LITERAL(136, 2156, 4), // "ssid"
QT_MOC_LITERAL(137, 2161, 5), // "stage"
QT_MOC_LITERAL(138, 2167, 15), // "robotIsDeadSlot"
QT_MOC_LITERAL(139, 2183, 15), // "selectViewRobot"
QT_MOC_LITERAL(140, 2199, 18), // "sendNewMapToRobots"
QT_MOC_LITERAL(141, 2218, 17), // "sendNewMapToRobot"
QT_MOC_LITERAL(142, 2236, 21), // "QSharedPointer<Robot>"
QT_MOC_LITERAL(143, 2258, 5), // "robot"
QT_MOC_LITERAL(144, 2264, 14), // "settingBtnSlot"
QT_MOC_LITERAL(145, 2279, 30), // "updatePathPainterPointViewSlot"
QT_MOC_LITERAL(146, 2310, 8), // "stopPath"
QT_MOC_LITERAL(147, 2319, 23), // "resetPathPointViewsSlot"
QT_MOC_LITERAL(148, 2343, 12), // "setEnableAll"
QT_MOC_LITERAL(149, 2356, 6), // "enable"
QT_MOC_LITERAL(150, 2363, 8), // "noReturn"
QT_MOC_LITERAL(151, 2372, 28), // "deletePathSelecRobotBtnEvent"
QT_MOC_LITERAL(152, 2401, 12), // "replacePoint"
QT_MOC_LITERAL(153, 2414, 14), // "deletePathSlot"
QT_MOC_LITERAL(154, 2429, 8), // "pathName"
QT_MOC_LITERAL(155, 2438, 12), // "editPathSlot"
QT_MOC_LITERAL(156, 2451, 15), // "displayPathSlot"
QT_MOC_LITERAL(157, 2467, 7), // "display"
QT_MOC_LITERAL(158, 2475, 10), // "setNewHome"
QT_MOC_LITERAL(159, 2486, 8), // "homeName"
QT_MOC_LITERAL(160, 2495, 10), // "deleteHome"
QT_MOC_LITERAL(161, 2506, 17), // "displayGroupPaths"
QT_MOC_LITERAL(162, 2524, 14), // "editGroupPaths"
QT_MOC_LITERAL(163, 2539, 16), // "createGroupPaths"
QT_MOC_LITERAL(164, 2556, 16), // "deleteGroupPaths"
QT_MOC_LITERAL(165, 2573, 14), // "saveGroupPaths"
QT_MOC_LITERAL(166, 2588, 25), // "modifyGroupPathsWithEnter"
QT_MOC_LITERAL(167, 2614, 11), // "displayPath"
QT_MOC_LITERAL(168, 2626, 10), // "createPath"
QT_MOC_LITERAL(169, 2637, 16), // "displayPathOnMap"
QT_MOC_LITERAL(170, 2654, 8), // "editPath"
QT_MOC_LITERAL(171, 2663, 17), // "doubleClickOnPath"
QT_MOC_LITERAL(172, 2681, 21), // "setMessageNoRobotPath"
QT_MOC_LITERAL(173, 2703, 4), // "code"
QT_MOC_LITERAL(174, 2708, 30), // "cancelEditNoRobotPathPointSlot"
QT_MOC_LITERAL(175, 2739, 21), // "cancelNoRobotPathSlot"
QT_MOC_LITERAL(176, 2761, 19), // "saveNoRobotPathSlot"
QT_MOC_LITERAL(177, 2781, 25), // "setMessageModifGroupPaths"
QT_MOC_LITERAL(178, 2807, 19), // "displayAssignedPath"
QT_MOC_LITERAL(179, 2827, 15) // "clearMapOfPaths"

    },
    "MainWindow\0sendCommand\0\0nameChanged\0"
    "changeCmdThreadRobotName\0addPathPoint\0"
    "name\0x\0y\0GraphicItemState\0addNoRobotPathPoint\0"
    "updatePathPainter\0updatePathPainterPointView\0"
    "resetPath\0resetPathCreationWidget\0"
    "updateRobot\0ipAddress\0posX\0posY\0ori\0"
    "updateMetadata\0width\0height\0resolution\0"
    "originX\0originY\0updateMap\0mapArray\0"
    "connectToRobot\0quit\0setSelectedRobot\0"
    "RobotView*\0robotView\0editSelectedRobot\0"
    "QAbstractButton*\0button\0"
    "setSelectedRobotNoParent\0"
    "setSelectedRobotFromPointSlot\0robotName\0"
    "robotBtnEvent\0pointBtnEvent\0mapBtnEvent\0"
    "pathBtnEvent\0plusGroupBtnEvent\0"
    "minusGroupBtnEvent\0editGroupBtnEvent\0"
    "openLeftMenu\0backSelecRobotBtnEvent\0"
    "editSelecRobotBtnEvent\0addPathSelecRobotBtnEvent\0"
    "backRobotBtnEvent\0editRobotBtnEvent\0"
    "checkRobotBtnEventMenu\0checkRobotBtnEventSelect\0"
    "checkRobotBtnEvent\0saveMapBtnEvent\0"
    "loadMapBtnEvent\0cancelEditSelecRobotBtnEvent\0"
    "robotSavedEvent\0minusSelecPointBtnEvent\0"
    "editSelecPointBtnEvent\0setSelectedPoint\0"
    "pointSavedEvent\0groupName\0deletePath\0"
    "robotNb\0playSelectedRobot\0"
    "askForDeleteGroupConfirmation\0group\0"
    "askForDeletePointConfirmation\0index\0"
    "displayPointEvent\0"
    "askForDeleteDefaultGroupPointConfirmation\0"
    "displayGroupMapEvent\0savePathSlot\0"
    "state\0cancelPathSlot\0addPointPathSlot\0"
    "displayPointsInGroup\0"
    "removePointFromInformationMenu\0"
    "displayPointMapEvent\0editPointButtonEvent\0"
    "editTmpPathPointSlot\0id\0editPointFromGroupMenu\0"
    "saveEditPathPointSlot\0cancelEditPathPointSlot\0"
    "moveEditedPathPointSlot\0"
    "displayPointInfoFromGroupMenu\0updatePoint\0"
    "updateCoordinates\0removePointFromGroupMenu\0"
    "displayPointFromGroupMenu\0doubleClickOnPoint\0"
    "checkedId\0doubleClickOnGroup\0"
    "doubleClickOnPathsGroup\0checkedButton\0"
    "reestablishConnectionsGroups\0"
    "reestablishConnectionsPoints\0createGroup\0"
    "modifyGroupWithEnter\0modifyGroupAfterClick\0"
    "enableReturnAndCloseButtons\0"
    "doubleClickOnRobot\0setMessageCreationPath\0"
    "message\0updateEditedPathPoint\0centerMap\0"
    "setMessageCreationPoint\0type\0"
    "CreatePointWidget::Error\0error\0"
    "choosePointName\0saveMapState\0cancelEvent\0"
    "setMessageTop\0msgType\0msg\0setLastMessage\0"
    "setMessageCreationGroup\0goHomeBtnEvent\0"
    "viewPathSelectedRobot\0checked\0"
    "editHomeEvent\0closeSlot\0setGraphicItemsState\0"
    "showHome\0showEditHome\0showAllHomes\0"
    "backEvent\0updateView\0robotIsAliveSlot\0"
    "hostname\0ip\0mapId\0ssid\0stage\0"
    "robotIsDeadSlot\0selectViewRobot\0"
    "sendNewMapToRobots\0sendNewMapToRobot\0"
    "QSharedPointer<Robot>\0robot\0settingBtnSlot\0"
    "updatePathPainterPointViewSlot\0stopPath\0"
    "resetPathPointViewsSlot\0setEnableAll\0"
    "enable\0noReturn\0deletePathSelecRobotBtnEvent\0"
    "replacePoint\0deletePathSlot\0pathName\0"
    "editPathSlot\0displayPathSlot\0display\0"
    "setNewHome\0homeName\0deleteHome\0"
    "displayGroupPaths\0editGroupPaths\0"
    "createGroupPaths\0deleteGroupPaths\0"
    "saveGroupPaths\0modifyGroupPathsWithEnter\0"
    "displayPath\0createPath\0displayPathOnMap\0"
    "editPath\0doubleClickOnPath\0"
    "setMessageNoRobotPath\0code\0"
    "cancelEditNoRobotPathPointSlot\0"
    "cancelNoRobotPathSlot\0saveNoRobotPathSlot\0"
    "setMessageModifGroupPaths\0displayAssignedPath\0"
    "clearMapOfPaths"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
     135,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  689,    2, 0x06 /* Public */,
       3,    2,  692,    2, 0x06 /* Public */,
       4,    1,  697,    2, 0x06 /* Public */,
       5,    4,  700,    2, 0x06 /* Public */,
      10,    3,  709,    2, 0x06 /* Public */,
      11,    2,  716,    2, 0x06 /* Public */,
      12,    1,  721,    2, 0x06 /* Public */,
      13,    1,  724,    2, 0x06 /* Public */,
      14,    1,  727,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      15,    4,  730,    2, 0x08 /* Private */,
      20,    5,  739,    2, 0x08 /* Private */,
      26,    1,  750,    2, 0x08 /* Private */,
      28,    0,  753,    2, 0x08 /* Private */,
      29,    0,  754,    2, 0x08 /* Private */,
      30,    1,  755,    2, 0x08 /* Private */,
      33,    1,  758,    2, 0x08 /* Private */,
      30,    1,  761,    2, 0x08 /* Private */,
      36,    1,  764,    2, 0x08 /* Private */,
      37,    1,  767,    2, 0x08 /* Private */,
      39,    0,  770,    2, 0x08 /* Private */,
      40,    0,  771,    2, 0x08 /* Private */,
      41,    0,  772,    2, 0x08 /* Private */,
      42,    0,  773,    2, 0x08 /* Private */,
      43,    0,  774,    2, 0x08 /* Private */,
      44,    0,  775,    2, 0x08 /* Private */,
      45,    0,  776,    2, 0x08 /* Private */,
      46,    0,  777,    2, 0x08 /* Private */,
      47,    0,  778,    2, 0x08 /* Private */,
      48,    0,  779,    2, 0x08 /* Private */,
      49,    0,  780,    2, 0x08 /* Private */,
      50,    0,  781,    2, 0x08 /* Private */,
      51,    0,  782,    2, 0x08 /* Private */,
      52,    0,  783,    2, 0x08 /* Private */,
      53,    0,  784,    2, 0x08 /* Private */,
      54,    1,  785,    2, 0x08 /* Private */,
      55,    0,  788,    2, 0x08 /* Private */,
      56,    0,  789,    2, 0x08 /* Private */,
      57,    0,  790,    2, 0x08 /* Private */,
      58,    0,  791,    2, 0x08 /* Private */,
      59,    0,  792,    2, 0x08 /* Private */,
      60,    0,  793,    2, 0x08 /* Private */,
      61,    0,  794,    2, 0x08 /* Private */,
      62,    4,  795,    2, 0x08 /* Private */,
      64,    1,  804,    2, 0x08 /* Private */,
      66,    1,  807,    2, 0x08 /* Private */,
      67,    1,  810,    2, 0x08 /* Private */,
      69,    1,  813,    2, 0x08 /* Private */,
      71,    3,  816,    2, 0x08 /* Private */,
      72,    1,  823,    2, 0x08 /* Private */,
      73,    0,  826,    2, 0x08 /* Private */,
      74,    1,  827,    2, 0x08 /* Private */,
      76,    0,  830,    2, 0x08 /* Private */,
      77,    4,  831,    2, 0x08 /* Private */,
      78,    0,  840,    2, 0x08 /* Private */,
      79,    0,  841,    2, 0x08 /* Private */,
      80,    0,  842,    2, 0x08 /* Private */,
      81,    0,  843,    2, 0x08 /* Private */,
      82,    5,  844,    2, 0x08 /* Private */,
      84,    0,  855,    2, 0x08 /* Private */,
      85,    1,  856,    2, 0x08 /* Private */,
      86,    1,  859,    2, 0x08 /* Private */,
      87,    1,  862,    2, 0x08 /* Private */,
      88,    0,  865,    2, 0x08 /* Private */,
      89,    0,  866,    2, 0x08 /* Private */,
      90,    2,  867,    2, 0x08 /* Private */,
      91,    0,  872,    2, 0x08 /* Private */,
      92,    0,  873,    2, 0x08 /* Private */,
      93,    1,  874,    2, 0x08 /* Private */,
      95,    1,  877,    2, 0x08 /* Private */,
      96,    1,  880,    2, 0x08 /* Private */,
      98,    0,  883,    2, 0x08 /* Private */,
      99,    0,  884,    2, 0x08 /* Private */,
     100,    1,  885,    2, 0x08 /* Private */,
     101,    1,  888,    2, 0x08 /* Private */,
     102,    1,  891,    2, 0x08 /* Private */,
     103,    0,  894,    2, 0x08 /* Private */,
     104,    1,  895,    2, 0x08 /* Private */,
     105,    1,  898,    2, 0x08 /* Private */,
     107,    3,  901,    2, 0x08 /* Private */,
     108,    0,  908,    2, 0x08 /* Private */,
     109,    2,  909,    2, 0x08 /* Private */,
     113,    1,  914,    2, 0x08 /* Private */,
     114,    0,  917,    2, 0x08 /* Private */,
     115,    0,  918,    2, 0x08 /* Private */,
     116,    2,  919,    2, 0x08 /* Private */,
     119,    0,  924,    2, 0x08 /* Private */,
     120,    2,  925,    2, 0x08 /* Private */,
     121,    0,  930,    2, 0x08 /* Private */,
     122,    2,  931,    2, 0x08 /* Private */,
     124,    0,  936,    2, 0x08 /* Private */,
     125,    0,  937,    2, 0x08 /* Private */,
     126,    1,  938,    2, 0x08 /* Private */,
     127,    0,  941,    2, 0x08 /* Private */,
     128,    0,  942,    2, 0x08 /* Private */,
     129,    0,  943,    2, 0x08 /* Private */,
     130,    0,  944,    2, 0x08 /* Private */,
     131,    0,  945,    2, 0x08 /* Private */,
     132,    5,  946,    2, 0x08 /* Private */,
     138,    2,  957,    2, 0x08 /* Private */,
     139,    0,  962,    2, 0x08 /* Private */,
     140,    1,  963,    2, 0x08 /* Private */,
     141,    2,  966,    2, 0x08 /* Private */,
     144,    0,  971,    2, 0x08 /* Private */,
     145,    0,  972,    2, 0x08 /* Private */,
     146,    1,  973,    2, 0x08 /* Private */,
     147,    0,  976,    2, 0x08 /* Private */,
     148,    3,  977,    2, 0x08 /* Private */,
     148,    2,  984,    2, 0x28 /* Private | MethodCloned */,
     148,    1,  989,    2, 0x28 /* Private | MethodCloned */,
     151,    0,  992,    2, 0x08 /* Private */,
     152,    2,  993,    2, 0x08 /* Private */,
     153,    2,  998,    2, 0x08 /* Private */,
     155,    2, 1003,    2, 0x08 /* Private */,
     156,    3, 1008,    2, 0x08 /* Private */,
     158,    1, 1015,    2, 0x08 /* Private */,
     160,    0, 1018,    2, 0x08 /* Private */,
     161,    0, 1019,    2, 0x08 /* Private */,
     162,    0, 1020,    2, 0x08 /* Private */,
     163,    0, 1021,    2, 0x08 /* Private */,
     164,    0, 1022,    2, 0x08 /* Private */,
     165,    1, 1023,    2, 0x08 /* Private */,
     166,    1, 1026,    2, 0x08 /* Private */,
     167,    0, 1029,    2, 0x08 /* Private */,
     168,    0, 1030,    2, 0x08 /* Private */,
      64,    0, 1031,    2, 0x08 /* Private */,
     169,    1, 1032,    2, 0x08 /* Private */,
     170,    0, 1035,    2, 0x08 /* Private */,
     171,    1, 1036,    2, 0x08 /* Private */,
     172,    1, 1039,    2, 0x08 /* Private */,
     174,    0, 1042,    2, 0x08 /* Private */,
     175,    0, 1043,    2, 0x08 /* Private */,
     176,    0, 1044,    2, 0x08 /* Private */,
     177,    1, 1045,    2, 0x08 /* Private */,
     178,    2, 1048,    2, 0x08 /* Private */,
     179,    0, 1053,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    2,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double, 0x80000000 | 9,    6,    7,    8,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double,    6,    7,    8,
    QMetaType::Void, 0x80000000 | 9, QMetaType::Bool,    2,    2,
    QMetaType::Void, 0x80000000 | 9,    2,
    QMetaType::Void, 0x80000000 | 9,    2,
    QMetaType::Void, 0x80000000 | 9,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Float, QMetaType::Float, QMetaType::Float,   16,   17,   18,   19,
    QMetaType::Void, QMetaType::Int, QMetaType::Int, QMetaType::Float, QMetaType::Float, QMetaType::Float,   21,   22,   23,   24,   25,
    QMetaType::Void, QMetaType::QByteArray,   27,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 31,   32,
    QMetaType::Void, 0x80000000 | 31,   32,
    QMetaType::Void, 0x80000000 | 34,   35,
    QMetaType::Void, 0x80000000 | 34,   35,
    QMetaType::Void, QMetaType::QString,   38,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double, QMetaType::QString,   63,    7,    8,    6,
    QMetaType::Void, QMetaType::Int,   65,
    QMetaType::Void, QMetaType::Int,   65,
    QMetaType::Void, QMetaType::QString,   68,
    QMetaType::Void, QMetaType::QString,   70,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double,    6,    7,    8,
    QMetaType::Void, QMetaType::QString,   70,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   75,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::Double, QMetaType::Double, 0x80000000 | 9,    6,    7,    8,   75,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::QString, QMetaType::Double, QMetaType::Double, 0x80000000 | 9,   83,    6,    7,    8,   75,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   75,
    QMetaType::Void, 0x80000000 | 9,   75,
    QMetaType::Void, 0x80000000 | 9,   75,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    7,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   94,
    QMetaType::Void, QMetaType::QString,   94,
    QMetaType::Void, QMetaType::QString,   97,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   94,
    QMetaType::Void, QMetaType::QString,  106,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, 0x80000000 | 9,    7,    8,   75,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 111,  110,  112,
    QMetaType::Void, QMetaType::QString,  106,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,  117,  118,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,  110,  106,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Bool,   65,  123,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   75,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::QString, QMetaType::QString, QMetaType::Int,  133,  134,  135,  136,  137,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,  133,  134,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   16,
    QMetaType::Void, 0x80000000 | 142, QMetaType::QString,  143,  135,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   65,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool, 0x80000000 | 9, QMetaType::Int,  149,   75,  150,
    QMetaType::Void, QMetaType::Bool, 0x80000000 | 9,  149,   75,
    QMetaType::Void, QMetaType::Bool,  149,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,   83,    6,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   63,  154,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   63,  154,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::Bool,   63,  154,  157,
    QMetaType::Void, QMetaType::QString,  159,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,  157,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,  154,
    QMetaType::Void, QMetaType::Int,  173,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,  173,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   63,  154,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendCommand((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->nameChanged((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 2: _t->changeCmdThreadRobotName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->addPathPoint((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< GraphicItemState(*)>(_a[4]))); break;
        case 4: _t->addNoRobotPathPoint((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 5: _t->updatePathPainter((*reinterpret_cast< GraphicItemState(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 6: _t->updatePathPainterPointView((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 7: _t->resetPath((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 8: _t->resetPathCreationWidget((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 9: _t->updateRobot((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const float(*)>(_a[2])),(*reinterpret_cast< const float(*)>(_a[3])),(*reinterpret_cast< const float(*)>(_a[4]))); break;
        case 10: _t->updateMetadata((*reinterpret_cast< const int(*)>(_a[1])),(*reinterpret_cast< const int(*)>(_a[2])),(*reinterpret_cast< const float(*)>(_a[3])),(*reinterpret_cast< const float(*)>(_a[4])),(*reinterpret_cast< const float(*)>(_a[5]))); break;
        case 11: _t->updateMap((*reinterpret_cast< const QByteArray(*)>(_a[1]))); break;
        case 12: _t->connectToRobot(); break;
        case 13: _t->quit(); break;
        case 14: _t->setSelectedRobot((*reinterpret_cast< RobotView*(*)>(_a[1]))); break;
        case 15: _t->editSelectedRobot((*reinterpret_cast< RobotView*(*)>(_a[1]))); break;
        case 16: _t->setSelectedRobot((*reinterpret_cast< QAbstractButton*(*)>(_a[1]))); break;
        case 17: _t->setSelectedRobotNoParent((*reinterpret_cast< QAbstractButton*(*)>(_a[1]))); break;
        case 18: _t->setSelectedRobotFromPointSlot((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 19: _t->robotBtnEvent(); break;
        case 20: _t->pointBtnEvent(); break;
        case 21: _t->mapBtnEvent(); break;
        case 22: _t->pathBtnEvent(); break;
        case 23: _t->plusGroupBtnEvent(); break;
        case 24: _t->minusGroupBtnEvent(); break;
        case 25: _t->editGroupBtnEvent(); break;
        case 26: _t->openLeftMenu(); break;
        case 27: _t->backSelecRobotBtnEvent(); break;
        case 28: _t->editSelecRobotBtnEvent(); break;
        case 29: _t->addPathSelecRobotBtnEvent(); break;
        case 30: _t->backRobotBtnEvent(); break;
        case 31: _t->editRobotBtnEvent(); break;
        case 32: _t->checkRobotBtnEventMenu(); break;
        case 33: _t->checkRobotBtnEventSelect(); break;
        case 34: _t->checkRobotBtnEvent((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 35: _t->saveMapBtnEvent(); break;
        case 36: _t->loadMapBtnEvent(); break;
        case 37: _t->cancelEditSelecRobotBtnEvent(); break;
        case 38: _t->robotSavedEvent(); break;
        case 39: _t->minusSelecPointBtnEvent(); break;
        case 40: _t->editSelecPointBtnEvent(); break;
        case 41: _t->setSelectedPoint(); break;
        case 42: _t->pointSavedEvent((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 43: _t->deletePath((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 44: _t->playSelectedRobot((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 45: _t->askForDeleteGroupConfirmation((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 46: _t->askForDeletePointConfirmation((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 47: _t->displayPointEvent((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 48: _t->askForDeleteDefaultGroupPointConfirmation((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 49: _t->displayGroupMapEvent(); break;
        case 50: _t->savePathSlot((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 51: _t->cancelPathSlot(); break;
        case 52: _t->addPointPathSlot((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< GraphicItemState(*)>(_a[4]))); break;
        case 53: _t->displayPointsInGroup(); break;
        case 54: _t->removePointFromInformationMenu(); break;
        case 55: _t->displayPointMapEvent(); break;
        case 56: _t->editPointButtonEvent(); break;
        case 57: _t->editTmpPathPointSlot((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< GraphicItemState(*)>(_a[5]))); break;
        case 58: _t->editPointFromGroupMenu(); break;
        case 59: _t->saveEditPathPointSlot((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 60: _t->cancelEditPathPointSlot((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 61: _t->moveEditedPathPointSlot((*reinterpret_cast< GraphicItemState(*)>(_a[1]))); break;
        case 62: _t->displayPointInfoFromGroupMenu(); break;
        case 63: _t->updatePoint(); break;
        case 64: _t->updateCoordinates((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 65: _t->removePointFromGroupMenu(); break;
        case 66: _t->displayPointFromGroupMenu(); break;
        case 67: _t->doubleClickOnPoint((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 68: _t->doubleClickOnGroup((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 69: _t->doubleClickOnPathsGroup((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 70: _t->reestablishConnectionsGroups(); break;
        case 71: _t->reestablishConnectionsPoints(); break;
        case 72: _t->createGroup((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 73: _t->modifyGroupWithEnter((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 74: _t->modifyGroupAfterClick((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 75: _t->enableReturnAndCloseButtons(); break;
        case 76: _t->doubleClickOnRobot((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 77: _t->setMessageCreationPath((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 78: _t->updateEditedPathPoint((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< GraphicItemState(*)>(_a[3]))); break;
        case 79: _t->centerMap(); break;
        case 80: _t->setMessageCreationPoint((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< CreatePointWidget::Error(*)>(_a[2]))); break;
        case 81: _t->choosePointName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 82: _t->saveMapState(); break;
        case 83: _t->cancelEvent(); break;
        case 84: _t->setMessageTop((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 85: _t->setLastMessage(); break;
        case 86: _t->setMessageCreationGroup((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 87: _t->goHomeBtnEvent(); break;
        case 88: _t->viewPathSelectedRobot((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 89: _t->editHomeEvent(); break;
        case 90: _t->closeSlot(); break;
        case 91: _t->setGraphicItemsState((*reinterpret_cast< const GraphicItemState(*)>(_a[1]))); break;
        case 92: _t->showHome(); break;
        case 93: _t->showEditHome(); break;
        case 94: _t->showAllHomes(); break;
        case 95: _t->backEvent(); break;
        case 96: _t->updateView(); break;
        case 97: _t->robotIsAliveSlot((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5]))); break;
        case 98: _t->robotIsDeadSlot((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 99: _t->selectViewRobot(); break;
        case 100: _t->sendNewMapToRobots((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 101: _t->sendNewMapToRobot((*reinterpret_cast< QSharedPointer<Robot>(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 102: _t->settingBtnSlot(); break;
        case 103: _t->updatePathPainterPointViewSlot(); break;
        case 104: _t->stopPath((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 105: _t->resetPathPointViewsSlot(); break;
        case 106: _t->setEnableAll((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< GraphicItemState(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 107: _t->setEnableAll((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< GraphicItemState(*)>(_a[2]))); break;
        case 108: _t->setEnableAll((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 109: _t->deletePathSelecRobotBtnEvent(); break;
        case 110: _t->replacePoint((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 111: _t->deletePathSlot((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 112: _t->editPathSlot((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 113: _t->displayPathSlot((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 114: _t->setNewHome((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 115: _t->deleteHome(); break;
        case 116: _t->displayGroupPaths(); break;
        case 117: _t->editGroupPaths(); break;
        case 118: _t->createGroupPaths(); break;
        case 119: _t->deleteGroupPaths(); break;
        case 120: _t->saveGroupPaths((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 121: _t->modifyGroupPathsWithEnter((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 122: _t->displayPath(); break;
        case 123: _t->createPath(); break;
        case 124: _t->deletePath(); break;
        case 125: _t->displayPathOnMap((*reinterpret_cast< const bool(*)>(_a[1]))); break;
        case 126: _t->editPath(); break;
        case 127: _t->doubleClickOnPath((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 128: _t->setMessageNoRobotPath((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 129: _t->cancelEditNoRobotPathPointSlot(); break;
        case 130: _t->cancelNoRobotPathSlot(); break;
        case 131: _t->saveNoRobotPathSlot(); break;
        case 132: _t->setMessageModifGroupPaths((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 133: _t->displayAssignedPath((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 134: _t->clearMapOfPaths(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 14:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< RobotView* >(); break;
            }
            break;
        case 15:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< RobotView* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MainWindow::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::sendCommand)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::nameChanged)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::changeCmdThreadRobotName)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(QString , double , double , GraphicItemState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::addPathPoint)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(QString , double , double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::addNoRobotPathPoint)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(GraphicItemState , bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::updatePathPainter)) {
                *result = 5;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(GraphicItemState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::updatePathPainterPointView)) {
                *result = 6;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(GraphicItemState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::resetPath)) {
                *result = 7;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(GraphicItemState );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::resetPathCreationWidget)) {
                *result = 8;
                return;
            }
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 135)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 135;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 135)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 135;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::sendCommand(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MainWindow::nameChanged(QString _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MainWindow::changeCmdThreadRobotName(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MainWindow::addPathPoint(QString _t1, double _t2, double _t3, GraphicItemState _t4)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void MainWindow::addNoRobotPathPoint(QString _t1, double _t2, double _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void MainWindow::updatePathPainter(GraphicItemState _t1, bool _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void MainWindow::updatePathPainterPointView(GraphicItemState _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void MainWindow::resetPath(GraphicItemState _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void MainWindow::resetPathCreationWidget(GraphicItemState _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}
QT_END_MOC_NAMESPACE
