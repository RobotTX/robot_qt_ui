/****************************************************************************
** Meta object code from reading C++ file 'groupspathswidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "View/groupspathswidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'groupspathswidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_GroupsPathsWidget_t {
    QByteArrayData data[15];
    char stringdata0[204];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GroupsPathsWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GroupsPathsWidget_t qt_meta_stringdata_GroupsPathsWidget = {
    {
QT_MOC_LITERAL(0, 0, 17), // "GroupsPathsWidget"
QT_MOC_LITERAL(1, 18, 12), // "newPathGroup"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 20), // "messageCreationGroup"
QT_MOC_LITERAL(4, 53, 13), // "codeEditGroup"
QT_MOC_LITERAL(5, 67, 13), // "modifiedGroup"
QT_MOC_LITERAL(6, 81, 11), // "deleteGroup"
QT_MOC_LITERAL(7, 93, 14), // "checkGroupName"
QT_MOC_LITERAL(8, 108, 4), // "name"
QT_MOC_LITERAL(9, 113, 18), // "checkEditGroupName"
QT_MOC_LITERAL(10, 132, 19), // "cancelCreationGroup"
QT_MOC_LITERAL(11, 152, 13), // "enableButtons"
QT_MOC_LITERAL(12, 166, 16), // "QAbstractButton*"
QT_MOC_LITERAL(13, 183, 6), // "button"
QT_MOC_LITERAL(14, 190, 13) // "newGroupPaths"

    },
    "GroupsPathsWidget\0newPathGroup\0\0"
    "messageCreationGroup\0codeEditGroup\0"
    "modifiedGroup\0deleteGroup\0checkGroupName\0"
    "name\0checkEditGroupName\0cancelCreationGroup\0"
    "enableButtons\0QAbstractButton*\0button\0"
    "newGroupPaths"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GroupsPathsWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   64,    2, 0x06 /* Public */,
       3,    2,   67,    2, 0x06 /* Public */,
       4,    1,   72,    2, 0x06 /* Public */,
       5,    1,   75,    2, 0x06 /* Public */,
       6,    0,   78,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   79,    2, 0x0a /* Public */,
       9,    1,   82,    2, 0x0a /* Public */,
      10,    0,   85,    2, 0x0a /* Public */,
      11,    1,   86,    2, 0x08 /* Private */,
      14,    0,   89,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    2,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Int, QMetaType::QString,    8,
    QMetaType::Int, QMetaType::QString,    8,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void,

       0        // eod
};

void GroupsPathsWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GroupsPathsWidget *_t = static_cast<GroupsPathsWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newPathGroup((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->messageCreationGroup((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 2: _t->codeEditGroup((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->modifiedGroup((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->deleteGroup(); break;
        case 5: { int _r = _t->checkGroupName((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 6: { int _r = _t->checkEditGroupName((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 7: _t->cancelCreationGroup(); break;
        case 8: _t->enableButtons((*reinterpret_cast< QAbstractButton*(*)>(_a[1]))); break;
        case 9: _t->newGroupPaths(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (GroupsPathsWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GroupsPathsWidget::newPathGroup)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (GroupsPathsWidget::*_t)(QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GroupsPathsWidget::messageCreationGroup)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (GroupsPathsWidget::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GroupsPathsWidget::codeEditGroup)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (GroupsPathsWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GroupsPathsWidget::modifiedGroup)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (GroupsPathsWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GroupsPathsWidget::deleteGroup)) {
                *result = 4;
                return;
            }
        }
    }
}

const QMetaObject GroupsPathsWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_GroupsPathsWidget.data,
      qt_meta_data_GroupsPathsWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *GroupsPathsWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GroupsPathsWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_GroupsPathsWidget.stringdata0))
        return static_cast<void*>(const_cast< GroupsPathsWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int GroupsPathsWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void GroupsPathsWidget::newPathGroup(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void GroupsPathsWidget::messageCreationGroup(QString _t1, QString _t2)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void GroupsPathsWidget::codeEditGroup(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void GroupsPathsWidget::modifiedGroup(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void GroupsPathsWidget::deleteGroup()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
