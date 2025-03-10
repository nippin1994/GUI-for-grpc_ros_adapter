/****************************************************************************
** Meta object code from reading C++ file 'automode.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "automode.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'automode.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_automode_t {
    QByteArrayData data[14];
    char stringdata0[314];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_automode_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_automode_t qt_meta_stringdata_automode = {
    {
QT_MOC_LITERAL(0, 0, 8), // "automode"
QT_MOC_LITERAL(1, 9, 29), // "autoPublishValuesPeriodically"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 21), // "on_kpGain_textChanged"
QT_MOC_LITERAL(4, 62, 21), // "on_kiGain_textChanged"
QT_MOC_LITERAL(5, 84, 21), // "on_kdGain_textChanged"
QT_MOC_LITERAL(6, 106, 30), // "on_kpGain_throttle_textChanged"
QT_MOC_LITERAL(7, 137, 30), // "on_kiGain_throttle_textChanged"
QT_MOC_LITERAL(8, 168, 30), // "on_kdGain_throttle_textChanged"
QT_MOC_LITERAL(9, 199, 17), // "on_Start_released"
QT_MOC_LITERAL(10, 217, 16), // "on_Stop_released"
QT_MOC_LITERAL(11, 234, 24), // "on_addTargetPos_released"
QT_MOC_LITERAL(12, 259, 29), // "on_clearAllTargetPos_released"
QT_MOC_LITERAL(13, 289, 24) // "on_loadSavedPos_released"

    },
    "automode\0autoPublishValuesPeriodically\0"
    "\0on_kpGain_textChanged\0on_kiGain_textChanged\0"
    "on_kdGain_textChanged\0"
    "on_kpGain_throttle_textChanged\0"
    "on_kiGain_throttle_textChanged\0"
    "on_kdGain_throttle_textChanged\0"
    "on_Start_released\0on_Stop_released\0"
    "on_addTargetPos_released\0"
    "on_clearAllTargetPos_released\0"
    "on_loadSavedPos_released"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_automode[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x08 /* Private */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    0,   85,    2, 0x08 /* Private */,

 // slots: parameters
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

       0        // eod
};

void automode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<automode *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->autoPublishValuesPeriodically(); break;
        case 1: _t->on_kpGain_textChanged(); break;
        case 2: _t->on_kiGain_textChanged(); break;
        case 3: _t->on_kdGain_textChanged(); break;
        case 4: _t->on_kpGain_throttle_textChanged(); break;
        case 5: _t->on_kiGain_throttle_textChanged(); break;
        case 6: _t->on_kdGain_throttle_textChanged(); break;
        case 7: _t->on_Start_released(); break;
        case 8: _t->on_Stop_released(); break;
        case 9: _t->on_addTargetPos_released(); break;
        case 10: _t->on_clearAllTargetPos_released(); break;
        case 11: _t->on_loadSavedPos_released(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject automode::staticMetaObject = { {
    &QDialog::staticMetaObject,
    qt_meta_stringdata_automode.data,
    qt_meta_data_automode,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *automode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *automode::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_automode.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int automode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
