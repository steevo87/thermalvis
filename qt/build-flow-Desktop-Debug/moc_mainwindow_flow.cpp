/****************************************************************************
** Meta object code from reading C++ file 'mainwindow_flow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../flow/mainwindow_flow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow_flow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow_flow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      27,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      25,   17,   16,   16, 0x08,
      52,   17,   16,   16, 0x08,
      81,   17,   16,   16, 0x08,
     115,   17,   16,   16, 0x08,
     147,   17,   16,   16, 0x08,
     183,   17,   16,   16, 0x08,
     226,   17,   16,   16, 0x08,
     263,   17,   16,   16, 0x08,
     296,   17,   16,   16, 0x08,
     330,   16,   16,   16, 0x08,
     361,   16,   16,   16, 0x08,
     392,   16,   16,   16, 0x08,
     432,  426,   16,   16, 0x08,
     473,   16,   16,   16, 0x08,
     500,   16,   16,   16, 0x08,
     533,   16,   16,   16, 0x08,
     564,   16,   16,   16, 0x08,
     601,   16,   16,   16, 0x08,
     633,   16,   16,   16, 0x08,
     666,   16,   16,   16, 0x08,
     699,   16,   16,   16, 0x08,
     732,   16,   16,   16, 0x08,
     764,   16,   16,   16, 0x08,
     796,  426,   16,   16, 0x08,
     835,  426,   16,   16, 0x08,
     874,  426,   16,   16, 0x08,
     913,   16,   16,   16, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow_flow[] = {
    "MainWindow_flow\0\0checked\0"
    "on_debugMode_toggled(bool)\0"
    "on_verboseMode_toggled(bool)\0"
    "on_showTrackHistory_toggled(bool)\0"
    "on_adaptiveWindow_toggled(bool)\0"
    "on_velocityPrediction_toggled(bool)\0"
    "on_attemptHistoricalRecovery_toggled(bool)\0"
    "on_autoTrackManagement_toggled(bool)\0"
    "on_attemptMatching_toggled(bool)\0"
    "on_detectEveryFrame_toggled(bool)\0"
    "on_maxFeatures_returnPressed()\0"
    "on_minFeatures_returnPressed()\0"
    "on_drawingHistory_returnPressed()\0"
    "index\0on_matchingMode_currentIndexChanged(int)\0"
    "on_maxFrac_returnPressed()\0"
    "on_minSeparation_returnPressed()\0"
    "on_maxVelocity_returnPressed()\0"
    "on_newFeaturesPeriod_returnPressed()\0"
    "on_delayTimeout_returnPressed()\0"
    "on_sensitivity_1_returnPressed()\0"
    "on_sensitivity_2_returnPressed()\0"
    "on_sensitivity_3_returnPressed()\0"
    "on_multiplier_1_returnPressed()\0"
    "on_multiplier_2_returnPressed()\0"
    "on_detector_1_currentIndexChanged(int)\0"
    "on_detector_2_currentIndexChanged(int)\0"
    "on_detector_3_currentIndexChanged(int)\0"
    "on_flowThreshold_returnPressed()\0"
};

void MainWindow_flow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow_flow *_t = static_cast<MainWindow_flow *>(_o);
        switch (_id) {
        case 0: _t->on_debugMode_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_verboseMode_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->on_showTrackHistory_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_adaptiveWindow_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->on_velocityPrediction_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_attemptHistoricalRecovery_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_autoTrackManagement_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_attemptMatching_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_detectEveryFrame_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->on_maxFeatures_returnPressed(); break;
        case 10: _t->on_minFeatures_returnPressed(); break;
        case 11: _t->on_drawingHistory_returnPressed(); break;
        case 12: _t->on_matchingMode_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->on_maxFrac_returnPressed(); break;
        case 14: _t->on_minSeparation_returnPressed(); break;
        case 15: _t->on_maxVelocity_returnPressed(); break;
        case 16: _t->on_newFeaturesPeriod_returnPressed(); break;
        case 17: _t->on_delayTimeout_returnPressed(); break;
        case 18: _t->on_sensitivity_1_returnPressed(); break;
        case 19: _t->on_sensitivity_2_returnPressed(); break;
        case 20: _t->on_sensitivity_3_returnPressed(); break;
        case 21: _t->on_multiplier_1_returnPressed(); break;
        case 22: _t->on_multiplier_2_returnPressed(); break;
        case 23: _t->on_detector_1_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 24: _t->on_detector_2_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 25: _t->on_detector_3_currentIndexChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 26: _t->on_flowThreshold_returnPressed(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow_flow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow_flow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow_flow,
      qt_meta_data_MainWindow_flow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow_flow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow_flow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow_flow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow_flow))
        return static_cast<void*>(const_cast< MainWindow_flow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow_flow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 27)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 27;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
