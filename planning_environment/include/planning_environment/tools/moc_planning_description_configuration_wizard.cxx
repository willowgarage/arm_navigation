/****************************************************************************
** Meta object code from reading C++ file 'planning_description_configuration_wizard.h'
**
** Created: Thu Jun 16 10:16:16 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "planning_description_configuration_wizard.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'planning_description_configuration_wizard.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PlanningDescriptionConfigurationWizard[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      41,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      49,   40,   39,   39, 0x05,
      74,   69,   39,   39, 0x05,

 // slots: signature, parameters, type, tag, flags
     122,  116,   99,   39, 0x0a,
     164,  153,   39,   39, 0x0a,
     188,  153,   39,   39, 0x0a,
     212,   69,   39,   39, 0x0a,
     238,  153,   39,   39, 0x0a,
     266,  153,   39,   39, 0x0a,
     290,  153,   39,   39, 0x0a,
     316,  153,   39,   39, 0x0a,
     340,  153,   39,   39, 0x0a,
     381,  368,   39,   39, 0x0a,
     412,  116,   39,   39, 0x2a,
     439,   39,   39,   39, 0x0a,
     461,   39,   39,   39, 0x0a,
     481,   39,   39,   39, 0x0a,
     508,   39,   39,   39, 0x0a,
     526,   39,   39,   39, 0x0a,
     553,   39,   39,   39, 0x0a,
     582,   39,   39,   39, 0x0a,
     609,   39,   39,   39, 0x0a,
     630,   39,   39,   39, 0x0a,
     651,   39,   39,   39, 0x0a,
     671,   39,   39,   39, 0x0a,
     690,   39,   39,   39, 0x0a,
     708,   39,   39,   39, 0x0a,
     728,   39,   39,   39, 0x0a,
     750,   39,   39,   39, 0x0a,
     770,   39,   39,   39, 0x0a,
     797,   39,   39,   39, 0x0a,
     824,   39,   39,   39, 0x0a,
     853,   39,   39,   39, 0x0a,
     884,   39,   39,   39, 0x0a,
     920,   39,   39,   39, 0x0a,
     959,   39,   39,   39, 0x0a,
     991,   39,   39,   39, 0x0a,
    1024,   39,   39,   39, 0x0a,
    1063, 1058,   39,   39, 0x0a,
    1085,   39,   39,   39, 0x0a,
    1098,   39,   39,   39, 0x0a,
    1114,   39,   39,   39, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PlanningDescriptionConfigurationWizard[] = {
    "PlanningDescriptionConfigurationWizard\0"
    "\0progress\0changeProgress(int)\0name\0"
    "changeLabel(const char*)\0std::vector<int>\0"
    "table\0getSelectedRows(QTableWidget*)\0"
    "checkState\0easyButtonToggled(bool)\0"
    "hardButtonToggled(bool)\0"
    "labelChanged(const char*)\0"
    "verySafeButtonToggled(bool)\0"
    "safeButtonToggled(bool)\0"
    "normalButtonToggled(bool)\0"
    "fastButtonToggled(bool)\0"
    "veryFastButtonToggled(bool)\0table,column\0"
    "toggleTable(QTableWidget*,int)\0"
    "toggleTable(QTableWidget*)\0"
    "defaultTogglePushed()\0oftenTogglePushed()\0"
    "occasionallyTogglePushed()\0dofTogglePushed()\0"
    "selectJointButtonClicked()\0"
    "deselectJointButtonClicked()\0"
    "deleteGroupButtonClicked()\0"
    "acceptChainClicked()\0acceptGroupClicked()\0"
    "baseLinkTreeClick()\0tipLinkTreeClick()\0"
    "validateDoneBox()\0groupTableClicked()\0"
    "defaultTableClicked()\0oftenTableClicked()\0"
    "occasionallyTableClicked()\0"
    "dofSelectionTableChanged()\0"
    "oftenCollisionTableChanged()\0"
    "defaultCollisionTableChanged()\0"
    "occasionallyCollisionTableChanged()\0"
    "generateOccasionallyInCollisionTable()\0"
    "generateOftenInCollisionTable()\0"
    "generateAlwaysInCollisionTable()\0"
    "generateDefaultInCollisionTable()\0"
    "file\0fileSelected(QString)\0writeFiles()\0"
    "autoConfigure()\0update()\0"
};

const QMetaObject PlanningDescriptionConfigurationWizard::staticMetaObject = {
    { &QWizard::staticMetaObject, qt_meta_stringdata_PlanningDescriptionConfigurationWizard,
      qt_meta_data_PlanningDescriptionConfigurationWizard, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PlanningDescriptionConfigurationWizard::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PlanningDescriptionConfigurationWizard::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PlanningDescriptionConfigurationWizard::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlanningDescriptionConfigurationWizard))
        return static_cast<void*>(const_cast< PlanningDescriptionConfigurationWizard*>(this));
    return QWizard::qt_metacast(_clname);
}

int PlanningDescriptionConfigurationWizard::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizard::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: changeProgress((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: changeLabel((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 2: { std::vector<int> _r = getSelectedRows((*reinterpret_cast< QTableWidget*(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<int>*>(_a[0]) = _r; }  break;
        case 3: easyButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: hardButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: labelChanged((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 6: verySafeButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: safeButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: normalButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: fastButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: veryFastButtonToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: toggleTable((*reinterpret_cast< QTableWidget*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 12: toggleTable((*reinterpret_cast< QTableWidget*(*)>(_a[1]))); break;
        case 13: defaultTogglePushed(); break;
        case 14: oftenTogglePushed(); break;
        case 15: occasionallyTogglePushed(); break;
        case 16: dofTogglePushed(); break;
        case 17: selectJointButtonClicked(); break;
        case 18: deselectJointButtonClicked(); break;
        case 19: deleteGroupButtonClicked(); break;
        case 20: acceptChainClicked(); break;
        case 21: acceptGroupClicked(); break;
        case 22: baseLinkTreeClick(); break;
        case 23: tipLinkTreeClick(); break;
        case 24: validateDoneBox(); break;
        case 25: groupTableClicked(); break;
        case 26: defaultTableClicked(); break;
        case 27: oftenTableClicked(); break;
        case 28: occasionallyTableClicked(); break;
        case 29: dofSelectionTableChanged(); break;
        case 30: oftenCollisionTableChanged(); break;
        case 31: defaultCollisionTableChanged(); break;
        case 32: occasionallyCollisionTableChanged(); break;
        case 33: generateOccasionallyInCollisionTable(); break;
        case 34: generateOftenInCollisionTable(); break;
        case 35: generateAlwaysInCollisionTable(); break;
        case 36: generateDefaultInCollisionTable(); break;
        case 37: fileSelected((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 38: writeFiles(); break;
        case 39: autoConfigure(); break;
        case 40: update(); break;
        default: ;
        }
        _id -= 41;
    }
    return _id;
}

// SIGNAL 0
void PlanningDescriptionConfigurationWizard::changeProgress(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PlanningDescriptionConfigurationWizard::changeLabel(const char * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
