#ifndef MYRENDERERPRESETSWIDGET_H
#define MYRENDERERPRESETSWIDGET_H

#include <QGroupBox>
#include <QComboBox>
#include <QPushButton>
#include <QMap>

class MyRendererTFEditor;

class MyRendererPresetsWidget : public QGroupBox
{
Q_OBJECT
    // reserved transfer function names
    static const QString k_bone;
    static const QString k_boneDVR;
    static const QString k_composite;
    static const QString k_new;
    static const int     k_builtInPresets = 3;

protected:
    // a handle to the transfer function editor class we're associated with
    MyRendererTFEditor  *m_editor;

    // the three control widgets for transfer function management
    QComboBox           *m_selections;
    QPushButton         *m_removeButton;
    QPushButton         *m_saveButton;

    // actual transfer function data is stored in a map of QByteArrays
    QMap<QString, QByteArray> m_data;

    // helper to check if a given transfer function is removable
    bool isRemovable(const QString &name);

    // updates the state of the save and remove buttons
    void updateButtonStates(bool clean = false);

    // these methods read and write the data collection to QSettings
    void loadData(const QString &name, const QString &file);
    void readSettings();
    void writeSettings();

public:
    explicit MyRendererPresetsWidget(MyRendererTFEditor *editor, QWidget *parent = 0);

    // populates a given menu with actions for import/export of presets
    void populateMenu(QMenu *menu);

signals:

public slots:
    void setSelection(const QString &name);
    void setCurrentDirty() { updateButtonStates(false); }

    void removeCurrent();
    void saveCurrent();

    void importFromFile(QString path = QString());
    void exportToFile(QString path = QString());
};

#endif // MYRENDERERPRESETSWIDGET_H
