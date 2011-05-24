#ifndef MYLABELLINGEDITOR_H
#define MYLABELLINGEDITOR_H

#include <QWidget>
#include <QCheckBox>
#include <QSpinBox>
#include <QTableWidget>
#include "Common/Labelling.h"

class MyLabellingEditor : public QWidget
{
Q_OBJECT
Q_PROPERTY(Labelling *labelling READ    labelling
                                WRITE   setLabelling
                                NOTIFY  labellingChanged
                                USER    true            )

    Labelling       *m_labelling;

    QCheckBox       *m_thresholdBox;
    QSpinBox        *m_thresholdValueBox;
    QTableWidget    *m_colorTable;

public:
    explicit MyLabellingEditor(QWidget *parent = 0);

    void setLabelling(Labelling * const lab);
    Labelling *labelling() const { return m_labelling; }

signals:
    void labellingChanged(Labelling *lab);

public slots:
    void setThreshold(bool t);
    void setThresholdValue(int v);
    void selectColor(int r, int c);
};

#endif // MYLABELLINGEDITOR_H
