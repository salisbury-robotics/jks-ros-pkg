#include "MyLabellingEditor.h"
#include <QtGui>

MyLabellingEditor::MyLabellingEditor(QWidget *parent) :
    QWidget(parent)
{
    // use a form layout to contain the editor widgets
    QFormLayout *layout = new QFormLayout(this);

    QLabel *title = new QLabel("Labelling Properties");
    title->setAlignment(Qt::AlignCenter);
    layout->addRow(title);

    m_thresholdBox = new QCheckBox;
    connect(m_thresholdBox, SIGNAL(toggled(bool)),
            this, SLOT(setThreshold(bool)));
    layout->addRow("Threshold", m_thresholdBox);

    m_thresholdValueBox = new QSpinBox;
    m_thresholdValueBox->setRange(-0x8000, 0xffff);
    connect(m_thresholdValueBox, SIGNAL(valueChanged(int)),
            this, SLOT(setThresholdValue(int)));
    layout->addRow("Threshold Value", m_thresholdValueBox);

    // create a table widget to hold a bunch of paint chips
    m_colorTable = new QTableWidget(3, 8);
    m_colorTable->horizontalHeader()->hide();
    m_colorTable->verticalHeader()->hide();
    m_colorTable->setSelectionMode(QAbstractItemView::NoSelection);
    int i = 1;
    for (int r = 0; r < m_colorTable->rowCount(); ++r)
        for (int c = 0; c < m_colorTable->columnCount(); ++c, ++i)
        {
            QTableWidgetItem *item = new QTableWidgetItem(QString("%1").arg(i));
            item->setTextAlignment(Qt::AlignCenter);
            item->setTextColor(Qt::gray);
            item->setFlags(Qt::ItemIsEnabled);
            m_colorTable->setItem(r, c, item);
        }
    m_colorTable->resizeColumnsToContents();
    layout->addRow(new QLabel("Color Look-up Table"));
    layout->addRow(m_colorTable);

    connect(m_colorTable, SIGNAL(cellClicked(int,int)),
            this, SLOT(selectColor(int,int)));

    // start with a null labelling
    setLabelling(0);
}

void MyLabellingEditor::setLabelling(Labelling * const lab)
{
    m_labelling = lab;

    if (m_labelling == 0) {
        m_thresholdBox->setEnabled(false);
        m_thresholdValueBox->setEnabled(false);
    }
    else {
        m_thresholdBox->setEnabled(true);
        m_thresholdBox->setChecked(m_labelling->threshold);
        m_thresholdValueBox->setEnabled(true);
        m_thresholdValueBox->setValue(m_labelling->thresholdValue);

        int i = 0;
        for (int r = 0; r < m_colorTable->rowCount(); ++r)
            for (int c = 0; c < m_colorTable->columnCount(); ++c, ++i)
            {
                cml::vector4f v(1, 1, 1, 0);
                if (i < m_labelling->entries()) v = m_labelling->colourLUT[i];
                m_colorTable->item(r, c)->setBackgroundColor(QColor::fromRgbF(v[0], v[1], v[2]));
            }
    }
}


void MyLabellingEditor::setThreshold(bool t)
{
    if (t != m_labelling->threshold)
    {
        m_labelling->threshold = t;
        m_labelling->dirty = true;
        emit labellingChanged(m_labelling);
    }
}

void MyLabellingEditor::setThresholdValue(int v)
{
    if (v != m_labelling->thresholdValue)
    {
        m_labelling->thresholdValue = v;
        m_labelling->dirty = true;
        emit labellingChanged(m_labelling);
    }
}

void MyLabellingEditor::selectColor(int r, int c)
{
    int i = r * m_colorTable->columnCount() + c;
    if (m_labelling && i < m_labelling->entries())
    {
        cml::vector4f &v = m_labelling->colourLUT[i];
        QColor initial = QColor::fromRgbF(v[0], v[1], v[2], v[3]);
        QColor color = QColorDialog::getColor(initial, this);
        if (!color.isValid()) color = initial;
        else m_labelling->dirty = true; // colour got changed
        v.set(color.redF(), color.greenF(), color.blueF(), color.alphaF());
        m_colorTable->item(r, c)->setBackgroundColor(color);

        emit labellingChanged(m_labelling);
    }
}
