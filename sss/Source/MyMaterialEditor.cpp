#include "MyMaterialEditor.h"
#include <QtGui>

MyMaterialEditor::MyMaterialEditor(QWidget *parent) :
    QWidget(parent)
{
    // the signal mapper will map the colour buttons to a slot to select colour
    QSignalMapper *mapper = new QSignalMapper(this);
    connect(mapper, SIGNAL(mapped(int)), this, SLOT(selectColor(int)));

    for (int i = 0; i < 4; ++i) {
        QPushButton *button = new QPushButton();
        button->setFlat(true);
        button->setMaximumWidth(64);
        connect(button, SIGNAL(clicked()), mapper, SLOT(map()));
        mapper->setMapping(button, i);
        m_colorButtons[i] = button;
    }

    m_shininessSpinner = new QDoubleSpinBox();
    m_shininessSpinner->setRange(0, 128);
    m_shininessSpinner->setMaximumWidth(64);
    connect(m_shininessSpinner, SIGNAL(valueChanged(double)),
            this, SLOT(setShininess(double)));

    // create a form layout to contain all the color editing buttons
    QFormLayout *layout = new QFormLayout(this);    
    layout->setVerticalSpacing(2);
    QLabel *title = new QLabel("Material Properties");
    title->setAlignment(Qt::AlignCenter);
    layout->addRow(title);
    layout->addRow("Ambient:", m_colorButtons[0]);
    layout->addRow("Diffuse:", m_colorButtons[1]);
    layout->addRow("Emission:", m_colorButtons[3]);
    layout->addRow("Specular:", m_colorButtons[2]);
    layout->addRow("Shininess:", m_shininessSpinner);
}

void MyMaterialEditor::setMaterial(const Material &m)
{
    m_material = m;

    updateButton(0, m.ambient);
    updateButton(1, m.diffuse);
    updateButton(2, m.specular);
    updateButton(3, m.emission);
    m_shininessSpinner->setValue(m.shininess);
}

void MyMaterialEditor::selectColor(int id)
{
    colour *c;
    switch (id) {
    case 0: c = &m_material.ambient; break;
    case 1: c = &m_material.diffuse; break;
    case 2: c = &m_material.specular; break;
    case 3: c = &m_material.emission; break;
    }

    QColor initial = QColor::fromRgbF((*c)[0], (*c)[1], (*c)[2], (*c)[3]);
    QColor current = QColorDialog::getColor(initial, this, "Select Color",
                                            QColorDialog::ShowAlphaChannel);

    if (current.isValid()) {
        double r, g, b, a;
        current.getRgbF(&r, &g, &b, &a);
        c->set(r, g, b, a);
        updateButton(id, *c);
        emit materialChanged(m_material);
    }
}

void MyMaterialEditor::setShininess(double s)
{
    if (m_material.shininess != float(s)) {
        m_material.shininess = float(s);
        emit materialChanged(m_material);
    }
}

void MyMaterialEditor::updateButton(int id, const colour &c)
{
    QPixmap pixmap(32, 24);
    pixmap.fill(QColor::fromRgbF(c[0], c[1], c[2], c[3]));
    QIcon icon(pixmap);
    m_colorButtons[id]->setIcon(icon);
}
