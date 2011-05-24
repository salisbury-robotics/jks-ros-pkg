#ifndef MYMATERIALEDITOR_H
#define MYMATERIALEDITOR_H

#include <QWidget>
#include <QPushButton>
#include <QDoubleSpinBox>
#include "Graphics/Material.h"

class MyMaterialEditor : public QWidget
{
Q_OBJECT
Q_PROPERTY(Material material    READ    material
                                WRITE   setMaterial
                                NOTIFY  materialChanged
                                USER    true            )

    Material        m_material;
    QPushButton    *m_colorButtons[4];
    QDoubleSpinBox *m_shininessSpinner;

    void updateButton(int id, const colour &c);

public:
    explicit MyMaterialEditor(QWidget *parent = 0);

    void setMaterial(const Material &m);
    Material material() const           { return m_material; }

signals:
    void materialChanged(Material m);

public slots:
    void selectColor(int id);
    void setShininess(double s);

};

#endif // MYMATERIALEDITOR_H
