// -------------------------------------------------------------------------- //
// Sonny Chan, Department of Computer Science, Stanford University			  //
// Joseph Lee, Department of Otolaryngology, Stanford University			  //
// -------------------------------------------------------------------------- //

#ifndef MyRendererTFEditor_RS_H
#define MyRendererTFEditor_RS_H

// -------------------------------------------------------------------------- //

#include <QWidget>

// -------------------------------------------------------------------------- //

class MyRendererTFEditor_RS : public QWidget
{
Q_OBJECT   

public:

	MyRendererTFEditor_RS( QWidget * parent = 0 );

	float m_display_min;
	float m_display_max;
	float m_display_range_min;

	bool b_min_selected;
	bool b_max_selected;
	bool b_range_selected;

protected:

	void mouseDoubleClickEvent( QMouseEvent * event );
	void mouseMoveEvent( QMouseEvent * event );
	void mousePressEvent( QMouseEvent * event );
	void mouseReleaseEvent( QMouseEvent * event );
	void paintEvent( QPaintEvent * event );

private:

	Qt::MouseButton m_mouse_button;
	float rescale( float x_old_min, float x_old_max, float x_old_pos, float x_new_min, float x_new_max );

signals:
	
	void sendDisplayRange( float min, float max );
};

#endif
