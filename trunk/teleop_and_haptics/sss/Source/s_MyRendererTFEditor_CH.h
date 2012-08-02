// -------------------------------------------------------------------------- //
// Sonny Chan, Department of Computer Science, Stanford University			  //
// Joseph Lee, Department of Otolaryngology, Stanford University			  //
// -------------------------------------------------------------------------- //

#ifndef MyRendererTFEditor_CH_H
#define MyRendererTFEditor_CH_H

#include <QPixmap>
#include <QWidget>

// -------------------------------------------------------------------------- //

typedef struct CHMarker
{
	CHMarker() 
		{ pos = QPoint(0,0); color = QColor( 0, 0, 0, 0 ); }
    CHMarker( const QPoint a_pos, const QColor a_color ) 
		{ pos = a_pos; color = a_color; };
	
	QPoint pos;
	QColor color;
} CHMarker;

// -------------------------------------------------------------------------- //

class MyRendererTFEditor_CH : public QWidget
{
Q_OBJECT

public:

	MyRendererTFEditor_CH( QWidget * parent = 0 );
	void initialize( void );

protected:

	void mouseMoveEvent( QMouseEvent * event );
	void mousePressEvent( QMouseEvent * event );
	void mouseReleaseEvent( QMouseEvent * event );
	void paintEvent( QPaintEvent * event );

private:

	Qt::MouseButton m_mouse_button;

	CHMarker m_ch_marker[2];

	QImage	m_image;
	QPixmap m_pixmap;

	void	addMarker( QPoint pos );
	QColor	interpolateColor( QPointF point, QColor c1, QPointF p1, QColor c2, QPointF p2, QPointF center );
	int		rescale( int x_old_min, int x_old_max, int x_old_pos, int x_new_min, int x_new_max );
	QPointF	projectPointOnLine( QPointF point, QPointF line_pt_1, QPointF line_pt_2 );

private slots:

signals:

	void sendColors( QColor color_R, QColor color_L );
};

#endif