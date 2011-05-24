// -------------------------------------------------------------------------- //
// Sonny Chan, Department of Computer Science, Stanford University			  //
// Joseph Lee, Department of Otolaryngology, Stanford University			  //
// -------------------------------------------------------------------------- //

#include "s_MyRendererTFEditor_RS.h"

#include <QtGui>

// -------------------------------------------------------------------------- //

MyRendererTFEditor_RS::MyRendererTFEditor_RS( QWidget * parent ) : QWidget( parent )
{
	// init variables
	b_min_selected = false;
	b_max_selected = false;
	b_range_selected = false;

	// initial display range parameters
	m_display_min	= 0.0;
	m_display_max	= 1.0;

	// minimal width of display range
	m_display_range_min = 0.05;

	// init widget size
	setFixedHeight( 15 );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_RS::mouseDoubleClickEvent( QMouseEvent * event )
{
	m_display_min	= 0.0;
	m_display_max	= 1.0;
	update();
	emit sendDisplayRange( m_display_min, m_display_max );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_RS::mouseMoveEvent( QMouseEvent * event )
{
	int x_pos	= event->pos().x();
	int w		= this->size().width();

	if( b_min_selected )
	{
		float display_min = x_pos / float(w);
		
		if( m_display_max - display_min < m_display_range_min )
			m_display_min = m_display_max - m_display_range_min;
		else if( display_min < 0.0 )
			m_display_min = 0.0;
		else
			m_display_min = display_min;
	}
	else if( b_max_selected )
	{
		float display_max = x_pos / float(w);
		
		if( display_max - m_display_min < m_display_range_min )
			m_display_max = m_display_min + m_display_range_min;
		else if( display_max > 1.0 )
			m_display_max = 1.0;
		else
			m_display_max = display_max;
	}
	else if( b_range_selected )
	{
		float display_range_half = ( m_display_max - m_display_min ) / 2;
		float display_center = x_pos / float(w);

		if( display_center - display_range_half < 0.0 )
			display_center = display_range_half;
		else if( display_center + display_range_half > 1.0 )
			display_center = 1.0 - display_range_half;

		m_display_min = display_center - display_range_half;
		m_display_max = display_center + display_range_half;
	}
	
	update();
	emit sendDisplayRange( m_display_min, m_display_max );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_RS::mousePressEvent( QMouseEvent * event )
{
	int x = event->pos().x();
	int w = this->size().width();
	int t = ( w - 2 ) * ( m_display_max - m_display_min ) / 3.0;
	int l = ( w - 2 ) * m_display_min + t;
	int r = ( w - 2 ) * m_display_max - t;
	
	if( event->button() == Qt::LeftButton )
	{
		if( x < l )			b_min_selected = true;
		else if( x > r )	b_max_selected = true;
		else				b_range_selected = true;
	}

	if( event->button() == Qt::RightButton )
		b_range_selected = true;

	mouseMoveEvent( event );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_RS::mouseReleaseEvent( QMouseEvent * event )
{
	b_min_selected		= false;
	b_max_selected		= false;
	b_range_selected	= false;

	update();
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_RS::paintEvent( QPaintEvent * )
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing, false);

	int	w = this->size().width();
	int	h = this->size().height();

	// rect that represents the displayed regions of the transfer function
	QRect r = QRect( QPoint( ( w - 2 ) * m_display_min, 0 ), 
					 QPoint( ( w - 2 ) * m_display_max, h - 2 ) );

	painter.setPen( Qt::white );
	painter.setBrush( Qt::white );
	painter.drawRect( r );
	painter.setBrush( Qt::NoBrush );
	
	if( b_min_selected || b_max_selected || b_range_selected )
		painter.setPen( QColor( 0, 0, 0, 75 ) );
	else
		painter.setPen( QColor( 0, 0, 0, 25 ) );
	
	painter.drawRect( r );
	painter.setPen( QColor( 0, 0, 0, 50 ) );
	painter.drawRect( QRect( QPoint( 0, 0 ), QPoint( w - 2, h - 2 ) ) );
	
}

// -------------------------------------------------------------------------- //

float MyRendererTFEditor_RS::rescale( float x_old_min, float x_old_max, float x_old_pos, float x_new_min, float x_new_max )
{
	return x_new_min + ( x_old_pos - x_old_min ) / ( x_old_max - x_old_min ) * ( x_new_max - x_new_min );
}

// -------------------------------------------------------------------------- //

