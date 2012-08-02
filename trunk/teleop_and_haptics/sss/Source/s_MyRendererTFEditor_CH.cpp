// -------------------------------------------------------------------------- //
// Sonny Chan, Department of Computer Science, Stanford University			  //
// Joseph Lee, Department of Otolaryngology, Stanford University			  //
// -------------------------------------------------------------------------- //

#include "s_MyRendererTFEditor_CH.h"
#include <QtGui>

// -------------------------------------------------------------------------- //

MyRendererTFEditor_CH::MyRendererTFEditor_CH( QWidget * parent ) : QWidget( parent )
{
	// initial size of the color hexagon, this will be scaled down
	setFixedSize( QSize( 300, 300 ) );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_CH::initialize( void )
{
	// this function should be run immediately after the widget is created to
	//	prevent repeated re-drawing of the color-hexagon
	m_pixmap	= QPixmap::grabWidget( this );
	
	// final size of the color hexagon
	QSize size = QSize( 90, 90 );

	setFixedSize( size );
	m_pixmap	= m_pixmap.scaled( size, Qt::KeepAspectRatio, Qt::SmoothTransformation );
	m_image		= m_pixmap.toImage();
	
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_CH::mouseMoveEvent( QMouseEvent *event )
{
	addMarker( event->pos() );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_CH::mousePressEvent( QMouseEvent * event )
{
	if( event->button() == Qt::LeftButton )	m_mouse_button = Qt::LeftButton;
	if( event->button() == Qt::RightButton ) m_mouse_button = Qt::RightButton;

	addMarker( event->pos() );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_CH::mouseReleaseEvent( QMouseEvent *event )
{
	QPoint pos = event->pos();
	QSize size = this->size();
	
	// color at mouse pointer
	QColor color = QColor::fromRgb( m_image.pixel( pos ) );

	// color outside of color hexagon
	QColor color_default_gray = QColor::fromRgb( m_image.pixel( QPoint( 0, 0 ) ) );

	// if mouse pointer is outside of the color hexagon
	if( !( color != color_default_gray && 
		( pos.x() > 0 && pos.x() < size.width() && pos.y() > 0 && pos.y() < size.height() ) ) )
	{
		// if the left mouse button is released
		if( m_mouse_button == Qt::LeftButton )
			// if only the left marker is placed
			if( m_ch_marker[1].pos == QPoint( 0, 0 ) )
				// delete the left marker
				m_ch_marker[0] = CHMarker( QPoint( 0, 0 ), QColor( 0, 0, 0, 0 ) );
			// if both markers are placed
			else
			{
				// delete the left marker and make the old right marker the new
				//	left marker
				m_ch_marker[0] = m_ch_marker[1];
				m_ch_marker[1] = CHMarker( QPoint( 0, 0 ), QColor( 0, 0, 0, 0 ) );
			}
		// if the right mouse button is released
		else
			// delete the right marker
			m_ch_marker[1] = CHMarker( QPoint( 0, 0 ), QColor( 0, 0, 0, 0 ) );

		update();
	}
	
	sendColors( m_ch_marker[0].color, m_ch_marker[1].color );
	m_mouse_button = Qt::NoButton;
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_CH::paintEvent( QPaintEvent * )
{
	QPainter painter( this );
	painter.setRenderHint( QPainter::Antialiasing, true );
	QPoint origin	= QPoint( 0, 0 );
	QSize size		= this->size();

	// widget is rendered pixel by pixel, only the first time it is rendered
	if( m_pixmap.isNull() )
	{
		size = QSize( size.width() - 20, size.height() - 20 );

		float wf = size.width();	float wh = wf / 2.0; float wq = wf / 4.0;
		float hf = size.height();	float hh = hf / 2.0;
		
		painter.translate( 10, 10 );

		for( int i = 0; i < hh; i++ )
		{
			for( int j = wq - wq * (i / hh); j < wq + wq * (i / hh) ; j++ )
			{
				painter.setPen( interpolateColor(	QPointF(  j,  i ),
						QColor(   0, 255, 255 ),	QPointF(  0, hh ),
						QColor(   0,   0, 255 ),	QPointF( wq,  0 ),
													QPointF( wh, hh ) ) );
				painter.drawPoint( QPointF( j,		i ));

				painter.setPen( interpolateColor(	QPointF(  j,  i ),
						QColor(   0,   0, 255 ),	QPointF(  0, hh ),
						QColor( 255,   0, 255 ),	QPointF( wh, hh ),
													QPointF( wq,  0 ) ) );
				painter.drawPoint( QPointF( j + wq, hh - i - 1 ));

				painter.setPen( interpolateColor(	QPointF(  j,  i ),
						QColor( 255,   0, 255 ),	QPointF( wq,  0 ),
						QColor( 255,   0,   0 ),	QPointF( wh, hh ),
													QPointF(  0, hh ) ) );
				painter.drawPoint( QPointF( j + wh, i ));
				

				painter.setPen( interpolateColor(	QPointF(  j,  i ),
						QColor( 255,   0,   0 ),	QPointF( wh, hh ),
						QColor( 255, 255,   0 ),	QPointF( wq,  0 ),
													QPointF(  0, hh ) ) );
				painter.drawPoint( QPointF( j + wh, hf - i - 1 ));
				

				painter.setPen( interpolateColor(	QPointF(  j,  i ),
						QColor( 255, 255,   0 ),	QPointF( wh, hh ),
						QColor(   0, 255,   0 ),	QPointF(  0, hh ),
													QPointF( wq,  0 ) ) );
				painter.drawPoint( QPointF( j + wq, i + hh ));
				

				painter.setPen( interpolateColor(	QPointF(  j,  i ),
						QColor(   0, 255,   0 ),	QPointF( wq,  0 ),
						QColor(   0, 255, 255 ),	QPointF(  0, hh ),
													QPointF( wh, hh ) ) );
				painter.drawPoint( QPointF( j,		hf - i -1 ));				
			}
		}
	}

	// when initialize is called, the rendered color hexagon is stored into a
	//	pixmap. on subsequent renders, the pixmap is drawn instead. this
	//	improves performance dramatically on slower computers
	else
	{
		// draw pre-rendered color hexagon
		painter.drawPixmap( origin, m_pixmap );

		// if the first marker is active...
		if( m_ch_marker[0].pos != QPoint( 0, 0 ) )
		{
			// initialize the pen
			QPen pen = QPen( Qt::black );
			pen.setWidthF( 2.0 );

			// grab position and color of the first marker
			QPoint pos = m_ch_marker[0].pos;
			QColor color_marker = m_ch_marker[0].color;

			// only draw the first marker
			painter.setPen( pen );
			painter.drawArc( QRectF( pos.x() - 11, pos.y() - 11, 23 , 23 ), 0, 360*16 );
			painter.setPen( color_marker );
			painter.setBrush( QBrush( color_marker ) );
			painter.drawEllipse( QRectF( pos.x() - 10, pos.y() - 10, 21 , 21 ) );

			// if the second marker is active...
			if( m_ch_marker[1].pos != QPoint( 0, 0 ) )
			{
				// letter the first marker
				QFont font;
				font.setBold( true );
				font.setPointSizeF( 12 );
				painter.setFont( font );
				painter.setPen( Qt::white );
				painter.drawText( QRectF( pos.x() - 9, pos.y() - 10, 21 , 21 ), Qt::AlignCenter, tr("L") );
				
				// grab the position and color of the second marker
				pos = m_ch_marker[1].pos;
				color_marker = m_ch_marker[1].color;

				// draw the second marker
				painter.setPen( pen );
				painter.drawArc( QRectF( pos.x() - 11, pos.y() - 11, 23 , 23 ), 0, 360*16 );
				painter.setPen( color_marker );
				painter.setBrush( QBrush( color_marker ) );
				painter.drawEllipse( QRectF( pos.x() - 10, pos.y() - 10, 21 , 21 ) );

				// leter the second marker
				painter.setPen( Qt::white );
				painter.drawText( QRectF( pos.x() - 9, pos.y() - 10, 21 , 21 ), Qt::AlignCenter, tr("R") );
			}
		}
	}
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_CH::addMarker( QPoint pos )
{
	QSize size = this->size();
	QColor color = QColor::fromRgb( m_image.pixel( pos ) );
	QColor color_default_gray = QColor::fromRgb( m_image.pixel( QPoint( 0, 0 ) ) );

	// if the cursor is within the color hexagon
	if( color != color_default_gray && 
		( pos.x() > 0 && pos.x() < size.width() && pos.y() > 0 && pos.y() < size.height() ) )
	{
		// assign first color marker
		if( m_mouse_button == Qt::LeftButton ||
			( m_mouse_button == Qt::RightButton && m_ch_marker[0].pos == QPoint( 0, 0 ) ) )	
			m_ch_marker[0] = CHMarker( pos, color );
		
		// assign second color marker
		else
			m_ch_marker[1] = CHMarker( pos, color );
		
		update();
	}
}

// -------------------------------------------------------------------------- //

QColor MyRendererTFEditor_CH::interpolateColor( QPointF point, QColor c1, QPointF p1, QColor c2, QPointF p2, QPointF center )
{
	QPointF p1p;			QPointF p2p;			
	QPointF p3p;			QPointF p4p;

	p1p = projectPointOnLine( point,	p1, center );
	p2p = projectPointOnLine( point,	p2, center );
	p3p = projectPointOnLine( point,	p1, p2 );
	p4p = projectPointOnLine( center,	p1, p2 );

	float l_1 = QLineF( p1p, point ).length();
	float l_2 = QLineF( p2p, point ).length();

	float f_1 = 1.0 - l_1 / (l_1 + l_2);
	float f_2 = 1.0 - l_2 / (l_1 + l_2);
	float f_c = QLineF( p3p, point ).length() / QLineF( p4p, center ).length();

	QColor color = QColor(
		int(f_1 * c1.red()		+ f_2 * c2.red()),
		int(f_1 * c1.green()	+ f_2 * c2.green()),
		int(f_1 * c1.blue()		+ f_2 * c2.blue()),
		255 );

	if( f_c > 1/3.0 )
	{
		color.setRed(	color.red()		+ int((255 - color.red())	* ( f_c - 1/3.0 ) / ( 2/3.0 )) );
		color.setGreen( color.green()	+ int((255 - color.green()) * ( f_c - 1/3.0 ) / ( 2/3.0 )) );
		color.setBlue(	color.blue()	+ int((255 - color.blue())	* ( f_c - 1/3.0 ) / ( 2/3.0 )) );
	}
	else
	{
		color.setRed(	color.red()		- int(color.red()			* ( 1/3.0 - f_c ) / ( 1/3.0 )) );
		color.setGreen( color.green()	- int(color.green()			* ( 1/3.0 - f_c ) / ( 1/3.0 )) );
		color.setBlue(	color.blue()	- int(color.blue()			* ( 1/3.0 - f_c ) / ( 1/3.0 )) );
	}

	return color;
}

// -------------------------------------------------------------------------- //

int MyRendererTFEditor_CH::rescale( int x_old_min, int x_old_max, int x_old_pos, int x_new_min, int x_new_max )
{
	return( x_new_max - x_new_min ) * x_old_pos / ( x_old_max - x_old_min );
}

// -------------------------------------------------------------------------- //

QPointF MyRendererTFEditor_CH::projectPointOnLine( 
	QPointF point, QPointF line_pt_1, QPointF line_pt_2 )
{
	// this function finds the point which represents the intersection of the 
	//	line constructed by line_pt_1 and line_pt_2 and the line perpendicular
	//	containing the point
	
	float line1_point1_x	= line_pt_1.x();	float line1_point1_y	= line_pt_1.y();
	float line1_point2_x	= line_pt_2.x();	float line1_point2_y	= line_pt_2.y();
	float line2_point_x		= point.x();		float line2_point_y		= point.y();
	float s_1;				float i_1;
	float s_2;				float i_2;
	QPointF result;

	if( line1_point1_y == line1_point2_y ) 
		result = QPointF( line2_point_x, line1_point1_y );
	else
	{
		s_1 = ( line1_point1_y - line1_point2_y ) / ( line1_point1_x - line1_point2_x );
		i_1 = line1_point1_y - s_1 * line1_point1_x;

		s_2 = - 1.0 / s_1;				
		i_2 = line2_point_y - s_2 * line2_point_x;

		result = 
			QPointF( (i_2 - i_1) / (s_1 - s_2), s_2 * (i_2 - i_1) / (s_1 - s_2) + i_2 );
	}

	return result;
}