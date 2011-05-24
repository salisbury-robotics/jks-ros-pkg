// -------------------------------------------------------------------------- //
// Sonny Chan, Department of Computer Science, Stanford University			  //
// Joseph Lee, Department of Otolaryngology, Stanford University			  //
// -------------------------------------------------------------------------- //

#include "s_MyRendererTFEditor_TFE.h"

#include <QtGui>
#include <cml/cml.h>

// -------------------------------------------------------------------------- //

MyRendererTFEditor_TFE::MyRendererTFEditor_TFE( QWidget * parent ) : QWidget( parent )
{
	// init message log
//	m_log = MessageLog::instance();
	
	// init variables
	m_display_min	= 0.0;
	m_display_max	= 1.0;

	m_mouse_button = Qt::NoButton;

	// init widget size
	setMinimumWidth( 600 );

	// initialize transfer function
//	m_transfer_function = new unsigned char [TF_SIZE * 4];
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::initialize( void )
{
    // first clear the markers so this function can be called again to reset
    m_tfe_markers.markers.clear();

	// initialize start and end tf markers
	m_tfe_markers.initMarker(
		0.00, QColor( 0, 0, 0, 0 ), QColor( 0, 0, 0, 0 ) );
	m_tfe_markers.initMarker(	
		1.00, QColor( 0, 0, 0, 0 ), QColor( 0, 0, 0, 0 ) );

	// add default markers
	m_tfe_markers.addMarker(		
		0.30, QColor( 255,  51,  51,  80 ), QColor( 255,  51,  51,  80 ) );
	m_tfe_markers.addMarker(		
		0.50, QColor( 255, 255, 128, 128 ), QColor( 255, 255, 128, 128 ) );
	m_tfe_markers.addMarker(		
		0.95, QColor( 255, 255, 255, 255 ), QColor( 255, 255, 255, 255 ) );

	// add iso markers
	m_tfe_markers.addMarker(		
		0.15, QColor( 228, 209, 192,  75 ), QColor( 228, 209, 192,  75 ), true, 0 );

    if (NUM_VOLUMES > 1)
	m_tfe_markers.addMarker(		
		0.20, QColor( 228, 209, 192,  75 ), QColor( 228, 209, 192,  75 ), true, 1 );

	// initialize utility variables
	m_tfe_marker_selected = 0;
	m_tfe_marker_side = 0;

	updateIsosurface();
	updateTF();
	update();
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::mouseMoveEvent( QMouseEvent * event )
{
	// if there is a valid node to select
	if( m_mouse_button == Qt::LeftButton && m_tfe_markers.markers.size() > 2 )
	{
		QPoint pos = event->pos();
		QSize size = this->size();

		float x_rescaled = 
			rescale( 0.0, 1.0, pos.x() / float(size.width()), m_display_min, m_display_max );
		float y_rescaled = 
			cml::clamp( 1.0 - pos.y() / float(size.height()), 0.0, 1.0 );

		// new variable names for code clarity
		TFEMarkers * m = & m_tfe_markers;
		int sel = m_tfe_marker_selected;

		// update marker position
		if( m->at( sel )->b_iso || m->at( sel )->color_left == m->at( sel )->color_right )
		{
			m->at( sel )->color_left.setAlphaF( y_rescaled );
			m->at( sel )->color_right.setAlphaF( y_rescaled );
		}
		else 
		{
			if( m_tfe_marker_side < 0 ) m->at( sel )->color_left.setAlphaF( y_rescaled );
			if( m_tfe_marker_side > 0 ) m->at( sel )->color_right.setAlphaF( y_rescaled );
		}
		 m->at( sel )->x = cml::clamp( x_rescaled, 0.0f, 1.0f );

		// re-add marker if ranking has changed
		if(  m->at( sel )->x >  m->at( sel + 1 )->x )
		{
			int count = 1;
			for( int i = sel + 2; i < m->markers.size(); i++ )
				if(  m->at( sel )->x >  m->at( i )->x ) count++;
			
			TFEMarker m_temp =  * m->at( sel );
			m->delMarker( sel );
			m->addMarker( m_temp.x, m_temp.color_left, m_temp.color_right, m_temp.b_iso, m_temp.iso_index );
			sel += count;
		}
		if( m->at( sel )->x < m->at( sel - 1 )->x )
		{
			int count = 1;
			for( int i = sel - 2; i > 0; i-- )
				if( m->at( sel )->x < m->at( i )->x ) count++;
			
			TFEMarker m_temp = * m->at( sel );
			m->delMarker( sel );
			m->addMarker( m_temp.x, m_temp.color_left, m_temp.color_right, m_temp.b_iso, m_temp.iso_index );
			sel -= count;
		}

		m_tfe_marker_selected = sel;
		if( m->at( sel )->b_iso ) updateIsosurface();
		else					  updateTF();

		update();
	}
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::mousePressEvent( QMouseEvent * event )
{
	QPoint pos = event->pos();
	int w = this->size().width();
	int h = this->size().height();

	// marker selection is only engaged with the left mouse button, and if
	//	there are markers to move
	if( event->button() == Qt::LeftButton && m_tfe_markers.markers.size() > 2 )
	{	
		int selected_index;
		
		float factor = 1 / ( m_display_max - m_display_min );
		float min_distance = sqrt( float( w * w * factor * factor + h * h ) );
		
		// find the index of the nearest marker
		for( int i = 1; i < m_tfe_markers.markers.size() - 1; i++ )
		{
			float x_rescaled = rescale( m_display_min, m_display_max, m_tfe_markers.markers.at(i).x, 0, w );
			
			float d_x_left	= pos.x() - x_rescaled + 1;
			float d_x_right = pos.x() - x_rescaled - 1;
		
			float d_y_left	= h - pos.y() - h * m_tfe_markers.markers.at(i).color_left.alpha() / 255.0;
			float d_y_right = h - pos.y() - h * m_tfe_markers.markers.at(i).color_right.alpha() / 255.0;

			float dist_left		= sqrt( d_x_left * d_x_left + d_y_left * d_y_left );
			float dist_right	= sqrt( d_x_right * d_x_right + d_y_right * d_y_right ); 

			float current_distance = std::min( dist_left, dist_right );

			if( current_distance < min_distance ) 
			{ 
				min_distance = current_distance;	
				selected_index = i; 

				if( dist_left < dist_right )	m_tfe_marker_side = -1;
				else							m_tfe_marker_side = +1;
			}
		}

		m_tfe_marker_selected = selected_index;
		m_mouse_button = Qt::LeftButton;
	}

	if( event->button() == Qt::RightButton ) m_mouse_button = Qt::RightButton;
	update();
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::mouseReleaseEvent( QMouseEvent * event )
{
	// store position and size of widget
	QPoint pos = event->pos();
	int w = this->size().width();
	int h = this->size().height();

	// rescale position coordinates
	float x_rescaled = 
		rescale( 0.0, 1.0, pos.x() / float(w), m_display_min, m_display_max );
	float y_rescaled = 
		1.0 - pos.y() / float(h);

	// new variable names for code clarity
	TFEMarkers * m = & m_tfe_markers;
	int sel = m_tfe_marker_selected;

	// define boundary for pulling markers off the tranfer function graph
	float outside = 0.25;

	// store number of valid colors stored
	int valid_colors;
	if( m_color_selected[1].rgb() != QColor( 0, 0, 0, 0 ).rgb() )		valid_colors = 2;
	else if( m_color_selected[0].rgb() != QColor( 0, 0, 0, 0 ).rgb() )	valid_colors = 1;
	else																valid_colors = 0;

	// if a marker has been dragged off the graph, delete it
	if( ( m_mouse_button == Qt::LeftButton && !m->at( sel )->b_iso ) &&
		( x_rescaled < 0.0 - outside || 
		  x_rescaled > 1.0 + outside || 
		  y_rescaled < 0.0 - outside || 
		  y_rescaled > 1.0 + outside ) )
		m->delMarker( sel );

	// if the user right-clicked and there are valid colors
	else if( m_mouse_button == Qt::RightButton && valid_colors > 0 && 
		( x_rescaled > 0.0 || 
		  x_rescaled < 1.0 || 
		  y_rescaled > 0.0 || 
		  y_rescaled < 1.0 ) )
	{	
		y_rescaled = cml::clamp( y_rescaled, 0.f, 1.f );
		
		m_color_selected[0].setAlphaF( y_rescaled - 0.05 );

		if( valid_colors > 1 ) m_color_selected[1].setAlphaF( y_rescaled + 0.05 );		
		
		// look-up index of adjacent marker
		int adjacent; float low_dist = w;
		for( int i = 1; i < m->markers.size() - 1; i++ )
		{
			// calculate distance to marker
			float x_pos_rescaled = rescale( m_display_min, m_display_max, m->at( i )->x, 0.0, 1.0 );
			float d_x	= ( pos.x() - x_pos_rescaled * w ) ;
			float d_y	= h - pos.y() - h * m->at( i )->color_left.alphaF();
			float dist	= sqrt( d_x * d_x + d_y * d_y );
		
			// if a closer marker is found, pick it
			if( dist < low_dist ) 
			{
				adjacent = i;
				low_dist = dist;
			}
		}

		// if user clicked close to a marker, change it's color
		if( low_dist < 8 )
		{
			if( m->at( adjacent )->b_iso )
			{
				m->at( adjacent )->color_left = m_color_selected[0];
				m->at( adjacent )->color_right = m_color_selected[0];
			}
			else
			{
				if( valid_colors > 1 )
				{
					m->at( adjacent )->color_left = m_color_selected[0];
					m->at( adjacent )->color_right = m_color_selected[1];
				}
				else
				{
					m->at( adjacent )->color_left = m_color_selected[0];
					m->at( adjacent )->color_right = m_color_selected[0];
				}
			}
		}

		// otherwise, if two colors are selected, add a dual color marker
		else if( valid_colors == 2 )
		{
			m_color_selected[0].setAlphaF( y_rescaled - 0.05 );
			m_color_selected[1].setAlphaF( y_rescaled + 0.05 );		
			m->addMarker( x_rescaled, m_color_selected[0], m_color_selected[1], false );
		}

		// otherwise, just add a single color marker
		else
			m->addMarker( x_rescaled, m_color_selected[0], m_color_selected[0], false );
	}

	m_mouse_button = Qt::NoButton;
	m_tfe_marker_selected = 0;
	m_tfe_marker_side = 0;
	update();
	updateIsosurface();
	updateTF();
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::paintEvent( QPaintEvent * )
{
	QPainter painter( this );
	painter.setRenderHint( QPainter::Antialiasing, false );

	int		s_w			= this->size().width();
	int		s_h			= this->size().height();
	std::vector < TFEMarker > mrks;

	// remove isomarker for convienence
	TFEMarkers tfe_markers = m_tfe_markers;
	for( int i = 0 ; i < tfe_markers.markers.size() ; )
		if( tfe_markers.markers.at( i ).b_iso )
			tfe_markers.delMarker( i );
		else i++;
	mrks = tfe_markers.markers;

	// paint tranfer function indirectly via tf markers
	for( int i = 0 ; i < mrks.size() - 1 ; i++ )
	{
		QPoint bottom_left(		rescale( m_display_min, m_display_max, mrks.at( i ).x,		0.0, 1.0 ) * s_w, s_h );
		QPoint top_left(		rescale( m_display_min, m_display_max, mrks.at( i ).x,		0.0, 1.0 ) * s_w, ( 1 - mrks.at( i ).color_right.alphaF() ) * s_h );
		QPoint bottom_right(	rescale( m_display_min, m_display_max, mrks.at( i+1 ).x,	0.0, 1.0 ) * s_w, s_h );
		QPoint top_right(		rescale( m_display_min, m_display_max, mrks.at( i+1 ).x,	0.0, 1.0 ) * s_w, ( 1 - mrks.at( i+1 ).color_left.alphaF() ) * s_h );
		
		QPoint points[] = {	top_right,
							bottom_right,
							bottom_left,
							top_left };

		QLinearGradient gradient( bottom_left, bottom_right );
		gradient.setColorAt( 0,	mrks.at( i ).color_right );
		gradient.setColorAt( 1,	mrks.at( i+1 ).color_left );

		painter.setPen( Qt::transparent );
		painter.setBrush( gradient );
		painter.drawPolygon( points, 4 );
	}

	// paint histogram
	//for( int i = 0 ; i < TF_SIZE ; i++ )
	//{
	//	int x_rescaled = int(rescale( 0, s_w - 1, i, m_display_min * ( TF_SIZE - 1 ), m_display_max * ( TF_SIZE - 1 ) ));
	//	
	//	// paint histogram
	//	painter.setPen( QColor( 185, 185, 185,  75 ) );
	//	painter.drawLine( QLineF(	i,
	//								s_h,
	//								i,
	//								s_h - m_histogram[x_rescaled] * s_h ) );
	//}
	
	painter.setRenderHint( QPainter::Antialiasing, true );
	mrks = m_tfe_markers.markers;

	// paint markers
	for( int i = 1 ; i < mrks.size() - 1 ; i++ )
	{	
		float rect_center_x = ( rescale( m_display_min, m_display_max, mrks.at(i).x, 0.0, 1.0 ) * s_w );
		float rect_center_y;
		QColor color_marker;
		int size_mark = 3;

		// paint vertical lines for iso marker
		if( mrks.at( i ).b_iso )
		{
			painter.setRenderHint( QPainter::Antialiasing, false );
			painter.setPen( QColor( 0, 0, 0, 50 ) );
			painter.drawLine( QLineF( rect_center_x - 1, s_h, rect_center_x - 1, 0 ) );
			painter.drawLine( QLineF( rect_center_x + 1, s_h, rect_center_x + 1, 0 ) );
			painter.setPen( mrks.at( i ).color_left );
			painter.drawLine( QLineF( rect_center_x, s_h, rect_center_x, 0 ) );
			painter.setRenderHint( QPainter::Antialiasing, true );
		}
		
		// draw markers
		rect_center_y = ( s_h - mrks.at(i).color_left.alpha() * s_h / 255.0 );
		color_marker = mrks.at(i).color_left;
		color_marker.setAlpha( 255 );

		if( mrks.at(i).color_left == mrks.at(i).color_right )
		{
			painter.setPen( Qt::black ); painter.setBrush( Qt::NoBrush );
			painter.drawEllipse( QRectF( rect_center_x - size_mark - 1, rect_center_y - size_mark - 1, 2*size_mark + 3 , 2*size_mark + 3 ) );
			painter.setPen( color_marker );	painter.setBrush( QBrush( color_marker ) );
			painter.drawEllipse( QRectF( rect_center_x - size_mark, rect_center_y - size_mark, 2*size_mark + 1 , 2*size_mark + 1 ) );
		}
		else
		{
			painter.setPen( Qt::black ); painter.setBrush( Qt::NoBrush );
			painter.drawEllipse( QRectF( rect_center_x - size_mark - 1, rect_center_y - size_mark - 1, 2*size_mark + 3 , 2*size_mark + 3 ) );
			painter.setPen( color_marker );	painter.setBrush( QBrush( color_marker ) );
			painter.drawEllipse( QRectF( rect_center_x - size_mark, rect_center_y - size_mark, 2*size_mark + 1 , 2*size_mark + 1 ) );
				
			rect_center_y = ( s_h - mrks.at(i).color_right.alpha() * s_h / 255.0 );
			color_marker = mrks.at(i).color_right;
			color_marker.setAlpha( 255 );
			painter.setPen( Qt::black ); painter.setBrush( Qt::NoBrush );
			painter.drawEllipse( QRectF( rect_center_x - size_mark - 1, rect_center_y - size_mark - 1, 2*size_mark + 3 , 2*size_mark + 3 ) );
			painter.setPen( color_marker );	painter.setBrush( QBrush( color_marker ) );
			painter.drawEllipse( QRectF( rect_center_x - size_mark, rect_center_y - size_mark, 2*size_mark + 1 , 2*size_mark + 1 ) );
		}
	}

	// draw white circle around selected marker
	if( m_tfe_marker_selected != 0 )
	{
		float rect_center_x = rescale( m_display_min, m_display_max, mrks.at(m_tfe_marker_selected).x, 0.0, 1.0 ) * s_w;
		float rect_center_y;
		int size_mark = 5;

		painter.setPen( QColor( 0, 0, 0, 50 ) ); 
		painter.setBrush( Qt::NoBrush );

		rect_center_y = s_h - mrks.at(m_tfe_marker_selected).color_left.alpha() * s_h / 255.0;

		if( mrks.at( m_tfe_marker_selected ).color_left == mrks.at( m_tfe_marker_selected ).color_right )
			painter.drawEllipse( QRectF( rect_center_x - size_mark, rect_center_y - size_mark, 2*size_mark + 1 , 2*size_mark + 1 ) );
		else
		{
			painter.drawEllipse( QRectF( rect_center_x - size_mark, rect_center_y - size_mark, 2*size_mark + 1 , 2*size_mark + 1 ) );
			rect_center_y = s_h - mrks.at(m_tfe_marker_selected).color_right.alpha() * s_h / 255.0;
			painter.drawEllipse( QRectF( rect_center_x - size_mark, rect_center_y - size_mark, 2*size_mark + 1 , 2*size_mark + 1 ) );
		}
	}
	
	// draw translucent white divisions
	int count_divisions;

	count_divisions = 20;
	for( int i = 1; i < count_divisions; i++ )
	{
		float tenth = s_w / float( count_divisions );
		painter.setPen( QColor( 255, 255, 255, 65 ) );
		painter.drawLine( QLineF( tenth * i, 0, tenth * i, s_h ) );
		painter.drawText( QPoint( tenth* i + 4, s_h - 3 ), tr( "%1" ).arg( m_display_min + ( m_display_max - m_display_min ) * i / count_divisions, 0, 'f', 2 ) );
	}

	count_divisions = 5;
	for( int i = 1; i < count_divisions; i++ )
	{
		float fourth = s_h / float( count_divisions );
		painter.setPen( QColor( 255, 255, 255, 65 ) );
		painter.drawLine( QLineF( 0, fourth * i, s_w, fourth * i ) );
	}

	// paint black rectangular border
	painter.setRenderHint( QPainter::Antialiasing, false );
	painter.setPen( QColor( 0, 0, 0, 50 ) );
	painter.setBrush( Qt::NoBrush );
	painter.drawRect( 0, 0, s_w - 1, s_h - 1 );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::wheelEvent(QWheelEvent *event)
{
	float delta = event->delta() * ( m_display_max - m_display_min ) * 0.000005;

	for( int i = 1; i < m_tfe_markers.markers.size() - 1; i++ )
		if( m_tfe_markers.markers.at( i ).x + delta > 0.0 &&
			m_tfe_markers.markers.at( i ).x + delta < 1.0 )
				m_tfe_markers.markers.at( i ).x += delta;
		 
	update();
	updateIsosurface();
	updateTF();
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::updateIsosurface( void )
{
	bool b_done = false;
	TFEMarkers * m = & m_tfe_markers;

	for( int i = 1 ; i < m->markers.size() - 1 ; i++ )
	{
		if( m->at( i )->b_iso )
		{
			m_isosurface_threshold[m->at( i )->iso_index] = m->at( i )->x;
			m_isosurface_color[m->at( i )->iso_index] = m->at( i )->color_left;
		}
	}

    // NOTE: This is Sonny's modified version that emits a signal when the
    //       isosurface changes
    emit isosurfaceChanged( m_isosurface_threshold[0], m_isosurface_color[0] );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::updateTF( void )
{
	TFEMarkers tf_only_markers = m_tfe_markers;
	
	// for convinience, delete the isosurface marker
	for( int i = tf_only_markers.markers.size() - 1 ; i > 0 ; )
		if( tf_only_markers.markers.at( i ).b_iso )
			tf_only_markers.delMarker( i );
		else i--;
	
	// premultiply alpha
	for( int i = tf_only_markers.markers.size() - 1; i > 0; i-- )
	{
		QColor c_l = tf_only_markers.markers.at( i ).color_left;
		tf_only_markers.markers.at( i ).color_left.setRedF( c_l.redF() * c_l.alphaF() );
		tf_only_markers.markers.at( i ).color_left.setBlueF( c_l.blueF() * c_l.alphaF() );
		tf_only_markers.markers.at( i ).color_left.setGreenF( c_l.greenF() * c_l.alphaF() );

		QColor c_r = tf_only_markers.markers.at( i ).color_right;
		tf_only_markers.markers.at( i ).color_right.setRedF( c_r.redF() * c_r.alphaF() );
		tf_only_markers.markers.at( i ).color_right.setBlueF( c_r.blueF() * c_r.alphaF() );
		tf_only_markers.markers.at( i ).color_right.setGreenF( c_r.greenF() * c_r.alphaF() );
	}
/*
	// populate the entire transfer function with correctly interpolated colors
	unsigned char * tf = m_transfer_function;

	for( int i = 0 ; i < TF_SIZE ; i++ )
	{
		for( int j = 0; j < tf_only_markers.markers.size() - 1; j++ )
		{
			if( ( tf_only_markers.markers[j].x <= ( i + 0.5 ) / float(TF_SIZE) ) && 
				( i + 0.5 ) / float(TF_SIZE) <= tf_only_markers.markers[j + 1].x )
			{
				TFEMarker m[2] = { tf_only_markers.markers.at( j ), tf_only_markers.markers.at( j + 1 ) };
			
				float fraction = ( ( i + 0.5 ) / float(TF_SIZE) - m[0].x ) / ( m[1].x - m[0].x );

				*tf++ = ( unsigned char )( 255.f * ( m[0].color_right.redF() + fraction * ( m[1].color_left.redF() - m[0].color_right.redF() ) ) );
                *tf++ = ( unsigned char )( 255.f * ( m[0].color_right.greenF() + fraction * ( m[1].color_left.greenF() - m[0].color_right.greenF() ) ) );
                *tf++ = ( unsigned char )( 255.f * ( m[0].color_right.blueF() + fraction * ( m[1].color_left.blueF() - m[0].color_right.blueF() ) ) );
                *tf++ = ( unsigned char )( 255.f * ( m[0].color_right.alphaF() + fraction * ( m[1].color_left.alphaF() - m[0].color_right.alphaF() ) ) );
			}
		}
	}
*/

    // NOTE: This is Sonny's modified version, where the signal transmits a
    //       TransferFunction object with it
    TransferFunction tf;
    for ( int i = 0; i < tf_only_markers.markers.size(); ++i )
    {
        TFEMarker &m = tf_only_markers.markers[i];

        // note: alpha must be pre-multiplied for correct mipmap averaging
        qreal r, g, b, a;

        m.color_left.getRgbF(&r, &g, &b, &a);
        cml::vector4f cl(r, g, b, a);

        m.color_right.getRgbF(&r, &g, &b, &a);
        cml::vector4f cr(r, g, b, a);

        tf.insert(m.x, cl, cr);
    }

    emit transferFunctionChanged( tf );
}

// -------------------------------------------------------------------------- //

float MyRendererTFEditor_TFE::rescale( float x_old_min, float x_old_max, float x_old_pos, float x_new_min, float x_new_max )
{
	return x_new_min + ( x_old_pos - x_old_min ) / ( x_old_max - x_old_min ) * ( x_new_max - x_new_min );
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::receiveColors( QColor color_L, QColor color_R )
{
	m_color_selected[0] = color_L;
	m_color_selected[1] = color_R;
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::receiveDisplayRange( float min, float max )
{
	m_display_min = min;
	m_display_max = max;

	update();
}

// -------------------------------------------------------------------------- //

void MyRendererTFEditor_TFE::readMarkers(QDataStream &stream)
{
    // re-initialize utility variables
    m_tfe_marker_selected = 0;
    m_tfe_marker_side = 0;

    // read the marker set from the data stream
    stream >> m_tfe_markers;

    // update everything and emit changed signals
    updateIsosurface();
    updateTF();
    update();
}

// -------------------------------------------------------------------------- //
// Serialization functions for the transfer function structures
// -------------------------------------------------------------------------- //

QDataStream & operator<< ( QDataStream &stream, const TFEMarker &marker )
{
    stream << marker.x << marker.b_iso;
    stream << marker.color_left << marker.color_right;
    return stream;
}

QDataStream & operator>> ( QDataStream &stream, TFEMarker &marker )
{
    stream >> marker.x >> marker.b_iso;
    stream >> marker.color_left >> marker.color_right;
    return stream;
}

// -------------------------------------------------------------------------- //

QDataStream & operator<< ( QDataStream &stream, const TFEMarkers &markers )
{
    int size = markers.markers.size();
    stream << size;
    for (int i = 0; i < size; ++i)
        stream << markers.markers[i];
    return stream;
}

QDataStream & operator>> ( QDataStream &stream, TFEMarkers &markers )
{
    int size;
    stream >> size;
    markers.markers.resize(size);
    for (int i = 0; i < size; ++i)
        stream >> markers.markers[i];
    return stream;
}

// -------------------------------------------------------------------------- //

