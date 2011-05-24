// -------------------------------------------------------------------------- //
// Sonny Chan, Department of Computer Science, Stanford University			  //
// Joseph Lee, Department of Otolaryngology, Stanford University			  //
// -------------------------------------------------------------------------- //

#ifndef MyRendererTFEditor_TFE_H
#define MyRendererTFEditor_TFE_H

// -------------------------------------------------------------------------- //

#include <math.h>
#include <vector>
#include <algorithm>

#include <QWidget>

#include "Graphics/TransferFunction.h"

// -------------------------------------------------------------------------- //

struct TFEMarker
{	
    TFEMarker()
    {
        x = 0.0;
        color_left = QColor( 0, 0, 0, 0 );
        color_right = QColor( 0, 0, 0, 0 );
        b_iso = false;
        iso_index = 0;
    }

    TFEMarker( float x_value, QColor c_l, QColor c_r, bool iso, int ind )
	{ 
		x = x_value; 
		color_left = c_l; 
		color_right = c_r; 
		b_iso = iso; 
		iso_index = ind;
	};
	
	float x; bool b_iso; int iso_index;
	QColor color_left; 
	QColor color_right;	
};

// data stream insertion and extraction operators for saving state of TFEMarker
QDataStream & operator<< ( QDataStream &stream, const TFEMarker &marker );
QDataStream & operator>> ( QDataStream &stream, TFEMarker &marker );

// -------------------------------------------------------------------------- //

class TFEMarkers
{
public:

	std::vector <TFEMarker> markers;

	TFEMarker * at( int ind ) { return & markers.at( ind ); }

	void addMarker( float x, QColor c_l, QColor c_r, bool iso = false, int ind = 0 ) 
	{ 
		bool b_done = false;
		for( std::vector <TFEMarker>::iterator it = markers.begin(); !b_done; )
		{
			if( x <= 0.0 || x >= 1.0 ) b_done = true;
			else if( it->x > x )
			{
				markers.insert( it, TFEMarker( x, c_l, c_r, iso, ind ) );
				b_done = true;
			}
			else it++;
		}
	}
	
	void delMarker( int index )
	{ markers.erase( markers.begin() + index ); }

	void initMarker( float x, QColor c_l, QColor c_r, bool iso = false, int ind = 0 ) 
	{ markers.push_back( TFEMarker( x, c_l, c_r, iso, ind ) ); }
};

// data stream insertion and extraction operators for saving state of TFEMarkers
QDataStream & operator<< ( QDataStream &stream, const TFEMarkers &markers );
QDataStream & operator>> ( QDataStream &stream, TFEMarkers &markers );

// -------------------------------------------------------------------------- //

class MyRendererTFEditor_TFE : public QWidget
{
Q_OBJECT   

    // TODO: NUM_VOLUMES should probably be defined elsewhere
    static const int NUM_VOLUMES = 1;

public:

	MyRendererTFEditor_TFE( QWidget * parent = 0 );
//	MessageLog * m_log;

	float m_isosurface_threshold[NUM_VOLUMES];
	QColor m_isosurface_color[NUM_VOLUMES];

	float m_display_min;
	float m_display_max;

	Qt::MouseButton m_mouse_button;

    unsigned char * m_transfer_function;

	TFEMarkers	m_tfe_markers;
	int			m_tfe_marker_selected;
	int			m_tfe_marker_side;

	void initialize( void );

    // functions to save and restore the marker set from a data stream
    void writeMarkers(QDataStream &stream)  { stream << m_tfe_markers; }
    void readMarkers(QDataStream &stream);

private:

	QColor		m_color_selected[2];

	void updateIsosurface( void );
	void updateTF( void );
	float rescale( float x_old_min, float x_old_max, float x_old_pos, float x_new_min, float x_new_max );

protected:

	void mouseMoveEvent( QMouseEvent * event );
	void mousePressEvent( QMouseEvent * event );
	void mouseReleaseEvent( QMouseEvent * event );
	void paintEvent( QPaintEvent * event );
	void wheelEvent( QWheelEvent * event );

private slots:

	void receiveColors( QColor color_L, QColor color_R );
	void receiveDisplayRange( float min, float max );

signals:

    void transferFunctionChanged( TransferFunction tf );
    void isosurfaceChanged( float threshold, const QColor &color );
};

#endif
