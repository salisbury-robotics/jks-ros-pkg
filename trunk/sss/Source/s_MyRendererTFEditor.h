// -------------------------------------------------------------------------- //
// Sonny Chan, Department of Computer Science, Stanford University			  //
// Joseph Lee, Department of Otolaryngology, Stanford University			  //
// -------------------------------------------------------------------------- //

#ifndef MyRendererTFEditor_H
#define MyRendererTFEditor_H

// -------------------------------------------------------------------------- //

#include "s_MyRendererTFEditor_CH.h"
#include "s_MyRendererTFEditor_TFE.h"
#include "s_MyRendererTFEditor_RS.h"

#include <QWidget>

// -------------------------------------------------------------------------- //

class MyRendererTFEditor : public QWidget // , public Singleton < MyRendererTFEditor >
{
Q_OBJECT

public:
	
	MyRendererTFEditor( QWidget * parent = 0 );

	MyRendererTFEditor_CH	* m_ch;
	MyRendererTFEditor_TFE	* m_tfe;
	MyRendererTFEditor_RS	* m_rs;

	float getIsoSurfaceThreshold( int i ) { return m_tfe->m_isosurface_threshold[i]; }
	QColor getIsoSurfaceColor( int i ) { return m_tfe->m_isosurface_color[i]; }
	unsigned char * getTransferFunction( void ) { return m_tfe->m_transfer_function; }

    // functions to save and load transfer functions from a data stream
    void writePreset(QDataStream &stream)   { m_tfe->writeMarkers(stream); }
    void readPreset(QDataStream &stream)    { m_tfe->readMarkers(stream); }
};

#endif
