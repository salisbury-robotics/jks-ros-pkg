// -------------------------------------------------------------------------- //
// Sonny Chan, Department of Computer Science, Stanford University			  //
// Joseph Lee, Department of Otolaryngology, Stanford University			  //
// -------------------------------------------------------------------------- //

#include "s_MyRendererTFEditor.h"

#include <QtGui>

// -------------------------------------------------------------------------- //

//template < >
//MyRendererTFEditor * Singleton < MyRendererTFEditor > :: m_instance = 0;

// -------------------------------------------------------------------------- //

MyRendererTFEditor::MyRendererTFEditor( QWidget * parent ) : QWidget( parent )
{
	// transfer function and histogram ui
	QGridLayout *	layout		= new QGridLayout;

	m_ch	= new MyRendererTFEditor_CH( this );
	m_ch->initialize();

	m_tfe	= new MyRendererTFEditor_TFE( this );
	m_tfe->initialize();

	m_rs	= new MyRendererTFEditor_RS( this );

	connect( 
		m_ch,	SIGNAL( sendColors( QColor, QColor ) ), 
		m_tfe,	SLOT( receiveColors( QColor, QColor ) ) );
	connect( 
		m_rs,	SIGNAL( sendDisplayRange( float, float ) ), 
		m_tfe,	SLOT( receiveDisplayRange( float, float ) ) );

	layout->addWidget( m_ch,	0, 0, 2, 1 );
	layout->addWidget( m_tfe,	0, 1, 1, 1 );
	layout->addWidget( m_rs,	1, 1, 1, 1 );
	layout->setSpacing( 2 );

	setLayout( layout );
}
