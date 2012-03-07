#include <cstring>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif

#include "app.h"
#include "timing.h"

void Application::InitGraphics( void )
{
    // Init graphics code
    
    this->SetView();
}

void Application::SetView( void )
{
    // Set view code
}

void Application::Display( void )
{
    // Display code
}

const char* Application::GetTitle()
{
    return "Cyclone";
}

void Application::Deinit( void )
{
}

void Application::Update( void )
{
    glutPostRedisplay();
}

void Application::Key( unsigned char key )
{
}


void Application::Resize( int width, int height )
{
    if( height <= 0 ) 
    {
        height = 1;
    }
        
    Application::m_Width = width;
    Application::m_Height = height;
    glViewport( 0, 0, width, height );
    this->SetView();
}

void Application::Mouse( int button, int state, int x, int y )
{
}

void Application::MouseDrag( int x, int y )
{
}

void Application::RenderText( float x, float y, const char *text, void *font )
{
    glDisable( GL_DEPTH_TEST );
    
    glMatrixMode( GL_PROJECTION );
    glPushMatrix();
    glLoadIdentity();
    glOrtho( 0.0, (double) this->m_Width, 0.0, (double) this->m_Height, -1.0, 1.0 );
    
    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glLoadIdentity();
    
    if( font == NULL ) 
    {
        font = GLUT_BITMAP_HELVETICA_10;
    }
    
    size_t len = strlen( text );
    
    glRasterPos2f( x, y );
    for( const char *letter = text ; letter < text + len ; ++letter ) 
    {
        if( *letter == '\n' ) 
        {
            y -= 12.0f;
            glRasterPos2f( x, y );
        }
        glutBitmapCharacter( font, *letter );
    }
    
    glPopMatrix();
    glMatrixMode( GL_PROJECTION );
    glPopMatrix();
    glMatrixMode( GL_MODELVIEW );
    
    glEnable( GL_DEPTH_TEST );
}


MassAggregateApplication::MassAggregateApplication( unsigned int particleCount ) : m_World( particleCount * 10 )
{

}

MassAggregateApplication::~MassAggregateApplication()
{

}

void MassAggregateApplication::InitGraphics( void )
{
    Application::InitGraphics();
}

void MassAggregateApplication::Display( void )
{

}

void MassAggregateApplication::Update( void )
{
    this->m_World.startFrame();

#ifndef _WIN32
    float duration = (float)TimingData::get().LastFrameDuration * 0.001f;
#else
	float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
#endif

	if( duration <= 0.0f ) 
    {
        return;
    }
        
    this->m_World.runPhysics( duration );
    
    Application::Update();
}

RigidBodyApplication::RigidBodyApplication( void ) : m_Theta( 0.0f ), m_Phi( 15.0f ), m_Resolver( s_MaxContacts * 8 ), m_RenderDebugInfo( false ), m_PauseSimulation( true ), m_AutoPauseSimulation( false )
{

}

void RigidBodyApplication::Update( void )
{
#ifndef _WIN32
    float duration = (float)TimingData::get().LastFrameDuration * 0.001f;
#else
	float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
#endif

	if( duration <= 0.0f ) 
    {
        return;
    }
    else if( duration > 0.05f )
    {
        duration = 0.05f;
    }
    
    if( m_PauseSimulation )
    {
        Application::Update();
        return;
    }
    else if( m_AutoPauseSimulation )
    {
        m_PauseSimulation = true;
        m_AutoPauseSimulation = false;
    }
    
    this->UpdateObjects( duration );
    this->GenerateContacts();
    
    this->m_Resolver.resolveContacts( this->m_CollisionData.contactArray, this->m_CollisionData.contactCount, duration );
    
    Application::Update();
}

void RigidBodyApplication::Display( void )
{
    // Display code
}

void RigidBodyApplication::DrawDebug( void )
{
    if( !this->m_RenderDebugInfo ) 
    {
        return;
    }
        
    this->GenerateContacts();
    
    glBegin( GL_LINES );
    for( unsigned i = 0 ; i < this->m_CollisionData.contactCount ; ++i )
    {
        if( this->m_Contacts[i].body[1] ) 
        {
            glColor3f( 0, 1, 0 );
        } 
        else 
        {
            glColor3f( 1, 0, 0 );
        }
        
        cyclone::Vector3 vec = this->m_Contacts[i].contactPoint;
        glVertex3f( vec.x, vec.y, vec.z );
        
        vec += this->m_Contacts[i].contactNormal;
        glVertex3f( vec.x, vec.y, vec.z );
    }
    
    glEnd();
}

void RigidBodyApplication::Mouse( int button, int state, int x, int y )
{
    this->m_LastX = x;
    this->m_LastY = y;
}

void RigidBodyApplication::MouseDrag( int x, int y )
{
    this->m_Theta += (x - this->m_LastX) * 0.25f;
    this->m_Phi += (y - this->m_LastY) * 0.25f;
    
    if( this->m_Phi < -20.0f ) 
    {
        this->m_Phi = -20.0f;
    }
    else if( this->m_Phi > 80.0f ) 
    {
        this->m_Phi = 80.0f;
    }
    
    this->m_LastX = x;
    this->m_LastX = y;
}

void RigidBodyApplication::Key( unsigned char key )
{
    // Key code
    
    Application::Key( key );
}