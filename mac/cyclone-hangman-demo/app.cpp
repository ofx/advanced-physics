#include <cstring>
#include <GLUT/glut.h>

#include "app.h"
#include "timing.h"

void Application::InitGraphics( void )
{
    glClearColor( 0.9f, 0.95f, 1.0f, 1.0f );
    glEnable( GL_DEPTH_TEST );
    glShadeModel( GL_SMOOTH );
    
    this->SetView();
}

void Application::SetView( void )
{
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 60.0, (double) this->m_Width / (double) this->m_Height, 1.0, 500.0 );
    glMatrixMode( GL_MODELVIEW );
}

void Application::Display( void )
{
    glClear( GL_COLOR_BUFFER_BIT );
    
    glBegin( GL_LINES );
        glVertex2i( 1, 1 );
        glVertex2i( 639, 319 );
    glEnd();
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
    this->m_ParticleArray = new cyclone::Particle[particleCount];
    for( unsigned i = 0 ; i < particleCount ; ++i )
    {
        this->m_World.getParticles().push_back( this->m_ParticleArray + i );
    }
    
    this->m_GroundContactGenerator.init( &this->m_World.getParticles() );
    this->m_World.getContactGenerators().push_back( &this->m_GroundContactGenerator );
}

MassAggregateApplication::~MassAggregateApplication()
{
    delete[] this->m_ParticleArray;
}

void MassAggregateApplication::InitGraphics( void )
{
    Application::InitGraphics();
}

void MassAggregateApplication::Display( void )
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    gluLookAt( 0.0, 3.5, 8.0,  0.0, 3.5, 0.0,  0.0, 1.0, 0.0 );
    
    glColor3f( 0, 0, 0 );
    
    cyclone::ParticleWorld::Particles &particles = this->m_World.getParticles();
    for( cyclone::ParticleWorld::Particles::iterator p = particles.begin() ; p != particles.end() ; ++p )
    {
        cyclone::Particle *particle = *p;
        cyclone::Vector3 pos = particle->getPosition();
        
        glPushMatrix();
        glTranslatef( pos.x, pos.y, pos.z );
        glutSolidSphere( 0.1f, 20, 10 );
        glPopMatrix();
    }
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
    this->m_CollisionData.contactArray = this->m_Contacts;
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
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    gluLookAt( 18.0f, 0, 0,  0, 0, 0,  0, 1.0f, 0 );
    glRotatef( -this->m_Phi, 0, 0, 1 );
    glRotatef( this->m_Theta, 0, 1, 0 );
    glTranslatef( 0, -5.0f, 0 );
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
    switch( key )
    {
        case 'R': case 'r':
            // Reset the simulation
            this->Reset();
            return;
            
        case 'C': case 'c':
            // Toggle rendering of contacts
            this->m_RenderDebugInfo = !this->m_RenderDebugInfo;
            return;
            
        case 'P': case 'p':
            // Toggle running the simulation
            this->m_PauseSimulation = !this->m_PauseSimulation;
            return;
            
        case ' ':
            // Advance one frame
            this->m_AutoPauseSimulation = true;
            this->m_PauseSimulation = false;
    }
    
    Application::Key( key );
}