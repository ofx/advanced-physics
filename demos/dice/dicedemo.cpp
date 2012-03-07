#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif
#include <cyclone/cyclone.h>
#include "app.h"

#include <stdio.h>
#include <cassert>

Application* getApplication( void );

class Dice : public cyclone::CollisionBox
{
public:
    Dice( void )
    {
        this->body = new cyclone::RigidBody;
    }

    ~Dice( void )
    {
        delete this->body;
    }

    void render( void )
    {
        GLfloat mat[16];
        this->body->getGLTransform( mat );

        glPushMatrix();
            glMultMatrixf( mat );
            glScalef( halfSize.x * 2, halfSize.y * 2, halfSize.z * 2 );
            glutSolidCube( 1.0 );
        glPopMatrix();
    }

    void Update( cyclone::real duration )
    {
        this->body->integrate( duration );
        this->calculateInternals();
    }

    void SetState( cyclone::real x, cyclone::real y, cyclone::real z )
    {
        this->body->setPosition( x, y, z );
        this->body->setOrientation( 1, 0, 0, 0 );
        this->body->setVelocity( 0, 0, 0 );
        this->body->setRotation( cyclone::Vector3( 0, 0, 0 ) );
        this->halfSize = cyclone::Vector3( 1, 1, 1 );

        cyclone::real mass = this->halfSize.x * this->halfSize.y * this->halfSize.z * 8.0f;
        this->body->setMass( mass );

        cyclone::Matrix3 tensor;
        tensor.setBlockInertiaTensor( this->halfSize, mass );
        this->body->setInertiaTensor( tensor );

        this->body->setLinearDamping( 0.95 );
        this->body->setAngularDamping( 0.8 );
        this->body->clearAccumulators();
        this->body->setAcceleration( 0, -10.0, 0 );

        //this->body->setCanSleep( false );
        this->body->setAwake();

        this->body->calculateDerivedData();
    }
};

class DiceDemo : public RigidBodyApplication
{
private:
    Dice *m_Dice;
public:
    DiceDemo( void );
    virtual ~DiceDemo( void );
    
    // Returns the window title
    virtual const char* GetTitle( void );

    // Initializes graphics
    virtual void InitGraphics( void );

    // Build the contacts for the current situation
    virtual void GenerateContacts( void );
    // Processes the objects in the simulation forward in time
    virtual void UpdateObjects( cyclone::real duration );

    // Resets the world
    virtual void Reset( void );
    
    // Display the world
    virtual void Display( void );
    // Handle keypress
    virtual void Key( unsigned char key );
    // Handles mouse actions
    virtual void Mouse( int button, int state, int x, int y );
};

DiceDemo::DiceDemo( void )
{
    this->m_Dice = new Dice();
    this->m_Dice->SetState( 0.0, 10.0, 20.0 );
}

DiceDemo::~DiceDemo()
{
    delete this->m_Dice;
}

void DiceDemo::Display( void )
{
    const static GLfloat lightPosition[] = {-1, 1, 0, 0};
    
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    gluLookAt( -25.0, 8.0, 5.0,  0.0, 5.0, 22.0,  0.0, 1.0, 0.0 );

    // Draw some scale lines
    glColor3f( 0, 0, 1.0 );
    glBegin( GL_LINES );
        for( unsigned i = 0 ; i < 200 ; i += 10 )
        {
            glVertex3f( -5.0, 0.0, i );
            glVertex3f( 5.0, 0.0, i );
        }
    glEnd();

    // Draw the dice
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_LIGHTING );
    glLightfv( GL_LIGHT0, GL_POSITION, lightPosition );
    glColorMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE );
    glEnable( GL_COLOR_MATERIAL );
    glColor3f( 1.0, 1.0, 1.0 );
    this->m_Dice->render();
    glDisable( GL_COLOR_MATERIAL );
    glDisable( GL_LIGHTING );
    glDisable( GL_DEPTH_TEST );

    // Render some text
    glColor3f( 0.0, 0.0, 0.0 );
    this->RenderText( 10.0, 10.0, "Mouse click to pick up the dice" );
}

const char *DiceDemo::GetTitle()
{
    return "Dice Demo";
}

void DiceDemo::InitGraphics( void )
{
    GLfloat lightAmbient[] = {0.8f, 0.8f, 0.8f, 1.0f};
    GLfloat lightDiffuse[] = {0.9f, 0.95f, 1.0f, 1.0f};

    glLightfv( GL_LIGHT0, GL_AMBIENT, lightAmbient );
    glLightfv( GL_LIGHT0, GL_DIFFUSE, lightDiffuse );

    glEnable( GL_LIGHT0 );

    Application::InitGraphics();
}

void DiceDemo::Mouse( int button, int state, int x, int y )
{

}

void DiceDemo::GenerateContacts( void )
{
    // Create a ground plane
    cyclone::CollisionPlane plane;
    plane.direction = cyclone::Vector3( 0, 1, 0 );
    plane.offset = 0;

    this->m_CollisionData.reset( RigidBodyApplication::s_MaxContacts );
    this->m_CollisionData.friction = (cyclone::real) 0.9;
    this->m_CollisionData.restitution = (cyclone::real) 0.1;
    this->m_CollisionData.tolerance = (cyclone::real) 0.1;

    cyclone::CollisionDetector::boxAndHalfSpace( *this->m_Dice, plane, &this->m_CollisionData );
}

void DiceDemo::UpdateObjects( cyclone::real duration )
{
    this->m_Dice->Update( duration );
}

void DiceDemo::Reset( void )
{

}

void DiceDemo::Key( unsigned char key )
{
    RigidBodyApplication::Key( key );
}

Application* getApplication( void )
{
    return new DiceDemo();
}