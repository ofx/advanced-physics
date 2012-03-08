#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <gl/glut.h>
#endif
#include <cyclone/cyclone.h>

#include "app.h"
#include "squadric.h"

#include <list>
#include <stdio.h>
#include <cassert>

#define DICE_ROUNDING_FACTOR 0.75

#define PICK_NAME_DICE_OFFSET 10
#define PICK_BUFFER_SIZE      256
#define PICK_TOLERANCE        10

static bool s_DebugDraw = true;

static int s_Dices = 0;

Application* getApplication( void );

class Dice : public cyclone::CollisionBox
{
protected:
    cyclone::CollisionSphere *m_RoundingSphere;

    GLint m_Name;
public:
    Dice( void )
    {
        this->body = new cyclone::RigidBody;
        this->m_Name = s_Dices++;

        this->m_RoundingSphere = new cyclone::CollisionSphere();
        this->m_RoundingSphere->body = new cyclone::RigidBody();
    }

    virtual ~Dice( void )
    {
        delete this->body;

        delete this->m_RoundingSphere;
    }

    virtual void render( void ) = 0;

    virtual void Update( cyclone::real duration ) = 0;
    virtual void DoCollisionTest( cyclone::CollisionPlane plane, cyclone::CollisionData *collisionData ) = 0;
    virtual void SetState( cyclone::real x, cyclone::real y, cyclone::real z ) = 0;
};

class EightSidedDice : public Dice
{
private:
    cyclone::CollisionSphere *m_RoundingSphere;
public:
    EightSidedDice( void )
    {
        this->body = new cyclone::RigidBody;

        this->m_RoundingSphere = new cyclone::CollisionSphere;
        this->m_RoundingSphere->body = new cyclone::RigidBody();
    }

    ~EightSidedDice( void )
    {
        delete this->body;

        delete this->m_RoundingSphere;
    }

    void render( void )
    {
        GLfloat mat[16];
        this->body->getGLTransform( mat );

        glPushMatrix();
            glMultMatrixf( mat );
            glPushMatrix();
                if( s_DebugDraw )
                {
                    glScalef( halfSize.x * 2, halfSize.y * 2, halfSize.z * 2 );
                    glutWireCube( 1.0 );
                    glutWireSphere( this->m_RoundingSphere->radius, 30, 30 );
                }
            glPopMatrix();

            glPushMatrix();
                glScalef( halfSize.x, halfSize.y, halfSize.z );
				sqSolidDoublePyramid( this->m_RoundingSphere->radius, 30, 20 );
            glPopMatrix();
        glPopMatrix();
    }

    void Update( cyclone::real duration )
    {
        this->body->integrate( duration );
        this->calculateInternals();

        // Update the rounding sphere position
        this->m_RoundingSphere->body->setPosition( this->body->getPosition() );
    }

    void DoCollisionTest( cyclone::CollisionPlane plane, cyclone::CollisionData *collisionData )
    {
        if( cyclone::IntersectionTests::boxAndHalfSpace( *this, plane ) && cyclone::IntersectionTests::sphereAndHalfSpace( *this->m_RoundingSphere, plane ) )
        {
            cyclone::CollisionDetector::boxAndHalfSpace( *this, plane, collisionData );
        }
    }

    void SetState( cyclone::real x, cyclone::real y, cyclone::real z )
    {
        // Dice body
        {
            this->body->setPosition( x, y, z );
            this->body->setOrientation( 1, 0, 0, 0 );
            this->body->setVelocity( 0, 0, 0 );
            this->body->setRotation( cyclone::Vector3( 0, 0, 0 ) );
            this->halfSize = cyclone::Vector3( 1, 1, 1 );

            assert( this->halfSize.x == this->halfSize.y && this->halfSize.y == this->halfSize.z );

            cyclone::real mass = this->halfSize.x * this->halfSize.y * this->halfSize.z * 8.0f;
            this->body->setMass( mass );

            cyclone::Matrix3 tensor;
            tensor.setBlockInertiaTensor( this->halfSize, mass );
            this->body->setInertiaTensor( tensor );

            this->body->setLinearDamping( 0.95 );
            this->body->setAngularDamping( 0.8 );
            this->body->clearAccumulators();
            this->body->setAcceleration( 0, -10.0, 0 );

            this->body->setAwake();

            this->body->calculateDerivedData();
        }

        // Rounding sphere body
        {
            this->m_RoundingSphere->radius = this->halfSize.x *= DICE_ROUNDING_FACTOR;
        }
    }
};

class SixSidedDice : public Dice
{
private:
    cyclone::CollisionSphere *m_RoundingSphere;
public:
    SixSidedDice( void )
    {
        this->body = new cyclone::RigidBody;

        this->m_RoundingSphere = new cyclone::CollisionSphere();
        this->m_RoundingSphere->body = new cyclone::RigidBody();
    }

    ~SixSidedDice( void )
    {
        delete this->body;

        delete this->m_RoundingSphere;
    }

    void render( void )
    {
        GLfloat mat[16];
        this->body->getGLTransform( mat );

        glPushMatrix();
            glMultMatrixf( mat );
            glPushMatrix();
                if( s_DebugDraw )
                {
                    glScalef( halfSize.x * 2, halfSize.y * 2, halfSize.z * 2 );
                    glutWireCube( 1.0 );
                    glutWireSphere( this->m_RoundingSphere->radius, 30, 30 );
                }
            glPopMatrix();

            glPushMatrix();
                glScalef( halfSize.x, halfSize.y, halfSize.z );
                glLoadName( this->m_Name + PICK_NAME_DICE_OFFSET );
                sqSolidRoundCube( this->m_RoundingSphere->radius, 30, 30 );
            glPopMatrix();
        glPopMatrix();
    }

    void Update( cyclone::real duration )
    {
        this->body->integrate( duration );
        this->calculateInternals();

        // Update the rounding sphere position
        this->m_RoundingSphere->body->setPosition( this->body->getPosition() );
    }

    void DoCollisionTest( cyclone::CollisionPlane plane, cyclone::CollisionData *collisionData )
    {
        if( cyclone::IntersectionTests::boxAndHalfSpace( *this, plane ) && cyclone::IntersectionTests::sphereAndHalfSpace( *this->m_RoundingSphere, plane ) )
        {
            cyclone::CollisionDetector::boxAndHalfSpace( *this, plane, collisionData );
        }
    }

    void SetState( cyclone::real x, cyclone::real y, cyclone::real z )
    {
        // Dice body
        {
            this->body->setPosition( x, y, z );
            this->body->setOrientation( 1, 0, 0, 0 );
            this->body->setVelocity( 0, 0, 0 );
            this->body->setRotation( cyclone::Vector3( 0, 0, 0 ) );
            this->halfSize = cyclone::Vector3( 1, 1, 1 );

            assert( this->halfSize.x == this->halfSize.y && this->halfSize.y == this->halfSize.z );

            cyclone::real mass = this->halfSize.x * this->halfSize.y * this->halfSize.z * 8.0f;
            this->body->setMass( mass );

            cyclone::Matrix3 tensor;
            tensor.setBlockInertiaTensor( this->halfSize, mass );
            this->body->setInertiaTensor( tensor );

            this->body->setLinearDamping( 0.95 );
            this->body->setAngularDamping( 0.8 );
            this->body->clearAccumulators();
            this->body->setAcceleration( 0, -10.0, 0 );

            this->body->setAwake();

            this->body->calculateDerivedData();
        }

        // Rounding sphere body
        {
            this->m_RoundingSphere->radius = this->halfSize.x *= DICE_ROUNDING_FACTOR;
        }
    }
};

class DiceDemo : public RigidBodyApplication
{
private:
    std::list<Dice*> m_Dices;

    unsigned int m_PickBuffer[PICK_BUFFER_SIZE];
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

    // Do GL picking
    virtual void Select( int x, int y );
    
    // Display the world
    virtual void Display( void );
    // Handle keypress
    virtual void Key( unsigned char key );
    // Handles mouse actions
    virtual void Mouse( int button, int state, int x, int y );
};

DiceDemo::DiceDemo( void )
{
    Dice *d;

    this->m_Dices.push_back( d = new SixSidedDice() );
    d->SetState( 0.0, 10.0, 20.0 );
}

static bool deleteElm( Dice *d )
{
    delete d; return true;
}

DiceDemo::~DiceDemo()
{
    this->m_Dices.remove_if( deleteElm );
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
    
    std::list<Dice*>::const_iterator it;
    for( it = this->m_Dices.begin() ; it != this->m_Dices.end() ; ++it )
    {
        (*it)->render();
    }

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

void DiceDemo::Select( int x, int y )
{
    GLuint b[PICK_BUFFER_SIZE] = { 0 };
    GLint h, v[4];
    int id;

    glSelectBuffer( PICK_BUFFER_SIZE, b );
    
    glGetIntegerv( GL_VIEWPORT, v );

    glRenderMode( GL_SELECT );

        glInitNames();
        glPushName( 0 );

        glMatrixMode( GL_PROJECTION );
        glPushMatrix();
            glLoadIdentity();

            gluPickMatrix( x, v[3] - y, PICK_TOLERANCE, PICK_TOLERANCE, v );
            gluPerspective( 60.0, (double) this->m_Width / (double) this->m_Height, 1.0, 500.0 );

            glMatrixMode( GL_MODELVIEW );

            glutSwapBuffers();

            this->Display();

            glMatrixMode( GL_PROJECTION );
        glPopMatrix();

    h = glRenderMode( GL_RENDER );

    printf( "%i hits\n", h );

    glMatrixMode( GL_MODELVIEW );
}

void DiceDemo::Mouse( int button, int state, int x, int y )
{
    if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN) )
    {
        this->Select( x, y );
    }
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

    std::list<Dice*>::const_iterator it;
    for( it = this->m_Dices.begin() ; it != this->m_Dices.end() ; ++it )
    {
        (*it)->DoCollisionTest( plane, &this->m_CollisionData );
    }
}

void DiceDemo::UpdateObjects( cyclone::real duration )
{
    std::list<Dice*>::const_iterator it;
    for( it = this->m_Dices.begin() ; it != this->m_Dices.end() ; ++it )
    {
        (*it)->Update( duration );
    }
}

void DiceDemo::Reset( void )
{

}

void DiceDemo::Key( unsigned char key )
{
    switch( key )
    {
        case 'D': case 'd':
            s_DebugDraw = !s_DebugDraw;
            break;
    }

    RigidBodyApplication::Key( key );
}

Application* getApplication( void )
{
    return new DiceDemo();
}