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

class DiceDemo : public RigidBodyApplication
{
    cyclone::ParticleCableConstraint *m_Supports;
    cyclone::ParticleRod *m_Cables;
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

}

DiceDemo::~DiceDemo()
{

}

void DiceDemo::Display( void )
{

}

const char *DiceDemo::GetTitle()
{
    return "Dice";
}

void DiceDemo::InitGraphics( void )
{

}

void DiceDemo::Mouse( int button, int state, int x, int y )
{

}

void DiceDemo::GenerateContacts( void )
{

}

void DiceDemo::UpdateObjects( cyclone::real duration )
{

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