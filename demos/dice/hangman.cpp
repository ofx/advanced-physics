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

inline cyclone::real VectorLength( cyclone::real x0, cyclone::real y0, cyclone::real x1, cyclone::real y1 )
{
    cyclone::real x = x0 - x1;
    cyclone::real y = y0 - y1;
    
    return sqrt( (x * x) + (y * y) );
}

class DiceDemo : public MassAggregateApplication
{
    cyclone::ParticleCableConstraint *m_Supports;
    cyclone::ParticleRod *m_Cables;
public:
    DiceDemo( void );
    virtual ~DiceDemo( void );
    
    virtual const char* GetTitle( void );
    
    virtual void Display( void );
    virtual void Update( void );
    virtual void Key( unsigned char key );
};

DiceDemo::DiceDemo( void ) : MassAggregateApplication( 0 )
{

}

DiceDemo::~DiceDemo()
{

}

void DiceDemo::Display( void )
{

}

void DiceDemo::Update()
{
    MassAggregateApplication::Update();
}

const char *DiceDemo::GetTitle()
{
    return "Dice";
}

void DiceDemo::Key( unsigned char key )
{
    MassAggregateApplication::Key( key );
}

Application* getApplication( void )
{
    return new DiceDemo();
}