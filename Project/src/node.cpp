#include "node.h"
#include "math.h"

node::node(int xp, int yp, int d, int p)
{
    m_xPos=xp;
    m_yPos=yp;
    m_level=d;
    m_priority=p;
}

int node::getxPos() const
{
    return m_xPos;
}

int node::getyPos() const
{
    return m_yPos;
}

int node::getLevel() const
{
    return m_level;
}

int node::getPriority() const
{
    return m_priority;
}

void node::updatePriority(const int & xDest, const int & yDest)
{
     m_priority = m_level+estimate(xDest, yDest)*10; //A*
}

// give better priority to going strait instead of diagonally
void node::nextLevel(const int & i) // i: direction
{
    //assume 8 directions
    m_level += (i%2 == 0 ? 10 : 14);
}

// Estimation function for the remaining distance to the goal.
const int & node::estimate(const int & xDest, const int & yDest) const
{
    static int xd, yd, d;
    xd = xDest-m_xPos;
    yd = yDest-m_yPos;

    // Euclidian Distance
    d = static_cast<int>(sqrt(xd*xd+yd*yd));

    // Manhattan distance
    //d=abs(xd)+abs(yd);

    // Chebyshev distance
    //d=max(abs(xd), abs(yd));

    return(d);
}


// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
    return a.getPriority() > b.getPriority();
}

