#ifndef NODE_H
#define NODE_H

class node
{

public:
    node(int xp, int yp, int d, int p);

    int getxPos() const;
    int getyPos() const;
    int getLevel() const;
    int getPriority() const;

    void updatePriority(const int & xDest, const int & yDest);
    void nextLevel(const int & i);

    const int & estimate(const int & xDest, const int & yDest) const;

private:
    // current position
    int m_xPos;
    int m_yPos;
    // total distance already travelled to reach the node
    int m_level;
    // priority=level+remaining distance estimate
    int m_priority;  // smaller: higher priority

};

bool operator<(const node & a, const node & b);

#endif // NODE_H
