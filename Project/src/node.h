#ifndef NODE_H
#define NODE_H

/**
 * @brief
 *
 */
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
    int m_xPos;         /**< Current position (x coordinate) */
    int m_yPos;         /**< Current position (y coordinate) */
    int m_level;        /**< Total distance already travelled to reach the node */
    int m_priority;     /**< Level + remaining distance estimate (smaller: higher priority) */

};

bool operator<(const node & a, const node & b);

#endif // NODE_H
