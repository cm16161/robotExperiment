#include "stack.h"

typedef struct
{
  Coordinate neighbours[4];
} Neighbours;

typedef struct {
  float theta;
  int distance; // Distance measured in Encoder Clicks
} Goto;

class FloodFill {
  public:
    FloodFill() {
      for (int i = 0; i < MAX_SIZE; i++) {
        m_visited[i] = false;
        m_added[i] = false;
        m_ordering[i] = -1;
      }
    }

    bool m_visited[MAX_SIZE];
    bool m_added[MAX_SIZE];
    int m_ordering[MAX_SIZE];
    
    Stack& m_stack = Stack::getInstance();

    Neighbours getNeighbours(Coordinate c);
    bool validateCoordinate(Coordinate c); // This is going to validate the height, width, visited
    bool validateStack(Coordinate c);
    void addToStack(Coordinate c);
    Coordinate getCoordinate();
    Goto rotateTo(Coordinate src, Coordinate dst);
    bool visited(Coordinate c); // Use Hash Function: index = 25x+y
    void addPathNumeration(Coordinate c, int index); // Use Hash Function: index = 25x+y
    int getPathNumeration(Coordinate c); // Use Hash Function: index = 25x+y
    bool onStack(Coordinate c);
    void addToVisited(Coordinate c); // Use Hash Function: index = 25x+y
    void addToAdded(Coordinate c);
    bool isEmpty(); // Check if underlying stack is empty
};

Neighbours FloodFill::getNeighbours(Coordinate c) {
  Neighbours ret;
  ret.neighbours[0] = Coordinate{c.x - 1, c.y};
  ret.neighbours[1] = Coordinate{c.x, c.y - 1};
  ret.neighbours[2] = Coordinate{c.x + 1, c.y};
  ret.neighbours[3] = Coordinate{c.x, c.y + 1};
  return ret;
}

bool FloodFill::validateCoordinate(Coordinate c) {
  if (c.x >= ROOT_MAX || c.x < 0 || c.y >= ROOT_MAX || c.y < 0) {
    return false;
  }
  return !visited(c);
}

bool FloodFill::validateStack(Coordinate c) {
  return !onStack(c);
}

void FloodFill::addToStack(Coordinate c) {
  m_stack.push(c);
}

Coordinate FloodFill::getCoordinate() {
  return m_stack.pop();
}

Goto FloodFill::rotateTo(Coordinate src, Coordinate dst) {
  //TODO
  return Goto{0, 0};
}

void FloodFill::addPathNumeration(Coordinate c, int order) {
  m_ordering[ROOT_MAX*c.x + c.y] = order;
}

int FloodFill::getPathNumeration(Coordinate c){
  return m_ordering[ROOT_MAX*c.x + c.y];
}

bool FloodFill::visited(Coordinate c) {
  return m_visited[ROOT_MAX*c.x + c.y];
}

bool FloodFill::onStack(Coordinate c) {
  return m_added[ROOT_MAX * c.x + c.y];
}

void FloodFill::addToVisited(Coordinate c) {
  m_visited[ROOT_MAX * c.x + c.y] = true;
}

void FloodFill::addToAdded(Coordinate c) {
  m_added[ROOT_MAX * c.x + c.y] = true;
}

bool FloodFill::isEmpty() {
  return m_stack.isEmpty();
}
