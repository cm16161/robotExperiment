#define ROOT_MAX 5
#define MAX_SIZE ROOT_MAX * ROOT_MAX

typedef struct {
  int x;
  int y;
} Coordinate;

class Stack {
  private:
    Stack (){};
  public:
   static Stack& getInstance(){
      static Stack stack;
      return stack;
   }
    void push(Coordinate c);
    Coordinate pop();
    Coordinate peak();
    bool isEmpty();

    Coordinate m_stack[MAX_SIZE];
    int m_index = 0;
};

void Stack::push(Coordinate c) {
  if (m_index == MAX_SIZE) {
    return;
  }
  m_stack[m_index] = c;
  m_index++;
}

Coordinate Stack::pop() {
  m_index--;
  if (m_index == -1) {
    m_index = 0;
    return Coordinate{-1, -1};
  }
  return m_stack[m_index];
}

Coordinate Stack::peak() {
  if (m_index <= 0) {
    return Coordinate{-1, -1};
  }
  return m_stack[m_index - 1];
}

bool Stack::isEmpty(){
  return (m_index == 0);
}
