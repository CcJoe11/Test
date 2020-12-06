#ifndef ROUNDQUEUE_H
#define ROUNDQUEUE_H

template<class type, int length>
class RoundQueue {
private:
    type data[length];
    int head;
    int tail;
public:
    int getHead(){return head;}
    RoundQueue<type, length>() : head(0), tail(0) {}

     int size() const {
        return length;
    };

    bool empty() const {
        return head == tail;
    };

    void push(const type &obj) {
        data[head] = obj;
        head = (head + 1) % length;
        if (head == tail) {
            tail = (tail + 1) % length;
        }
    };

    bool pop(type &obj) {
        if (empty()) return false;
        obj = data[tail];
        tail = (tail + 1) % length;
        return true;
    };

    type &operator[](int idx) {
        while (tail + idx < 0) idx += length;
        return data[(tail + idx) % length];
    };
};

#endif // ROUNDQUEUE_H
