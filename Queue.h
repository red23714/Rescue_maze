#pragma once

class Queue 
{   
public :
    Queue() 
    {
        queue = new int[SIZE];
        head = tail = 0; 
    } 

    void push(int num) 
    { 
        if (tail + 1 == head || (tail + 1 == SIZE && !head)) return; 
        tail++; 
        if(tail == SIZE) tail = 0;
        queue[tail] = num; 
    } 

    void pop() 
    { 
        if(head == tail) return;
        head++; 
        if (head == SIZE) head = 0; 
    } 

    bool empty()
    {
        return head == tail;
    }
    
    int size()
    {   
        int s = 0;
        for(int i = head; i < tail; i++)
        {
            s++;
        }
        return s;
    }
    
    int back()
    {
        return queue[tail];
    }
    
    int front()
    {
        return queue[head + 1];
    }

private:
    static const int SIZE=100; 
    int *queue; 
    int head, tail; 
} ; 