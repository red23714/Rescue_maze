#pragma once

#include <Arduino.h>

template<typename T>
class Vec 
{
public:
    Vec() 
    {
        arr_ = new T[1];
        capacity_ = 1;
    }

    Vec(int size) 
    {
        size_ = (size_t)size;
        capacity_ = size_*2;
        arr_ = new T[capacity_];
    }

    ~Vec() 
    {
	    delete[] arr_;
    }

    [[nodiscard]] bool isEmpty() const 
    {
		return size_ == 0;
    }
    [[nodiscard]] size_t size() const 
    {
		return size_;
    }

    void push_back(const T& value) 
    {
		if (size_ >= capacity_) addMemory();
        arr_[size_++] = value;
    }

    void remove(size_t index) 
    {
		for (size_t i = index + 1; i < size_; ++i) 
        {
    		arr_[i - 1] = arr_[i];
        }
        --size_;
    }

    T& operator[](size_t index)  
    {
		return arr_[index];
    }

    const T& operator[](size_t index) const 
    {
		return arr_[index];
    }

    T* begin() 
    {
		return &arr_[0];
    }
    const T* begin() const 
    {
        return &arr_[0];
    }
    T* end() 
    {
        return &arr_[size_];
    }
    const T* end() const 
    {
        return &arr_[size_];
    }

    Vec(Vec& other) 
    {
        if (this != &other) 
        {
            delete[] arr_;
            arr_ = new T[other.capacity_];

            for (size_t i = 0; i < other.size_; ++i) arr_[i] = other.arr_[i];

            size_ = other.size_;
            capacity_ = other.capacity_;
        }
    }
    Vec(Vec&& other) noexcept 
    {
        if (this != &other) 
        {
            delete[] arr_;
            arr_ = other.arr_;
            size_ = other.size_;
            capacity_ = other.capacity_;
            other.arr_ = nullptr;
            other.size_ = other.capacity_ = 0;
        }
    }

    Vec& operator=(const Vec& other) 
    {
        if (this != &other) {
            delete[] arr_;
            arr_ = new T[other.capacity_];

            for (size_t i = 0; i < other.size_; ++i) arr_[i] = other.arr_[i];

            size_ = other.size_;
            capacity_ = other.capacity_;
        }
        return *this;
    }

    Vec& operator=(Vec&& other) noexcept 
    {
        if (this != &other) 
        {
            delete[] arr_;
            arr_ = other.arr_;
            size_ = other.size_;
            capacity_ = other.capacity_;
            other.arr_ = nullptr;
            other.size_ = other.capacity_ = 0;
        }
        return *this;
    }

private:
    T* arr_;
    size_t size_{};
    size_t capacity_{};

    void addMemory() 
    {
		capacity_ *= 2;
        T* tmp = arr_;
        arr_ = new T[capacity_];
        for (size_t i = 0; i < size_; ++i) arr_[i] = tmp[i];
        delete[] tmp;
    }
};