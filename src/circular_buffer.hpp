#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <vector>
#include <stdexcept>

// 与boost库的使用方法相同；

template <typename T>
class circular_buffer {
public:

    // 默认构造函数
    circular_buffer()
        : capacity_(0), head_(0), tail_(0), size_(0) {
    }


    explicit circular_buffer(size_t capacity)
        : capacity_(capacity), buffer_(capacity), head_(0), tail_(0), size_(0) {
        if (capacity == 0) {
            throw std::invalid_argument("Capacity must be greater than 0");
        }
    }

    // Push a new element to the buffer
    void push_back(const T& value) {
        buffer_[tail_] = value;
        tail_ = (tail_ + 1) % capacity_;
        if (size_ == capacity_) {
            head_ = (head_ + 1) % capacity_; // Overwrite oldest element
        }
        else {
            ++size_;
        }
    }

    void push_back(T&& value) {
        buffer_[tail_] = std::move(value);
        tail_ = (tail_ + 1) % capacity_;
        if (size_ == capacity_) {
            head_ = (head_ + 1) % capacity_;
        }
        else {
            ++size_;
        }
    }

    // Push a new element to the front of the buffer
    void push_front(const T& value) {
        head_ = (head_ + capacity_ - 1) % capacity_;
        buffer_[head_] = value;
        if (size_ == capacity_) {
            tail_ = (tail_ + capacity_ - 1) % capacity_; // Overwrite newest element
        }
        else {
            ++size_;
        }
    }

    void push_front(T&& value) {
        head_ = (head_ + capacity_ - 1) % capacity_;
        buffer_[head_] = std::move(value);
        if (size_ == capacity_) {
            tail_ = (tail_ + capacity_ - 1) % capacity_;
        }
        else {
            ++size_;
        }
    }

    // Remove the oldest element
    void pop_front() {
        if (empty()) {
            throw std::out_of_range("Buffer is empty");
        }
        head_ = (head_ + 1) % capacity_;
        --size_;
    }

    // Access the oldest element
    T& front() {
        if (empty()) {
            throw std::out_of_range("Buffer is empty");
        }
        return buffer_[head_];
    }

    const T& front() const {
        if (empty()) {
            throw std::out_of_range("Buffer is empty");
        }
        return buffer_[head_];
    }

    // Access the newest element
    T& back() {
        if (empty()) {
            throw std::out_of_range("Buffer is empty");
        }
        return buffer_[(tail_ + capacity_ - 1) % capacity_];
    }

    const T& back() const {
        if (empty()) {
            throw std::out_of_range("Buffer is empty");
        }
        return buffer_[(tail_ + capacity_ - 1) % capacity_];
    }

    // Assign the buffer with repeated values
    void assign(size_t count, const T& value) {
        if (count == 0) {
            throw std::invalid_argument("Count must be greater than 0");
        }
        buffer_.resize(count);
        capacity_ = count;
        clear();
        for (size_t i = 0; i < count; ++i) {
            push_back(value);
        }
    }

    // Check if the buffer is empty
    bool empty() const {
        return size_ == 0;
    }

    // Check if the buffer is full
    bool full() const {
        return size_ == capacity_;
    }

    // Get the number of elements in the buffer
    size_t size() const {
        return size_;
    }

    // Get the capacity of the buffer
    size_t capacity() const {
        return capacity_;
    }

    // Clear the buffer
    void clear() {
        head_ = 0;
        tail_ = 0;
        size_ = 0;
    }

    // Access an element by index (0 = oldest)
    T& operator[](size_t index) {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        return buffer_[(head_ + index) % capacity_];
    }

    const T& operator[](size_t index) const {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        return buffer_[(head_ + index) % capacity_];
    }

private:
    size_t capacity_;
    std::vector<T> buffer_;
    size_t head_;
    size_t tail_;
    size_t size_;
};

#endif // CIRCULAR_BUFFER_HPP
