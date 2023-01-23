#pragma once 
#include <memory>   
template <class T>
class ring_buffer {
public:
	explicit ring_buffer(size_t size) :
		buf_(std::unique_ptr<T[]>(new T[size])),
		max_size_(size)
	{}

	// Fills the ring buffer with initial.
	explicit ring_buffer(size_t size, T initial):
	ring_buffer(size) {
		for (size_t i = 0; i < size; ++i) {
			put(initial);
		}
	}

	void put(T item){
		buf_[head_] = item;

		if(full_){
			tail_ = (tail_ + 1) % max_size_;
		}

		head_ = (head_ + 1) % max_size_;

		full_ = head_ == tail_;
	}
	
	T pop(){
		if(empty()){
			return T();
		}
		//Read data and advance the tail (we now have a free space)
		T val = buf_[tail_];
		full_ = false;
		tail_ = (tail_ + 1) % max_size_;

		return val;
	}

	T get(size_t i){
		return buf_[tail_ + i] % max_size_;
	}

	void reset(){
		head_ = tail_;
		full_ = false;
	}

	bool empty() {return (!full_ && (head_ == tail_));}

	bool full() {return full_;}

	size_t capacity() const {
		return max_size_;
	}
	size_t size() const {
		size_t size = max_size_;

		if(!full_)
		{
			if(head_ >= tail_)
			{
				size = head_ - tail_;
			}
			else
			{
				size = max_size_ + head_ - tail_;
			}
		}
	return size;
	}

private:
	std::unique_ptr<T[]> buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	const size_t max_size_;
	bool full_ = 0;
};
