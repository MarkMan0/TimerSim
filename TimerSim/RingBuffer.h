#pragma once

#include <cstdint>

template<class T, uint8_t BUFFER_SIZE>
class RingBuffer {

public:
	static constexpr uint8_t BUFF_SZ = BUFFER_SIZE;

	using block_t = T;
	using block_ptr = T*;


	bool isEmpty() const;
	bool isFull() const;
	bool hasFree() const;
	bool hasItems() const;

	void reset();

	//returns pointer to next free block
	block_ptr getHead();

	//returns pointer to next block to read
	block_ptr getTail();

	//call to commit the changes made to current head
	void commitHead();

	//call when the current tail is handled to mark it as free
	void advanceTail();

	uint8_t size() const;

private:
	block_t buffer_[BUFFER_SIZE] = { 0 };

	uint8_t head_ = 0, tail_ = 0;
	bool full_ = false;


};


template<class T, uint8_t BS>
bool RingBuffer<T, BS>::isEmpty() const{
	return (!full_ && head_ == tail_);
}

template<class T, uint8_t BS>
bool RingBuffer<T, BS>::hasItems() const {
	return !isEmpty();
}

template<class T, uint8_t BS>
bool RingBuffer<T, BS>::isFull() const {
	return full_;
}

template<class T, uint8_t BS>
bool RingBuffer<T, BS>::hasFree() const {
	return !isFull();
}

template<class T, uint8_t BS>
void RingBuffer<T, BS>::reset() {
	head_ = tail_;
}

template<class T, uint8_t BS>
typename RingBuffer<T, BS>::block_ptr RingBuffer<T, BS>::getHead() {
	return (buffer_ + head_);
}

template<class T, uint8_t BS>
typename RingBuffer<T, BS>::block_ptr RingBuffer<T, BS>::getTail() {
	return (buffer_ + tail_);
}

template<class T, uint8_t BS>
void RingBuffer<T, BS>::commitHead() {

	if (full_) {	//overwrite if full
		tail_ = (tail_ + 1) % BUFF_SZ;
	}
	head_ = (head_ + 1) % BUFF_SZ;

	full_ = head_ == tail_;
}

template<class T, uint8_t BS>
void RingBuffer<T, BS>::advanceTail() {
	tail_ = (tail_ + 1) % BUFF_SZ;
	full_ = false;
}

template<class T, uint8_t BS>
uint8_t RingBuffer<T, BS>::size() const {
	if (!full_)	{
		if (head_ >= tail_)	{
			return head_ - tail_;
		}
		else {
			return BUFF_SZ + head_ - tail_;
		}
	}

	return BUFF_SZ;
}