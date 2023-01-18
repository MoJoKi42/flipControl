#include <stdint.h>


#define _RINGBUFFER_SIZE 2048
unsigned char _RINGBUFFER[_RINGBUFFER_SIZE];
uint32_t _ringbuffer_pos_write = 0;
uint32_t _ringbuffer_pos_read = 0;


// init/clear ringbuffer
void ringbuffer_init() {
    _ringbuffer_pos_write = 0;
    _ringbuffer_pos_read = 0;
}


// fill ringbuffer with data
void ringbuffer_pushback(unsigned char data) {
    _RINGBUFFER[_ringbuffer_pos_write] = data;
    _ringbuffer_pos_write++;
    if (_ringbuffer_pos_write >= _RINGBUFFER_SIZE) {
        _ringbuffer_pos_write = 0;
    }
}


// read data from ringbuffer
unsigned char ringbuffer_get() {

    // nothing to read
    if (_ringbuffer_pos_read == _ringbuffer_pos_write) {
        return 0;
    }

    // read data
    unsigned char temp = _RINGBUFFER[_ringbuffer_pos_read];
    _ringbuffer_pos_read++;
    if (_ringbuffer_pos_read >= _RINGBUFFER_SIZE) {
        _ringbuffer_pos_read = 0;
    }
    return temp;
}
