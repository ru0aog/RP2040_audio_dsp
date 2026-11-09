// src/af_queue.h  
//  
// Однопроизводительно-однопотребительная (SPSC) очередь указателей  
// на блоки int16. Используется дважды:  
//   af_capture (Core1 IRQ) -> af_dsp (Core1 loop)  
//   af_dsp (Core1)         -> af_pwm/af_usb (Core0)  
//  
// Без блокировок: head пишет только producer, tail — только consumer.  
  
#pragma once  
  
#include <Arduino.h>  
  
template <typename T, uint8_t N>  
struct AfQueue {  
    volatile uint8_t head = 0;   // пишет producer  
    volatile uint8_t tail = 0;   // пишет consumer  
    T* slots[N];  
  
    // producer: положить блок; false если очередь полна  
    inline bool push(T* p) {  
        uint8_t next = (uint8_t)((head + 1) % N);  
        if (next == tail) return false;  
        slots[head] = p;  
        __dmb();  
        head = next;  
        return true;  
    }  
  
    // consumer: взять блок; nullptr если пусто  
    inline T* pop() {  
        if (tail == head) return nullptr;  
        T* p = slots[tail];  
        __dmb();  
        tail = (uint8_t)((tail + 1) % N);  
        return p;  
    }  
  
    inline bool empty() const { return tail == head; }  
};
