#ifndef SPSC_QUEUE_C28X_H
#define SPSC_QUEUE_C28X_H

#include <stdint.h>
#include <stdbool.h>

//
// ===============================
//  Single-Producer / Single-Consumer
//  Lock-Free Queue for C2000 (F28379D)
// ===============================
//
// Design assumptions:
//
// 1. Single producer: an ISR only
//    - Only the ISR calls spsc_enqueue()
//    - Only the ISR writes 'head'
//
// 2. Single consumer: main loop only
//    - Only the main loop calls spsc_dequeue()
//    - Only the main loop writes 'tail'
//
// 3. C28x architecture properties (F28379D):
//    - Single core, no cache on data RAM.
//    - Strongly ordered memory model: loads/stores are executed
//      in program order; CPU does not reorder memory accesses.
//    - No speculative execution that would reorder memory ops.
//
//  Given this, once the ISR writes buffer[head], the write is
//  visible to the main loop before the ISR updates 'head'. 
//  This means we do not need memory barriers.
//
// 4. We use a circular ring buffer with head/tail indices.
//    - Empty when head == tail
//    - Full when advancing head by 1 would make head == tail
//
// 5. We declare head and tail as 'volatile':
//    - Prevents compiler from caching them in registers.
//    - Ensures ISR and main loop always see latest values.
//    - Prevents the compiler from reordering accesses around
//      volatile reads/writes in ways that would break semantics.
//


// -------------------------------
// Queue configuration
// -------------------------------

//
// Queue size must be a power of two.
// - This allows us to use a cheap bitmask instead of modulo.
// - Example: if size = 64, we can wrap index with (idx & 63).
//   This is much cheaper than integer division on C28x.
//
// Choose a size that can tolerate worst-case ISR production
// bursts before the main loop can catch up.
//
#define SPSC_QUEUE_SIZE 64u   // must be a power of 2 (e.g. 32, 64, 128, ...)


// -------------------------------
// Queue data structure
// -------------------------------

typedef struct {
    //
    // Fixed-size buffer holding the data.
    // Each slot holds one int16_t value.
    //
    int16_t buffer[SPSC_QUEUE_SIZE];

    //
    // 'head' is the producer write index:
    // - Only the ISR modifies 'head' (via spsc_enqueue).
    // - The main loop only reads 'head'.
    //
    // 'volatile' is critical:
    // - Prevents compiler from optimizing away reloads.
    // - Ensures ISR and main loop see each other's updates.
    //
    volatile uint16_t head;

    //
    // 'tail' is the consumer read index:
    // - Only the main loop modifies 'tail' (via spsc_dequeue).
    // - The ISR only reads 'tail'.
    //
    volatile uint16_t tail;

} spsc_queue_t;


// -------------------------------
// Initialization
// -------------------------------

//
// Initialize the queue to empty state.
//
// Safety notes:
// - Setting head = tail = 0 establishes the invariant
//   "empty queue" at startup.
// - There must be no concurrent use of the queue before this
//   function is called (i.e., do this before enabling interrupts).
//
static inline void spsc_init(spsc_queue_t *q)
{
    q->head = 0u;
    q->tail = 0u;
}


// -------------------------------
// State queries
// -------------------------------

//
// Returns true if the queue has no data.
//
// Safety notes:
// - Both head and tail are volatile, so this function sees
//   the latest values written by ISR/main loop.
// - It's safe to call from either context.
//   Typically you'll call this from the main loop.
//
static inline bool spsc_is_empty(const spsc_queue_t *q)
{
    return (q->head == q->tail);
}

//
// Returns true if the queue cannot accept more data.
//
// Safety notes:
// - We define the queue as "full" when advancing the head index
//   by one would cause head == tail.  That is, we always leave
//   one slot unused in the ring to distinguish full from empty.
//
//   Example with size = 8:
//     head == tail      → empty
//     (head + 1) & 7 == tail → full
//
// - This check is safe because:
//   * ISR is the only writer of head
//   * Main loop is the only writer of tail
//   * Both sides see each other’s updates via volatile.
//
static inline bool spsc_is_full(const spsc_queue_t *q)
{
    uint16_t next = (uint16_t)((q->head + 1u) & (SPSC_QUEUE_SIZE - 1u));
    return (next == q->tail);
}


// -------------------------------
// Enqueue (ISR / producer)
// -------------------------------

//
// Attempt to enqueue a value from the ISR.
//
// Returns:
//   - true  if the value was successfully enqueued
//   - false if the queue is full (value is dropped)
//
// Safety notes:
// - This function is intended to be called only from the ISR
//   (single-producer).
// - It never blocks, never spins, and never disables interrupts.
//   Worst case: it returns false and the value is dropped or
//   handled by the caller in some other way.
//
// Concurrency reasoning:
//
// - The ISR is the only writer of 'head', and main loop never
//   writes it. So there is no write-write race on head.
// - The main loop is the only writer of 'tail', and ISR never
//   writes it. So there is no write-write race on tail.
// - ISR may read 'tail' while main loop is writing it, and main
//   loop may read 'head' while ISR is writing it. These are
//   read-write races, but they are safe on C28x because:
//     * The volatile qualifier ensures accesses are not reordered.
//     * The operations are naturally atomic for 16-bit types on
//       this 16-bit architecture.
// - C28x performs memory accesses in program order. Therefore:
//
//   In the ISR:
//     q->buffer[head] = value;   // data write
//     q->head = next;            // publish new head
//
//   The main loop cannot observe the new head value before the
//   buffer write is visible, because the CPU does not reorder
//   these stores. Thus, when the consumer sees an updated head,
//   the corresponding buffer slot is already properly written.
//
static inline bool spsc_enqueue(spsc_queue_t *q, int16_t value)
{
    // Snapshot the current head index.
    uint16_t head = q->head;

    // Compute next index in the ring, wrapping with a mask.
    // Because SPSC_QUEUE_SIZE is a power of 2, (size - 1) is
    // an all-ones mask for the low bits, equivalent to modulo.
    uint16_t next = (uint16_t)((head + 1u) & (SPSC_QUEUE_SIZE - 1u));

    // If advancing head would collide with tail, the queue is full.
    // We choose to *not overwrite* existing data, so we fail here.
    if (next == q->tail) {
        // Queue full: caller may choose to drop or count overruns.
        return false;
    }

    // Store the data into the current head slot.
    // On C28x, this store is guaranteed to complete before the
    // subsequent store to 'head' (because of strong ordering).
    q->buffer[head] = value;

    // Publish the new head index:
    // - This makes the value visible to the consumer.
    // - As soon as 'head' != 'tail', consumer can safely read.
    //
    // No memory barrier is required here on C28x.
    q->head = next;

    return true;
}


// -------------------------------
// Dequeue (main loop / consumer)
// -------------------------------

//
// Attempt to dequeue one value from the queue.
//
// Parameters:
//   - q   : pointer to queue
//   - out : pointer to destination variable
//
// Returns:
//   - true  if a value was dequeued and stored into *out
//   - false if the queue was empty
//
// Safety notes:
// - This function is intended to be called only from the main loop
//   (single-consumer).
// - It never blocks and does not disable interrupts.
// - The logic mirrors the enqueue function's reasoning:
//
//   In the main loop:
//     value = q->buffer[tail];   // data read
//     q->tail = new_tail;        // consume slot
//
//   Because C28x does not reorder, the store to tail happens after
//   the read from buffer[tail]. Therefore, the ISR will never see
//   a tail value that "claims" a slot has been freed before the
//   consumer actually read from it.
//
static inline bool spsc_dequeue(spsc_queue_t *q, int16_t *out)
{
    // Snapshot the current tail index.
    uint16_t tail = q->tail;

    // If tail == head, queue is empty: nothing to read.
    if (tail == q->head) {
        return false;   // empty
    }

    // Read the value from the current tail slot.
    // This is safe because:
    // - Producer will only write to this slot before it advances head.
    // - Consumer only reads from a slot when tail != head.
    //   That implies producer has already written to this slot.
    *out = q->buffer[tail];

    // Advance the tail index, wrapping with mask.
    // This "consumes" the slot and makes it available to the producer.
    q->tail = (uint16_t)((tail + 1u) & (SPSC_QUEUE_SIZE - 1u));

    return true;
}

//
// Clear all items from the queue.
//
// SAFETY NOTES:
//
// • We do NOT touch q->head because the ISR owns it.
//   Touching head would create a race where the ISR could
//   advance from an unexpected position.
//
// • Instead, we simply set tail = head.
//   This discards all currently unread items.
//
// • This is safe because tail is ONLY written by the consumer,
//   and reading head is always safe.
//
// • Queue invariants are preserved:
//     empty := (head == tail)
//
// • If the ISR fires while this executes:
//     - ISR may increment head
//     - That is fine: queue will reflect newly added items.
//     - No slots are corrupted, because tail never jumps ahead
//       of the real head.
//
static inline void spsc_clear(spsc_queue_t *q)
{
    q->tail = q->head;
}

#endif // SPSC_QUEUE_C28X_H

