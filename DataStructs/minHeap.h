#ifndef SOLVINGTSP_MINHEAP_H
#define SOLVINGTSP_MINHEAP_H

#include <vector>
#include <unordered_map>

#define LEFT(i) (2 * (i))
#define RIGHT(i) (2 * (i) + 1)
#define PARENT(i) ((i) / 2)

// Binary min-heap to represent integer keys of type K with values (priorities) of type V
template <class Key, class Value>
class MinHeap {
    struct Node { // An element of the heap: a pair (key, value)
        Key key;
        Value value;
    };

    int heapSize;                  // Number of elements in the heap
    int maxHeapSize;               // Maximum number of elements in heap
    std::vector<Node> heapArray;  // The heap array
    std::unordered_map<Key, int> positions; // Maps a key into its position in the heapArray
    const Key KEY_NOT_FOUND;

    /**
     * Moves a value up the tree until it reaches its appropriate position in the heap.
     * <b>Complexity\n</b>
     * <pre>
     *      <b>O(log N)</b>, where N is the number of elements in the heap.
     * </pre>
     * @param index The index of the element to bubble up.
     * @note This function moves a value up the tree until it reaches its correct position in the heap.
     *       It maintains the min-heap property by swapping elements as necessary.
     */
    void bubbleUp(int index); // Time Complexity: O(log N)

    /**
     * Moves a value down the tree until it reaches its appropriate position in the heap.
     * <pre>
     *      <b>O(log N)</b> O(log N) in general, O(1) in Fibonacci heap case.(decrease key also)
     *                  N represents the number of elements in the heap.
     * </pre>
     * @param index The index of the element to bubble down.
     * @note This function moves a value down the tree until it reaches its correct position in the heap.
     *       It maintains the min-heap property by swapping elements as necessary.
     */
    void bubbleDown(int index); // Time Complexity: O(log N) O(1) in fibTree case

    /**
     * Swaps two nodes within the heap and updates their positions.
     *
     * @param index1 Index of the first node to be swapped.
     * @param index2 Index of the second node to be swapped.
     * @note This function swaps two nodes within the heap and updates their positions in the heapArray.
     */
    void swapNodes(int index1, int index2);

public:
    /**
     * @brief Constructor to create a MinHeap instance.
     *
     * @param n Maximum number of pairs (Key, Value) allowed in the heap.
     * @param notFound Value returned when the heap is empty.
     * @note Constructs a min-heap for a maximum of n pairs (Key, Value) with 'notFound' returned when empty.
     */
    MinHeap(int n, const Key& notFound);

    int getSize();              // Return number of elements in the heap

    /**
     * @brief Checks if the heap contains a specific key.
     *
     * @param key The key to check for existence in the heap.
     * @return True if the key exists in the heap, otherwise false.
     * @note Verifies if a specified key exists within the heap.
     */
    bool contains(const Key& key);  // Checks if heap contains a key

    /**
     * @brief Inserts a new key-value pair into the min-heap.
     *
     * @param key The key to be inserted.
     * @param value The value associated with the key to be inserted.
     * @note Inserts a new key-value pair into the min-heap.
     */
    void insert(const Key& key, const Value& value);      // Insert (key, value) into the heap. Time Complexity: O(log N)

    /**
     * @brief Decreases the value associated with a specific key in the heap.
     *
     * @param key The key whose value needs to be decreased.
     * @param value The new value to be set for the key.
     * @note Decreases the value associated with a key in the heap if the new value is smaller.
     */
    void decreaseKey(const Key& key, const Value& value); // Decrease value of key. Time Complexity: O(log N)

    /**
     * @brief Removes and returns the key with the smallest value in the heap.
     *
     * @return The key with the smallest value.
     * @note Removes and returns the key with the smallest value in the heap.
     */
    Key extractMin(); // Remove and return key with the smallest value. Time Complexity: O(log N)
    /**
     * @brief Checks if the Fibonacci heap is empty.
     * @return True if the heap is empty, otherwise false.
     */
    bool empty() const;
};

template<class Key, class Value>
bool MinHeap<Key, Value>::empty() const {
    return heapSize == 0;
}

// ----------------------------------------------

// Move a value up the tree until it reaches its position
template <class Key, class Value>
void MinHeap<Key, Value>::bubbleUp(int index) {
    while (index > 1 && heapArray[index].value < heapArray[PARENT(index)].value) {
        swapNodes(index, PARENT(index));
        index = PARENT(index);
    }
}

// Move a value down the tree until it reaches its position
template <class Key, class Value>
void MinHeap<Key, Value>::bubbleDown(int index) {
    while (LEFT(index) <= heapSize) {
        int childIndex = LEFT(index);
        if (RIGHT(index) <= heapSize && heapArray[RIGHT(index)].value < heapArray[childIndex].value) {
            childIndex = RIGHT(index);
        }
        if (heapArray[index].value < heapArray[childIndex].value) {
            break;
        }
        swapNodes(index, childIndex);
        index = childIndex;
    }
}

// Swap two nodes within the heap and update their positions
template <class Key, class Value>
void MinHeap<Key, Value>::swapNodes(int index1, int index2) {
    Node tmp = heapArray[index1];
    heapArray[index1] = heapArray[index2];
    heapArray[index2] = tmp;
    positions[heapArray[index1].key] = index1;
    positions[heapArray[index2].key] = index2;
}

// ----------------------------------------------

// Create a min-heap for a maximum of n pairs (Key, Value) with notFound returned when empty
template <class Key, class Value>
MinHeap<Key, Value>::MinHeap(int n, const Key& notFound) : KEY_NOT_FOUND(notFound), heapSize(0), maxHeapSize(n), heapArray(n + 1) {
}

// Return number of elements in the heap
template <class Key, class Value>
int MinHeap<Key, Value>::getSize() {
    return heapSize;
}

// Checks if heap contains a key
template <class Key, class Value>
bool MinHeap<Key, Value>::contains(const Key& key) {
    return positions.find(key) != positions.end();
}

// Insert (key, value) into the heap
template <class Key, class Value>
void MinHeap<Key, Value>::insert(const Key& key, const Value& value) {
    if (heapSize == maxHeapSize) return; // The heap is full, do nothing
    if (contains(key)) return;           // The key already exists, do nothing
    heapArray[++heapSize] = {key, value};
    positions[key] = heapSize;
    bubbleUp(heapSize);
}

// Decrease value of key to the indicated value
template <class Key, class Value>
void MinHeap<Key, Value>::decreaseKey(const Key& key, const Value& value) {
    if (!contains(key)) return; // The key does not exist, do nothing
    int index = positions[key];
    if (value > heapArray[index].value) return; // The value would increase, do nothing
    heapArray[index].value = value;
    bubbleUp(index);
}

// Remove and return key with the smallest value
template <class Key, class Value>
Key MinHeap<Key, Value>::extractMin() {
    if (heapSize == 0) return KEY_NOT_FOUND;
    Key minKey = heapArray[1].key;
    positions.erase(minKey);
    heapArray[1] = heapArray[heapSize--];
    bubbleDown(1);
    return minKey;
}


#endif //SOLVINGTSP_MINHEAP_H
