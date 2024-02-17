template <class T> class RingBuffer{
  public:
    RingBuffer(const size_t &size);
    void addValue(const T &value);
    T getAverage(void) const;
    T getMax(void) const;
  private:
    T* myArray;
    size_t arraySize;
    uint32_t index;
    size_t numElements;
};

template <class T> RingBuffer <T>::RingBuffer(const size_t &size)
{
  arraySize = size;
  myArray = new T[size];
}

template <class T> void RingBuffer <T>::addValue(const T &value)
{

  if(++numElements > arraySize) numElements = arraySize;
  myArray[index++] = value;
  index %= arraySize;
}

template <class T> T RingBuffer <T>::getAverage(void) const
{
  T sum = 0;
  for(int i = 0; i < numElements; i++)
  {
    sum += myArray[i];
  }
  return (sum / numElements);
}

template <class T> T RingBuffer <T>::getMax(void) const
{
  T sum = 0;
  for(int i = 0; i < numElements; i++)
  {
    sum += myArray[i];
  }
  return (sum / numElements);
}
