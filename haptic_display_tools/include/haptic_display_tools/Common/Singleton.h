#ifndef SINGLETON_H
#define SINGLETON_H

// warning: the derived class is responsible for declaring its own m_instance
//          variable in the program!

template <class T>
class Singleton
{
    static T *m_instance;

protected:
    Singleton()                             { }
    Singleton(const Singleton &)            { }
    Singleton &operator=(const Singleton &) { }

public:
    static T *instance()
    {
        if (m_instance == 0)
            m_instance = new T();
        return m_instance;
    }
};

#endif // SINGLETON_H
