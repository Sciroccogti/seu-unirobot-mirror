#ifndef SEU_UNIROBOT_SINGLETON_HPP
#define SEU_UNIROBOT_SINGLETON_HPP

#include <cassert>

#ifdef DEBUG
#define ASSERT assert
#else
#define ASSERT(x);
#endif

template <typename T> 
class singleton
{
    static T *singleton_;

public:
    ~singleton()
    {
        ASSERT( singleton_ );
        singleton_ = 0;
    }
    static T &get_singleton()
    {
        static T the_T;
        ASSERT( singleton_ );
        return ( *singleton_ );
    }
protected:
    singleton()
    {
        assert( !singleton_ );
        singleton_ = static_cast<T *>(this);
    }
};

template <typename T> 
T *singleton <T>::singleton_ = 0;

#ifdef NO_ASSERT
#undef NO_ASSERT
#endif
#undef ASSERT
#endif
