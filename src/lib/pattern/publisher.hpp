#ifndef SUBJECT_HPP
#define SUBJECT_HPP

#include <memory>

class subscriber;
class publisher;

typedef std::shared_ptr<subscriber> sub_ptr;
typedef std::shared_ptr<publisher> pub_ptr;

class publisher: public std::enable_shared_from_this<publisher>
{
public:
    virtual void attach(sub_ptr) = 0;
    virtual void detach(sub_ptr) = 0;
protected:
    std::vector<sub_ptr> subs_;
    void notify()
    {
        for(auto sub:subs_)
            sub->update(shared_from_this());
    }
};

#endif