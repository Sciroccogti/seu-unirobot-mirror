#ifndef SEU_UNIROBOT_PATTERN_HPP
#define SEU_UNIROBOT_PATTERN_HPP

#include <memory>
#include <vector>

class subscriber;
class publisher;

typedef std::shared_ptr<subscriber> sub_ptr;
typedef std::shared_ptr<publisher> pub_ptr;

class subscriber: public std::enable_shared_from_this<subscriber>
{
public:
    virtual void updata(const pub_ptr &pub)=0;
};

class publisher: public std::enable_shared_from_this<publisher>
{
public:
    virtual void attach(const sub_ptr &s)
    {
        subs_.push_back(s);
    }
    
    virtual void detach(const sub_ptr &s)
    {
        auto iter = subs_.begin();
        while(iter!=subs_.end())
        {
            if((*iter) == s)
            {
                subs_.erase(iter);
                break;
            }
            iter++;
        }
    }
    
    virtual void notify()
    {
        for(int i=0;i<subs_.size();i++)
            subs_[i]->updata(shared_from_this());
    }
protected:
    std::vector<sub_ptr> subs_;
};

#endif