#ifndef __OBSERVER_HPP
#define __OBSERVER_HPP

#include <memory>
#include <vector>

class Subscriber;
class Publisher;

typedef std::shared_ptr<Subscriber> sub_ptr;
typedef std::shared_ptr<Publisher> pub_ptr;

class Subscriber: public std::enable_shared_from_this<Subscriber>
{
public:
    virtual void updata(const pub_ptr &pub, const int &type) = 0;
};

class Publisher: public std::enable_shared_from_this<Publisher>
{
public:
    virtual void attach(const sub_ptr &s)
    {
        subs_.push_back(s);
    }

    virtual void detach(const sub_ptr &s)
    {
        auto iter = subs_.begin();

        while (iter != subs_.end())
        {
            if ((*iter) == s)
            {
                subs_.erase(iter);
                break;
            }

            iter++;
        }
    }

    virtual void notify(const int &type)
    {
        for (size_t i = 0; i < subs_.size(); i++)
        {
            subs_[i]->updata(shared_from_this(), type);
        }
    }
protected:
    std::vector<sub_ptr> subs_;
};

#endif
