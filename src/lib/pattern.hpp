#pragma once

#include <memory>
#include <vector>

class subscriber;
class promulgator;

typedef std::shared_ptr<subscriber> sub_ptr;
typedef std::shared_ptr<promulgator> pro_ptr;

class subscriber: public std::enable_shared_from_this<subscriber>
{
public:
    virtual void updata(const pro_ptr &pub, const int &type) = 0;
};

class promulgator: public std::enable_shared_from_this<promulgator>
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
