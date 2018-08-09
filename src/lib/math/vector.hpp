#ifndef SEU_UNIROBOT_VECTOR_HPP
#define SEU_UNIROBOT_VECTOR_HPP

#include <eigen3/Eigen/Dense>

namespace robot_math
{
    template<typename T, int size>
    class vector_t: public Eigen::Matrix<T, size, 1>
    {
    public:
        T x() const { return this->data()[0]; }

        T &x() { return this->data()[0]; }

        T y() const { return this->data()[1]; }

        T &y() { return this->data()[1]; }

        T z() const { return this->data()[2]; }

        T &z() { return this->data()[2]; }

        vector_t &operator=(const Eigen::Matrix<T, size, 1> &m)
        {
            for(int i=0;i<size;i++)
                this->data()[i] = m(i,0);
            return *this;
        }

        double length()
        {
            double sum = 0.0;
            for(int i=0; i<this->rows();i++)
                sum += pow(this->data()[i], 2);
            return sqrt(sum);
        }
    };
}
#endif