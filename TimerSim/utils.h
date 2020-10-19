#pragma once

#include <cmath>


namespace utils {

    template<class T = double>
    struct BasePoint {
        T x_ = 0, y_ = 0, z_ = 0;

        BasePoint<T>() = default;
        BasePoint<T>(T x, T y, T z) : x_(x), y_(y), z_(z) { }

        template<class U>
        BasePoint<T>(const BasePoint<U>& rhs) : x_(static_cast<T>(rhs.x_)), y_(static_cast<T>(rhs.y_)), z_(static_cast<T>(rhs.z_)) { }


        /*				OPERATORS				*/
        template<class U>
        BasePoint<T>& operator= (const BasePoint<U>& rhs) {
            if (&rhs == this) {
                return *this;
            }
            x_ = static_cast<T>(rhs.x_);
            y_ = static_cast<T>(rhs.y_);
            z_ = static_cast<T>(rhs.z_);
            return *this;
        }

        BasePoint<T>& operator+=(const BasePoint<T>& rhs) {
            if (&rhs == this) {
                x_ *= 2;
                y_ *= 2;
                z_ *= 2;
            } else {
                x_ += rhs.x_;
                y_ += rhs.y_;
                z_ += rhs.z_;
            }
            return *this;
        }

        BasePoint<T>& operator-=(const BasePoint<T>& rhs) {
            if (&rhs == this) {
                x_ = 0;
                y_ = 0;
                z_ = 0;
            } else {
                x_ -= rhs.x_;
                y_ -= rhs.y_;
                z_ -= rhs.z_;
            }
            return *this;
        }

        BasePoint<T> operator+(const BasePoint<T>& rhs) const {
            return (BasePoint<T>(*this) += rhs);
        }

        BasePoint<T> operator-(const BasePoint<T>& rhs) const {
            return (BasePoint<T>(*this) -= rhs);
        }
        /*
        bool operator<(const BasePoint<T>& rhs) const {
            if (this->x_ != rhs.x_)
                return this->x_ < rhs.x_;
            else
                return this->y_ < rhs.y_;
        }*/

        bool operator==(const BasePoint<T>& rhs) const {
            return (this->x_ == rhs.x_ && this->y_ == rhs.y_ && this->z_ == rhs.z_);
        }

        template <class U>
        operator BasePoint<U>() {
            BasePoint<U> u;
            u.x_ = static_cast<U>(x_);
            u.y_ = static_cast<U>(y_);
            u.z_ = static_cast<U>(z_);
            return u;
        }

        /*double dirTo(const BasePoint<T>& other) const {
            return atan2(other.y_ - y_, other.x_ - x_);
        }*/
        double dist(const BasePoint<T>& other) const {
            return sqrt(pow(x_ - other.x_, 2) + pow(y_ - other.y_, 2), pow(z_ - other.z_, 2));
        }

        void toAbs() {
            x_ = std::abs(x_);
            y_ = std::abs(y_);
            z_ = std::abs(z_);
        }

        BasePoint<T> abs() const {
            BasePoint<T> p(*this);
            p.toAbs();
            return p;
        }

    };
    using Point = BasePoint<double>;



    template<class T>
    constexpr T pow2(T a) {
        return a * a;
    }

    template <class L, class R> static inline constexpr auto min_(const L lhs, const R rhs) -> decltype(lhs + rhs) {
        return lhs < rhs ? lhs : rhs;
    }
    template <class L, class R> static inline constexpr auto max_(const L lhs, const R rhs) -> decltype(lhs + rhs) {
        return lhs > rhs ? lhs : rhs;
    }

    template<class T, class ... Ts> static inline constexpr const T max_(T V, Ts... Vs) {
        return max_(V, max_(Vs...));
    }

    template<class T, class ... Ts> static inline constexpr const T min_(T V, Ts... Vs) {
        return min_(V, min_(Vs...));
    }

};

