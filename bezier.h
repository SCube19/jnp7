#ifndef BEZIER_H
#define BEZIER_H
#include <iostream>
#include <functional>

namespace bezier
{
    namespace constants
    {
        constexpr int NUM_OF_CUBIC_BEZIER_NODES = 4;
    }

    namespace types
    {
        using real_t = double;
        using node_index_t = size_t;

        class point_2d
        {
        public:
            real_t X;
            real_t Y;

            point_2d() = delete;
            point_2d(real_t xp, real_t yp) : X(xp), Y(yp){};

            point_2d operator+(const point_2d &p) const
            {
                return point_2d(this->X + p.X, this->Y + p.Y);
            }

            point_2d operator*(real_t scalar) const
            {
                return point_2d(this->X * scalar, this->Y * scalar);
            }

            friend point_2d operator*(real_t scalar, const point_2d &p);

            bool operator==(const point_2d &p) const
            {
                return (this->X == p.X) && (this->Y == p.Y);
            }

            friend std::ostream &operator<<(std::ostream &os, const point_2d &p);
        };
        point_2d operator*(real_t scalar, const point_2d &p)
        {
            return point_2d(p.X * scalar, p.Y * scalar);
        }

        std::ostream &operator<<(std::ostream &os, const point_2d &p)
        {
            os << '(' << p.X << ", " << p.Y << ')';
            return os;
        }

        using PointFunction = std::function<types::point_2d(types::node_index_t)>;
    } // namespace types

    types::PointFunction Cup()
    {
    }

    types::PointFunction Cap()
    {
    }

    types::PointFunction ConvexArc()
    {
    }

    types::PointFunction ConcaveArc()
    {
    }

    types::PointFunction LineSegment(const types::point_2d &p, const types::point_2d &q)
    {
    }

    template <typename numeric>
    types::PointFunction MovePoint(const types::PointFunction &f, types::node_index_t i, numeric x, numeric y)
    {
    }

    template <typename degrees>
    types::PointFunction Rotate(const types::PointFunction &f, degrees a)
    {
    }

    template <typename numeric>
    types::PointFunction Scale(const types::PointFunction &f, numeric x, numeric y)
    {
    }

    template <typename numeric>
    types::PointFunction Translate(const types::PointFunction &f, numeric x, numeric y)
    {
    }

    template <typename Func>
    Func Concatenate(Func f1, Func f2)
    {
    }

    template <typename Func, typename... Rest>
    Func Concatenate(Func f1, Func f2, Rest &&...funcs)
    {
        return Concatenate(Concatenate(f1, f2), funcs...);
    }

    class P3CurvePlotter
    {
    public:
        P3CurvePlotter(bezier::types::PointFunction f, int segments = 1, size_t resolution = 80)
        {
        }

        void Print(const std::ostream &s = std::cout, char fb = '*', char bg = ' ') const
        {
        }

        const bezier::types::point_2d operator()(bezier::types::PointFunction f, double t) const
        {
        }
    };

} // namespace bezier
#endif // !BEZIER_H
