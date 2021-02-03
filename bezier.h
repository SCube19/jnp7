#ifndef BEZIER_H
#define BEZIER_H
#include <iostream>
#include <functional>
#include <memory>
#include <math.h>
#include <type_traits>
#include <vector>
#include <numeric>

#include <unistd.h>

namespace bezier
{

    namespace constants
    {
        constexpr int NUM_OF_CUBIC_BEZIER_NODES = 4;
        constexpr double ARC = 4 * (sqrt(2) - 1) / 3;
    } // namespace constants

    namespace types
    {
        using real_t = double;
        using node_index_t = size_t;

        //////////////////POINT 2D/////////////////////////////
        class point_2d
        {
        public:
            real_t X;
            real_t Y;

            point_2d() = delete;
            point_2d(real_t xp, real_t yp) : X(xp), Y(yp){};

            const point_2d operator+(const point_2d &p) const
            {
                return point_2d(this->X + p.X, this->Y + p.Y);
            }

            const point_2d operator*(real_t scalar) const
            {
                return point_2d(this->X * scalar, this->Y * scalar);
            }

            friend const point_2d operator*(real_t scalar, const point_2d &p);

            bool operator==(const point_2d &p) const
            {
                return (this->X == p.X) && (this->Y == p.Y);
            }

            friend std::ostream &operator<<(std::ostream &os, const point_2d &p);
        };

        const point_2d operator*(real_t scalar, const point_2d &p)
        {
            return point_2d(p.X * scalar, p.Y * scalar);
        }

        std::ostream &operator<<(std::ostream &os, const point_2d &p)
        {
            os << '(' << p.X << ", " << p.Y << ')';
            return os;
        }
        ///////////////////////////////////////////////////////////////////////////////////////////

        using PointFunction = std::function<const types::point_2d(types::node_index_t)>;

    } // namespace types

    /////////////////////////////////HELPER FUNCS///////////////////////////////////////////////////
    const types::point_2d get(const std::vector<types::point_2d> &bezierVec, types::node_index_t i)
    {
        return i < bezierVec.size() ? bezierVec[i] : throw std::out_of_range("a curve node index is out of range");
    }

    size_t domainSize(const types::PointFunction &f, size_t i = 0)
    {
        try
        {
            f(i);
            return 1 + domainSize(f, i + 1);
        }
        catch (const std::out_of_range &e)
        {
            return 0;
        }
    }

    const std::vector<types::point_2d> decodeFunction(const types::PointFunction &f)
    {
        const size_t size = domainSize(f);
        std::vector<types::node_index_t> args(size);
        std::iota(args.begin(), args.end(), 0);

        std::vector<types::point_2d> rVec{};
        std::transform(args.cbegin(), args.cend(), std::back_inserter(rVec), [f](types::node_index_t i) -> types::point_2d { return f(i); });

        return rVec;
    }

    template <typename Bind>
    const std::vector<types::point_2d> applyTransformation(const Bind &toApply, const types::PointFunction &f)
    {
        std::vector<types::point_2d> rVector{};
        const std::vector<types::point_2d> decoded = decodeFunction(f);
        std::transform(decoded.cbegin(), decoded.cend(), std::back_inserter(rVector), toApply);
        return rVector;
    }

    //////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CUP////////////////////////////////////////////////////
    types::PointFunction Cup()
    {
        return types::PointFunction(
            [](types::node_index_t i) {
                const std::vector<types::point_2d> bezierVec{
                    types::point_2d(-1, 1),
                    types::point_2d(-1, -1),
                    types::point_2d(1, -1),
                    types::point_2d(1, 1)};

                return get(bezierVec, i);
            });
    }

    ////////////////////////////////////////CAP////////////////////////////////////////////////////
    //użycie rotate(Cup()) wymagałoby odbicia lustrzanego punktów - brak uproszczenia kodu
    types::PointFunction Cap()
    {
        return types::PointFunction(
            [](types::node_index_t i) {
                const std::vector<types::point_2d> bezierVec{
                    types::point_2d(-1, -1),
                    types::point_2d(-1, 1),
                    types::point_2d(1, 1),
                    types::point_2d(1, -1)};

                return get(bezierVec, i);
            });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CONVEX ARC////////////////////////////////////////////////////
    types::PointFunction ConvexArc()
    {
        return types::PointFunction(
            [](types::node_index_t i) {
                const std::vector<types::point_2d> bezierVec{
                    types::point_2d(0, 1),
                    types::point_2d(constants::ARC, 1),
                    types::point_2d(1, constants::ARC),
                    types::point_2d(1, 0)};

                return get(bezierVec, i);
            });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CONCAVE ARC////////////////////////////////////////////////////
    template <typename numeric>
    types::PointFunction MovePoint(const types::PointFunction &f, types::node_index_t i, numeric x, numeric y);

    types::PointFunction ConcaveArc()
    {
        return MovePoint(MovePoint(ConvexArc(), 1, -constants::ARC, -constants::ARC), 2, -constants::ARC, -constants::ARC);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////LINE SEGMENT////////////////////////////////////////////////////
    types::PointFunction LineSegment(const types::point_2d &p, const types::point_2d &q)
    {
        return types::PointFunction(
            [p, q](types::node_index_t i) {
                const std::vector<types::point_2d> bezierVec{
                    types::point_2d(p.X, p.Y),
                    types::point_2d(p.X, p.Y),
                    types::point_2d(q.X, q.Y),
                    types::point_2d(q.X, q.Y)};

                return get(bezierVec, i);
            });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////MOVE POINT////////////////////////////////////////////////////

    template <typename numeric>
    const std::vector<types::point_2d> movePt(const types::PointFunction &f, types::node_index_t i, numeric x, numeric y)
    {
        std::vector<types::point_2d> rVector = decodeFunction(f);
        rVector[i] = rVector[i] + types::point_2d(x, y);
        return rVector;
    }

    template <typename numeric>
    types::PointFunction MovePoint(const types::PointFunction &f, types::node_index_t i, numeric x, numeric y)
    {
        static_assert(std::is_arithmetic<numeric>());
        return f == nullptr ? f : types::PointFunction([f, i, x, y](types::node_index_t q) {
            const std::vector<types::point_2d> bezierVec = movePt<numeric>(f, i, x, y);

            return get(bezierVec, q);
        });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////ROTATE////////////////////////////////////////////////////
    template <typename degrees>
    const double radians(degrees deg)
    {
        return ((double)deg * (M_PI / 180.0f));
    }

    template <typename degrees>
    const types::point_2d rotPoint(const types::point_2d &p, degrees deg)
    {
        return types::point_2d(p.X * cos(radians(deg)) - p.Y * sin(radians(deg)), p.X * sin(radians(deg)) + p.Y * cos(radians(deg)));
    }

    template <typename degrees>
    types::PointFunction Rotate(const types::PointFunction &f, degrees a)
    {
        static_assert(std::is_arithmetic<degrees>());
        return f == nullptr ? f : types::PointFunction([f, a](types::node_index_t i) {
            const std::vector<types::point_2d> bezierVec = applyTransformation(std::bind(rotPoint<degrees>, std::placeholders::_1, a), f);

            return get(bezierVec, i);
        });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////SCALE////////////////////////////////////////////////////
    template <typename numeric>
    const types::point_2d scalePoint(const types::point_2d &p, numeric x, numeric y)
    {
        return types::point_2d(p.X * x, p.Y * y);
    }

    template <typename numeric>
    types::PointFunction Scale(const types::PointFunction &f, numeric x, numeric y)
    {
        static_assert(std::is_arithmetic<numeric>());
        return f == nullptr ? f : types::PointFunction([f, x, y](types::node_index_t i) {
            const std::vector<types::point_2d> bezierVec = applyTransformation(std::bind(scalePoint<numeric>, std::placeholders::_1, x, y), f);

            return get(bezierVec, i);
        });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////TRANSLATE////////////////////////////////////////////////////
    template <typename numeric>
    const types::point_2d translatePoint(const types::point_2d &p, numeric x, numeric y)
    {
        return p + types::point_2d(x, y);
    }

    template <typename numeric>
    types::PointFunction Translate(const types::PointFunction &f, numeric x, numeric y)
    {
        static_assert(std::is_arithmetic<numeric>());
        return f == nullptr ? f : types::PointFunction([f, x, y](types::node_index_t i) {
            const std::vector<types::point_2d> bezierVec = applyTransformation(std::bind(translatePoint<numeric>, std::placeholders::_1, x, y), f);

            return get(bezierVec, i);
        });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CONCATENATE////////////////////////////////////////////////////
    const std::vector<types::point_2d> vecConcat(const std::vector<types::point_2d> &v1, const std::vector<types::point_2d> &v2)
    {
        std::vector<types::point_2d> v12(v1);
        v12.insert(v12.end(), v2.begin(), v2.end());
        return v12;
    }

    template <typename Func>
    Func Concatenate(const Func &f1, const Func &f2)
    {
        return (f1 == nullptr && f2 == nullptr) ? f1 : (f1 == nullptr ? f2 : (f2 == nullptr ? f1 : types::PointFunction([f1, f2](types::node_index_t i) {
            const std::vector<types::point_2d> bezierVec = vecConcat(decodeFunction(f1), decodeFunction(f2));

            return get(bezierVec, i);
        })));
    }

    template <typename Func, typename... Rest>
    Func Concatenate(Func f1, Func f2, Rest &&...funcs)
    {
        return Concatenate(Concatenate(f1, f2), funcs...);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////P3 CURVE PLOTTER////////////////////////////////////////////////////
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace bezier
#endif // !BEZIER_H
