#ifndef BEZIER_H
#define BEZIER_H
#include <iostream>
#include <functional>
#include <memory>
#include <math.h>
#include <type_traits>
#include <vector>
#include <numeric>
#include <set>

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
    size_t domainSize(const types::PointFunction &f, size_t i = 0)
    {
        try
        {
            f(i);
            return 1 + domainSize(f, i + constants::NUM_OF_CUBIC_BEZIER_NODES);
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

        std::vector<types::point_2d> rVec;
        // for(auto x: args)
        //     std::cout << x  << " ";
        // std::cout << std::endl;

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

                return i < bezierVec.size() ? bezierVec[i] : throw std::out_of_range("a curve node index is out of range");
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

                return i < bezierVec.size() ? bezierVec[i] : throw std::out_of_range("a curve node index is out of range");
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

                return i < bezierVec.size() ? bezierVec[i] : throw std::out_of_range("a curve node index is out of range");
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

                return i < bezierVec.size() ? bezierVec[i] : throw std::out_of_range("a curve node index is out of range");
            });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////MOVE POINT////////////////////////////////////////////////////

    template <typename numeric>
    const types::point_2d movePt(const types::PointFunction &f, types::node_index_t q, types::node_index_t i, numeric x, numeric y)
    {
        return q == i ? f(q) + types::point_2d(x, y) : f(q);
    }

    template <typename numeric>
    types::PointFunction MovePoint(const types::PointFunction &f, types::node_index_t i, numeric x, numeric y)
    {
        static_assert(std::is_arithmetic<numeric>());
        return f == nullptr ? f : types::PointFunction([f, i, x, y](types::node_index_t q) {
            return movePt(f, q, i, x, y);
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
            return rotPoint(f(i), a);
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
            return scalePoint(f(i), x, y);
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
            return translatePoint(f(i), x, y);
        });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CONCATENATE////////////////////////////////////////////////////
    template <typename Func>
    Func Concatenate(const Func &f1, const Func &f2)
    {
        return (f1 == nullptr && f2 == nullptr) ? f1 : (f1 == nullptr ? f2 : (f2 == nullptr ? f1 : types::PointFunction([f1, f2](types::node_index_t i) {
            const size_t fSize = constants::NUM_OF_CUBIC_BEZIER_NODES * domainSize(f1);
            return i < fSize ? f1(i) : f2(i - fSize);
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
        const std::set<std::pair<size_t, size_t>> points;
        const size_t res;

        const types::point_2d deCasteljau(types::PointFunction f, double t, types::node_index_t i) const
        {
            const types::point_2d p0 = f(i);
            const types::point_2d p1 = f(i + 1);
            const types::point_2d p2 = f(i + 2);
            const types::point_2d p3 = f(i + 3);

            const types::point_2d b0 = (1 - t) * p0 + t * p1;
            const types::point_2d b1 = (1 - t) * p1 + t * p2;
            const types::point_2d b2 = (1 - t) * p2 + t * p3;

            const types::point_2d b01 = (1 - t) * b0 + t * b1;
            const types::point_2d b11 = (1 - t) * b1 + t * b2;

            const types::point_2d b012 = (1 - t) * b01 + t * b11;
            return b012;
        }

        std::pair<size_t, size_t> deCasteljau(const std::vector<types::point_2d> &funcPoints, double t, types::node_index_t i) const
        {
            const types::point_2d b0 = (1 - t) * funcPoints[i] + t * funcPoints[i + 1];
            const types::point_2d b1 = (1 - t) * funcPoints[i + 1] + t * funcPoints[i + 2];
            const types::point_2d b2 = (1 - t) * funcPoints[i + 2] + t * funcPoints[i + 3];

            const types::point_2d b01 = (1 - t) * b0 + t * b1;
            const types::point_2d b11 = (1 - t) * b1 + t * b2;

            const types::point_2d b012 = (1 - t) * b01 + t * b11;
            return std::make_pair<size_t, size_t>((size_t)b012.Y, (size_t)b012.X);
        }

        std::set<std::pair<size_t, size_t>> calculateSegment(const std::vector<types::point_2d> &funcPoints, types::node_index_t startSegment) const
        {
            const double precision = 1 / this->res;
            std::vector<double> ts(res);
            std::generate(ts.begin(), ts.end(), [precision, n = -precision]() mutable {n += precision; return n; });
            const auto f = [this, funcPoints, startSegment](double t) { return deCasteljau(funcPoints, t, startSegment); };
            std::set<std::pair<size_t, size_t>> segmentPoints;
            std::transform(ts.cbegin(), ts.cend(), std::inserter(segmentPoints, segmentPoints.begin()), f);
            return segmentPoints;
        }

        //not functional
        const std::set<std::pair<size_t, size_t>> calculateCurve(types::PointFunction f, int segments = 1)
        {
            std::cout << "HERE10\n";
            const std::vector<types::point_2d> funcPoints = decodeFunction(f);
            std::cout << "HERE1\n";
            std::vector<int> args(segments);
            std::cout << "HERE2\n";
            std::generate(args.begin(), args.end(), [n = -4]() mutable {n += 4; return n; });
            std::cout << "HERE3\n";
            std::set<std::pair<size_t, size_t>> rPoints;
            std::cout << "HERE4\n";
            for (int i = 0; i < segments; i++)
            {
                std::set<std::pair<size_t, size_t>> tmp = calculateSegment(funcPoints, i * 4);
                rPoints.insert(tmp.begin(), tmp.end());
            }
            std::cout << "HERE5\n";
            return rPoints;
        }

        const std::string drawCurve(char fb, char bg, std::set<std::pair<size_t, size_t>>::iterator it, size_t count = 0) const
        {
            const size_t i = count / this->res;
            const size_t j = count % this->res;

            const char sign = (it->first == i && it->second == j) ? fb : bg;
            (it->first == i && it->second == j) ? it++ : it;

            return ((count + 1) % this->res == 0) ? "\n" : "" + sign + (count + 1 == this->res ? "" : drawCurve(fb, bg, it, count + 1));
        }

    public:
        P3CurvePlotter(types::PointFunction f, int segments = 1, size_t resolution = 80) : res(resolution), points(calculateCurve(f, segments))
        {
        }

        void Print(std::ostream &s = std::cout, char fb = '*', char bg = ' ') const
        {
            s << drawCurve(fb, bg, points.begin());
        }

        const types::point_2d operator()(types::PointFunction f, double t, types::node_index_t i) const
        {
            return deCasteljau(f, t, 4 * i);
        }
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace bezier
#endif // !BEZIER_H
