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

namespace bezier
{

    namespace constants
    {
        constexpr int NUM_OF_CUBIC_BEZIER_NODES = 4;
        constexpr double ARC = 4 * (sqrt(2) - 1) / 3;
    } // namespace constants

    namespace types
    {
        using real_t = long double;
        using node_index_t = size_t;

        //////////////////POINT 2D/////////////////////////////
        class point_2d
        {
        public:
            real_t X;
            real_t Y;

            point_2d() = delete;
            point_2d(real_t xp, real_t yp) : X(xp), Y(yp){};

            const point_2d operator+(const point_2d &p) const noexcept
            {
                return point_2d(this->X + p.X, this->Y + p.Y);
            }

            const point_2d operator*(real_t scalar) const noexcept
            {
                return point_2d(this->X * scalar, this->Y * scalar);
            }

            friend const point_2d operator*(real_t scalar, const point_2d &p) noexcept;

            bool operator==(const point_2d &p) const noexcept
            {
                return (this->X == p.X) && (this->Y == p.Y);
            }

            friend std::ostream &operator<<(std::ostream &os, const point_2d &p) noexcept;
        };

        const point_2d operator*(real_t scalar, const point_2d &p) noexcept
        {
            return point_2d(p.X * scalar, p.Y * scalar);
        }

        std::ostream &operator<<(std::ostream &os, const point_2d &p) noexcept
        {
            os << '(' << p.X << ", " << p.Y << ')';
            return os;
        }

        //////////////////////////////POINT FUNCTION FUNCTOR//////////////////////////////////////////////////
        struct PointFunction
        {
            //potrzebny będzie nam do przyspieszenia konkatenacji
            //atrybuty nie powinny być const aby pozwolić użytkownikowi na operator=
            std::function<const point_2d(node_index_t)> f;
            node_index_t leftSubtreeSize;
            node_index_t rightSubtreeSize;

            PointFunction(const std::function<const point_2d(node_index_t)> &f1, node_index_t lsize, node_index_t rsize) : f(f1), leftSubtreeSize(lsize), rightSubtreeSize(rsize) {}

            //od strony użytkownika powinien mieć dostęp do tego operatora
            const point_2d operator()(node_index_t i) const
            {
                return f(i);
            }

            PointFunction &operator=(const PointFunction &) = default;
        };
    } // namespace types

    //////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CUP////////////////////////////////////////////////////
    //tworzy obiekt PointFunction z rozmiarem lewego poddrzewa = 4 (technicznie jest 0, lecz upraszcza to poźniejszą rekurencje)i
    //funcją która mapuje 4 punkty tak jak w treści za pomocą operatora ?
    const types::PointFunction Cup()
    {
        return types::PointFunction(
            [](types::node_index_t i) {
                return i == 0   ? types::point_2d(-1, 1)
                       : i == 1 ? types::point_2d(-1, -1)
                       : i == 2 ? types::point_2d(1, -1)
                       : i == 3 ? types::point_2d(1, 1)
                                : throw std::out_of_range("a curve node index is out of range");
            },
            constants::NUM_OF_CUBIC_BEZIER_NODES, 0);
    }

    ////////////////////////////////////////CAP////////////////////////////////////////////////////
    //analogicznie
    const types::PointFunction Cap()
    {
        return types::PointFunction(
            [](types::node_index_t i) {
                return i == 0   ? types::point_2d(-1, -1)
                       : i == 1 ? types::point_2d(-1, 1)
                       : i == 2 ? types::point_2d(1, 1)
                       : i == 3 ? types::point_2d(1, -1)
                                : throw std::out_of_range("a curve node index is out of range");
            },
            constants::NUM_OF_CUBIC_BEZIER_NODES, 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CONVEX ARC////////////////////////////////////////////////////
    //analogicznie
    const types::PointFunction ConvexArc()
    {
        return types::PointFunction(
            [](types::node_index_t i) {
                return i == 0   ? types::point_2d(0, 1)
                       : i == 1 ? types::point_2d(constants::ARC, 1)
                       : i == 2 ? types::point_2d(1, constants::ARC)
                       : i == 3 ? types::point_2d(1, 0)
                                : throw std::out_of_range("a curve node index is out of range");
            },
            constants::NUM_OF_CUBIC_BEZIER_NODES, 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CONCAVE ARC////////////////////////////////////////////////////
    //analogicznie
    const types::PointFunction ConcaveArc()
    {
        return types::PointFunction(
            [](types::node_index_t i) {
                return i == 0   ? types::point_2d(0, 1)
                       : i == 1 ? types::point_2d(0, 1 - constants::ARC)
                       : i == 2 ? types::point_2d(1 - constants::ARC, 0)
                       : i == 3 ? types::point_2d(1, 0)
                                : throw std::out_of_range("a curve node index is out of range");
            },
            constants::NUM_OF_CUBIC_BEZIER_NODES, 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////LINE SEGMENT////////////////////////////////////////////////////
    //analogicznie
    const types::PointFunction LineSegment(const types::point_2d &p, const types::point_2d &q)
    {
        return types::PointFunction(
            [p, q](types::node_index_t i) {
                return i == 0   ? types::point_2d(p.X, p.Y)
                       : i == 1 ? types::point_2d(p.X, p.Y)
                       : i == 2 ? types::point_2d(q.X, q.Y)
                       : i == 3 ? types::point_2d(q.X, q.Y)
                                : throw std::out_of_range("a curve node index is out of range");
            },
            constants::NUM_OF_CUBIC_BEZIER_NODES, 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////MOVE POINT////////////////////////////////////////////////////
    //------------------------------MOVE PT--------------------------------------------------------------//
    //zwraca odpowiednio przeniesiony konkretny punkt
    template <typename numeric>
    const types::point_2d movePt(const types::PointFunction &f, types::node_index_t q, types::node_index_t i, numeric x, numeric y)
    {
        return q == i ? f(q) + types::point_2d(x, y) : f(q);
    }

    //zwraca funktor bazujący na wejściowym zmieniając return value wykorzystując movePt()
    template <typename numeric>
    const types::PointFunction MovePoint(const types::PointFunction &f, types::node_index_t i, numeric x, numeric y)
    {
        static_assert(std::is_arithmetic<numeric>());
        return types::PointFunction([f, i, x, y](types::node_index_t q) {
            return movePt(f, q, i, x, y);
        },
                                    f.leftSubtreeSize, f.rightSubtreeSize);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////ROTATE////////////////////////////////////////////////////
    //------------------------------RADIANS--------------------------------------------------------------//
    //zmienia stopnie na radiany
    template <typename degrees>
    double radians(degrees deg)
    {
        return ((double)deg * (M_PI / 180.0f));
    }

    //------------------------------ROT POINTS--------------------------------------------------------------//
    //obraca otrzymany punkt o deg stopni
    template <typename degrees>
    const types::point_2d rotPoint(const types::point_2d &p, degrees deg)
    {
        return types::point_2d(p.X * cos(radians(deg)) - p.Y * sin(radians(deg)), p.X * sin(radians(deg)) + p.Y * cos(radians(deg)));
    }

    //zwraca funktor bazujący na wejściowym zmieniając return value wykorzystując rotPoint()
    template <typename degrees>
    const types::PointFunction Rotate(const types::PointFunction &f, degrees a)
    {
        static_assert(std::is_arithmetic<degrees>());
        return types::PointFunction([f, a](types::node_index_t i) {
            return rotPoint(f(i), a);
        },
                                    f.leftSubtreeSize, f.rightSubtreeSize);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////SCALE////////////////////////////////////////////////////
    //------------------------------SCALE POINT--------------------------------------------------------------//
    //zwraca przeskalowany punkt otrzymany w argumencie
    template <typename numeric>
    const types::point_2d scalePoint(const types::point_2d &p, numeric x, numeric y)
    {
        return types::point_2d(p.X * x, p.Y * y);
    }

    //zwraca funktor bazujący na wejściowym zmieniając return value wykorzystując scalePoint()
    template <typename numeric>
    const types::PointFunction Scale(const types::PointFunction &f, numeric x, numeric y)
    {
        static_assert(std::is_arithmetic<numeric>());
        return types::PointFunction([f, x, y](types::node_index_t i) {
            return scalePoint(f(i), x, y);
        },
                                    f.leftSubtreeSize, f.rightSubtreeSize);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////TRANSLATE////////////////////////////////////////////////////
    //------------------------------TRANSLATE POINT--------------------------------------------------------------//
    //zwraca odpowiednio przeniesiony punkt podany w argumencie
    template <typename numeric>
    const types::point_2d translatePoint(const types::point_2d &p, numeric x, numeric y)
    {
        return p + types::point_2d(x, y);
    }

    //zwraca funktor bazujący na wejściowym zmieniając return value wykorzystując translatePoint()
    template <typename numeric>
    const types::PointFunction Translate(const types::PointFunction &f, numeric x, numeric y)
    {
        static_assert(std::is_arithmetic<numeric>());
        return types::PointFunction([f, x, y](types::node_index_t i) {
            return translatePoint(f(i), x, y);
        },
                                    f.leftSubtreeSize, f.rightSubtreeSize);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////CONCATENATE////////////////////////////////////////////////////
    //ponieważ konkatenacja funckji tworzy drzewo binarne, możemy wykorzystać binsearch, dzięki wiedzy o rozmiarach poddrzew
    //nowy funktor ma rozmiary podrzew będące odpowiednio sumą poddrzew f1 i f2
    //a sam w sobie wykonuje rekurencyjnego binsearcha
    template <typename Func>
    const Func Concatenate(const Func &f1, const Func &f2)
    {
        return types::PointFunction([f1, f2](types::node_index_t i) {
            const types::node_index_t leftSize = f1.leftSubtreeSize + f1.rightSubtreeSize;

            return i < leftSize ? f1(i) : f2(i - leftSize);
        },
                                    f1.leftSubtreeSize + f1.rightSubtreeSize, f2.leftSubtreeSize + f2.rightSubtreeSize);
    }

    //wersja wieloargumentowa
    template <typename Func, typename... Rest>
    const Func Concatenate(const Func &f1, const Func &f2, Rest &&...funcs)
    {
        return Concatenate(Concatenate(f1, f2), funcs...);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////P3 CURVE PLOTTER////////////////////////////////////////////////////
    class P3CurvePlotter
    {
        //struktura niestety imperatywna, co powoduje późniejsze lekkie odbiegnięcie od funkcyjności
        //tj. korzystanie ze zmiennych tymczasowych nieconst
        //na rzecz możliwości wykorzystania funckji std::transform() i std::generate()
        //celem obejścia korzystania z pętli for
        const int res;
        const std::set<std::pair<int, int>> points;

        //formuła wyliczająca punkt - algorytm dawał te same wyniki
        const types::point_2d deCasteljau(types::PointFunction f, long double t, types::node_index_t i) const
        {
            return f(i) * std::pow(1 - t, 3) + 3 * f(i + 1) * t * std::pow(1 - t, 2) + 3 * f(i + 2) * std::pow(t, 2) * (1 - t) + f(i + 3) * std::pow(t, 3);
        }

        //mapowanie punktu (double)[-1, 1]x[-1, 1] na (int)[-res, res]x[-res, res]
        const std::pair<int, int> convertToIntPoint(const types::point_2d &p) const
        {
            return std::make_pair<int, int>(floor((p.Y * ((long double)res / 2))), floor((p.X * ((long double)res / 2))));
        }

        //generowanie punktów
        std::set<std::pair<int, int>> calculateSegment(const types::PointFunction &f, types::node_index_t startSegment) const
        {
            //nie chcemy generować punktów spoza obszaru
            const int resStart = -res / 2;
            const int resEnd = res / 2 + (res % 2);

            //krok w liczeniu punktów
            const long double precision = 1 / ((long double)(res * res));

            //zwrotna krzywa
            std::set<std::pair<int, int>> segmentPoints;

            //generowanie ciągu długości res^2 od 0 do 1 z równym interwałem
            std::vector<long double> ts(res * res + 1);
            std::generate(ts.begin(), ts.end(), [precision, t = -precision]() mutable {t += precision; return t; });

            //lambda wymagająca tylko jednego argumentu do std::transform
            const auto deCast = [resStart, resEnd, this, f, startSegment](long double t) {
                const std::pair<int, int> tmp = convertToIntPoint(this->operator()(f, t, startSegment));
                //sprawdzamy czy wygenerowany punkt mieści się w obszarze, jeśli nie to zwracamy pkt(res + 1, res + 1)
                return tmp.first >= resStart && tmp.second >= resStart && tmp.first < resEnd && tmp.second < resEnd ? tmp : std::make_pair<int, int>(res + 1, res + 1);
            };

            //tworzymy zbiór {f(t): t nalezy do [0, 1]}
            std::transform(ts.cbegin(), ts.cend(), std::inserter(segmentPoints, segmentPoints.begin()), deCast);
            //usuwamy punkt (res + 1, res + 1)
            segmentPoints.erase(std::make_pair<int, int>(res + 1, res + 1));

            return segmentPoints;
        }

        //próba ufuncyjnienia struktury imperatywnej
        void myInserter(const types::PointFunction &f, size_t segments, std::set<std::pair<int, int>> &rPoints, types::node_index_t startSegment = 0)
        {
            if (startSegment == segments)
                return;
            const std::set<std::pair<int, int>> tmp = calculateSegment(f, startSegment);
            rPoints.insert(tmp.begin(), tmp.end());
            myInserter(f, segments, rPoints, startSegment + 1);
        }

        //przygotowyjemy dane do wyświetlenia - argument res jest wymagany bo w tym momencie jeszcze nie powstał jeszcze atrybut klasy
        const std::set<std::pair<int, int>> calculateCurve(const types::PointFunction &f, size_t segments)
        {
            std::set<std::pair<int, int>> rPoints;
            myInserter(f, segments, rPoints);
            return rPoints;
        }

        //generowanie pojedynczej linii
        //pętla for przepisana na rekurencje
        std::string drawRow(char fb, char bg, std::set<std::pair<int, int>>::iterator &it, const int i, int j) const
        {
            const int end = res / 2 + (res % 2);
            return j < end ? ((it != points.end() && it->first == i && it->second == j) ? fb + drawRow(fb, bg, ++it, i, j + 1) : bg + drawRow(fb, bg, it, i, j + 1))
                           : "\n";
        }

        const std::string drawCurve(char fb, char bg, std::set<std::pair<int, int>>::iterator &it, int i) const
        {
            const int end = res / 2 + (res % 2);
            const std::string s = drawRow(fb, bg, it, i, -res / 2);
            return i < end ? drawCurve(fb, bg, it, i + 1) + s : "";
        }

    public:
        P3CurvePlotter(types::PointFunction f, size_t segments = 1, size_t resolution = 80) : res(resolution), points(calculateCurve(f, segments))
        {
        }

        void Print(std::ostream &s = std::cout, char fb = '*', char bg = ' ') const
        {
            std::set<std::pair<int, int>>::iterator it = points.begin();
            s << drawCurve(fb, bg, it, -res / 2);
        }

        const types::point_2d operator()(types::PointFunction f, double t, types::node_index_t i) const
        {
            return deCasteljau(f, t, constants::NUM_OF_CUBIC_BEZIER_NODES * i);
        }
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace bezier
#endif // !BEZIER_H
