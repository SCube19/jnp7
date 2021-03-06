#include "bezier.h"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

int main() {

  //Koniczyna
  auto fn = bezier::Cap();
  //std::cout << ":00010\n";
  fn = bezier::MovePoint(fn, 0, 0.75, 1.25);
  //std::cout << ":00011\n";
  fn = bezier::MovePoint(fn, 3, -0.75, 1.25);
  //std::cout << ":00012\n";
  fn = bezier::Concatenate(fn,
                          bezier::Rotate(fn, 270),
                          bezier::Rotate(fn, 180),
                          bezier::Rotate(fn, 90));
  
  fn = bezier::Scale(fn, 1.0, 0.5);

  //std::cout << ":0001\n";
  const bezier::P3CurvePlotter plot1(fn, 4);
  //std::cout << ":0002\n";
  plot1.Print(std::cout, '*', '.');
  //std::cout << ":0003\n";
  std::ofstream file{"myclover.out"};
  //std::cout << ":0004\n";
  plot1.Print(file, 'o');

  // Gwiazda
  fn = bezier::ConcaveArc();
  fn = bezier::Concatenate(fn,
                           bezier::Rotate(fn, 270),
                           bezier::Rotate(fn, 180),
                           bezier::Rotate(fn, 90));
  const bezier::P3CurvePlotter plot2(bezier::Scale(bezier::Rotate(fn, 30), 1.0, 0.5), 4, 50);
  plot2.Print(std::cout, 'X', ' ');
  plot2.Print(std::cout, ' ', '0');

  //Buźka
  
  fn = bezier::ConvexArc();
  fn = bezier::Concatenate(fn,
                           bezier::Rotate(fn, 270),
                           bezier::Rotate(fn, 180),
                           bezier::Rotate(fn, 90),
                           bezier::Translate(bezier::Scale(bezier::Cup(), 0.625, 0.25), 0.0, -0.375),
                           bezier::Scale(bezier::Cap(), 0.125, 0.125),
                           bezier::LineSegment(bezier::types::point_2d{-0.625, 0.375},
                                               bezier::types::point_2d{-0.25, 0.5}),
                           bezier::LineSegment(bezier::types::point_2d{0.625, 0.375},
                                               bezier::types::point_2d{0.25, 0.5}));

  fn = bezier::Scale(fn, 1.0, 0.5);

  const bezier::P3CurvePlotter plot3(fn, 8, 60);
  plot3.Print();

  //Węzły (punkty kontrolne) krzywej – sprawdzenie poprawności węzłów
  constexpr bezier::types::real_t precision = std::pow(2, -16);
  bezier::types::real_t a, b;
  fn = bezier::Concatenate(bezier::Rotate(bezier::Cup(), 90),
                           bezier::MovePoint(bezier::MovePoint(bezier::Cap(), 0, 1.0, 0.0), 3, -1.0, 0.0));
  std::vector<std::pair<bezier::types::real_t, bezier::types::real_t>>
    v{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}, {0, -1}, {-1, 1}, {1, 1}, {0, -1}};
  for_each(v.begin(), v.end(),
    [&](auto & p) {
      static bezier::types::node_index_t k = 0;
      a = std::abs(p.first - fn(k).X);
      b = std::abs(p.second - fn(k).Y);
      assert(a <= precision && b <= precision);
      k++;
    }
  );
  std::cout << "Curve node correctness: PASS\n";

  //std::cout <<  "Tu1\n";
  // Sprawdzenie wyjątku
  fn = bezier::Concatenate(bezier::ConvexArc(),
                           bezier::ConcaveArc(),
                           bezier::Cap(),
                           bezier::Cup());

  //std::cout <<  "Tu2\n";
  try {
    //std::cout <<  "Tu3\n";
    a = fn(4 * bezier::constants::NUM_OF_CUBIC_BEZIER_NODES).X;
    std::cout << "Exception: FAIL\n";
  }
  catch (const std::out_of_range &) {
    std::cout << "Exception: PASS\n";
  }
  catch (...) {
    std::cout << "Exception: FAIL\n";
  }
}
