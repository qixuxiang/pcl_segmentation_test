#include <gtest/gtest.h>
#include "dprocess/dprocess.hpp"


using namespace dprocess;
using namespace std;

TEST(dprocess, main) {
  class Foo: public DProcess<Foo> {
  public:
    void tick() override {
      std::cout << "bar" << std::endl;
    }
  };

  Foo f;
  f.spinOnce();
  f.join();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}