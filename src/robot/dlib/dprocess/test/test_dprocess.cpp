#include <gtest/gtest.h>
#include "dprocess/dprocess.hpp"


using namespace dprocess;
using namespace std;

TEST(dprocess, main) {
  class Foo: public DProcess<Foo> {
  public:
    Foo(bool r) : DProcess(r){}
    void tick() override {
      usleep(1000);
    }
  };

  Foo f(true);
  f.spinOnce();
  f.join();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}