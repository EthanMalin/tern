#include "tests.h"

int main(int argc, char **argv) {
  if (run_tests()) {
    printf("All tests passed.\n");
  } else {
    printf("One or more test(s) failed.\n");
  }
  return 0;
}