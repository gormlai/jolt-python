#include <stdio.h>
#include "jolt_lib.h"

int main(const int argc, const char **argv) {
  printf("Example of using Jolt Lib!\n");

  jolt_init();
  jolt_start();
  jolt_update();
  jolt_shutdown();

  return 0;

}

