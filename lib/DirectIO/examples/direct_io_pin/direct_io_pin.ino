#include <DirectIO.h>

OutputPin pin(4);

void setup() {}

void loop() {
  while(1) {
    pin = HIGH;
    pin = LOW;
  }
}

