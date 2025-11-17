#include "uwb.h"

void setup()
{
    Serial.begin(115200);
    start_uwb();
}

void loop()
{
    responder_loop();
}
