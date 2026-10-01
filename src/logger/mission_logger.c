#include <stdio.h>

void s10_mission_log(const char *message)
{
    if (message == 0)
    {
        return;
    }

    printf("%s\n", message);
}
