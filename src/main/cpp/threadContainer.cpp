#include "threadContainer.h"
#include <unistd.h>
#include <frc/commands/WaitCommand.h>

void *hello(void *newThread)
{
    while (true)
    {
        sleep(10);
        printf("Hello World");
    }
}