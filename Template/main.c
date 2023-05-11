#include "gd32f10x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"

int main(void)
{
    /* configure systick */
    systick_config();
    while(1)
    {}
}
