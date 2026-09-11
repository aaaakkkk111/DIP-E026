#include <stdio.h>

#include "neural_actor.h"

int main(void)
{
    float input[NEURAL_ACTOR_INPUTS];
    float output[NEURAL_ACTOR_OUTPUTS];
    int i;

    for (;;)
    {
        for (i = 0; i < NEURAL_ACTOR_INPUTS; ++i)
        {
            if (scanf("%f", &input[i]) != 1)
            {
                return 0;
            }
        }
        NeuralActor_Predict(input, output);
        printf("%.9g %.9g\n", (double)output[0], (double)output[1]);
    }
}
