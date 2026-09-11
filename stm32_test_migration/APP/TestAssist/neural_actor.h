#ifndef __NEURAL_ACTOR_H_
#define __NEURAL_ACTOR_H_

#define NEURAL_ACTOR_INPUTS 13
#define NEURAL_ACTOR_HIDDEN 32
#define NEURAL_ACTOR_OUTPUTS 2

void NeuralActor_Init(void);
void NeuralActor_Predict(const float input[NEURAL_ACTOR_INPUTS],
                         float output[NEURAL_ACTOR_OUTPUTS]);

#endif
