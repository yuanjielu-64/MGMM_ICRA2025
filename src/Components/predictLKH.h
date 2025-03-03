#ifndef PREDICTLKH_H
#define PREDICTLKH_H

#include "treelite/c_api.h"
#include "treelite/c_api_runtime.h"
#include <vector>
#include <math.h>
#include <algorithm>

bool LoadModel(PredictorHandle &model, std::string modelName, const char fname[],  const char type[]);
bool Predict(PredictorHandle &model, const int nrStartGoalPairs, const double pairs[], double results[]);

#endif
