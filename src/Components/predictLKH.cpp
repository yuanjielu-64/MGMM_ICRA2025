#include "Components/predictLKH.h"
#include "Utils/Logger.hpp"
#include "treelite/c_api.h"
#include "treelite/c_api_runtime.h"

bool LoadModel(PredictorHandle &model, std::string modelName, const char fname[], const char type[]) {
    std::string path = fname + modelName + "/lgb_" + type + ".so";
    Antipatrea::Logger::m_out << "loading model <" << path.c_str() << ">" << std::endl;
    return TreelitePredictorLoad(path.c_str(), -1, &model) == 0;
}

bool Predict(PredictorHandle &model, const int nrStartGoalPairs, const double pairs[], double results[]) {
    //Antipatrea::Logger::m_out << "Predict ..nrPairs = " << nrStartGoalPairs << std::endl;
//  for(int i = 0; i < nrStartGoalPairs; ++i)
//	Antipatrea::Logger::m_out << i << ": " << pairs[4*i] << " " << pairs[4*i + 1] << " " << pairs[4*i + 2] << " " << pairs[4*i + 3] << std::endl;

    DMatrixHandle batch;
    double miss = 0;
    void *missing_value = static_cast<void *>(&miss);

    int flag = TreeliteDMatrixCreateFromMat(pairs, "float64", nrStartGoalPairs, 4, missing_value, &batch);
    if (flag != 0)
        throw std::invalid_argument("Error creating1\n");

    size_t outLen;
    bool result = TreelitePredictorPredictBatch(
            model,
            batch,
            0,
            0,
            results,
            &outLen
    ) == 0;

    for (int i = 0; i < outLen; ++i) {
        results[i] = exp(results[i]);
    }

    //for(int i = 0; i < nrStartGoalPairs; ++i)
    //    Antipatrea::Logger::m_out << i << ": " << pairs[4*i] << " " << pairs[4*i + 1] << " " << pairs[4*i + 2] << " " << pairs[4*i + 3] << " " << exp(results[i]) << " " << std::endl;

    return result;
}