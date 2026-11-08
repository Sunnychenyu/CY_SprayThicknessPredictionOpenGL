#pragma once
#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <array>
#include <functional>
#include <memory>

namespace spraythickness::opengl {
class OpenGLCalibrationKernels;
std::shared_ptr<OpenGLCalibrationKernels> createCalibrationKernels();
struct CalibrationFieldInput {
    std::vector<std::array<float,4>> surface;
    std::vector<std::array<float,4>> spray;
    std::vector<std::array<double,4>> events;
    std::vector<double> observedUm;
    double initialHistorySeconds=0;
};
struct CalibrationFieldStatistics {
    double squaredError=0,cross=0,norm=0,maximum=0;
    std::array<double,5> thermal{};
};
// Created, used and destroyed with the same current OpenGL compute context.
// Spatial rates and the measured field remain resident across thermal candidates.
class OpenGLCalibrationField final {
public:
    OpenGLCalibrationField(const PaperGaussianParameters& model,const CalibrationFieldInput& input,
        const std::function<void()>& checkpoint={},
        std::shared_ptr<OpenGLCalibrationKernels> kernels={});
    ~OpenGLCalibrationField();
    CalibrationFieldStatistics evaluate(const HeatHistoryParameters& history,bool enabled,int mode,
        const std::function<void()>& checkpoint={});
    std::vector<double> values(const std::vector<int>& indices={});
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
class OpenGLProbabilityObjective final {
public:
    OpenGLProbabilityObjective(const std::vector<std::array<double,4>>& points,int components,
        double scaleU,double scaleV,double peak,std::shared_ptr<OpenGLCalibrationKernels> kernels={},
        bool fitThickness=false);
    ~OpenGLProbabilityObjective();
    double evaluate(const std::vector<double>& parameters,std::vector<double>* gradient,
        const std::function<void()>& checkpoint={});
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
}
