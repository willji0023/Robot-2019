// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <tuple>
#include <type_traits>

#include <Eigen/Core>
#include <frc/logging/CSVLogFile.h>
#include <wpi/StringRef.h>
#include <wpi/Twine.h>

#include "ControllerLabel.hpp"

namespace frc3512 {

template <int States, int Inputs, int Outputs>
class LinearController {
public:
    /**
     * Constructs a LinearController with logging functionality.
     *
     * @param controllerName Name of the controller log file.
     * @param stateNames     Names of all the states. Each should have the form
     *                       "State Name (Unit)".
     * @param inputNames     Names of all the inputs. Each should have the form
     *                       "Input Name (Unit)".
     * @param outputNames    Names of all the outputs. Each should have the form
     *                       "Output Name (Unit)".
     */
    template <typename... StateNames, typename... InputNames,
              typename... OutputNames,
              typename = std::enable_if_t<std::conjunction_v<
                  std::is_convertible_v<StateNames, ControllerLabel>...>>,
              typename = std::enable_if_t<std::conjunction_v<
                  std::is_convertible_v<InputNames, ControllerLabel>...>>,
              typename = std::enable_if_t<std::conjunction_v<
                  std::is_convertible_v<OutputNames, ControllerLabel>...>>>
    LinearController(wpi::StringRef controllerName,
                     const std::tuple<StateNames...>& stateNames,
                     const std::tuple<InputNames...>& inputNames,
                     const std::tuple<OutputNames...>& outputNames)
        : m_stateLogger{controllerName + "States", stateNames},
          m_inputLogger{controllerName + "Inputs", inputNames},
          m_outputLogger{controllerName + "Outputs", outputNames} {
        static_assert(std::tuple_size_v<decltype(stateNames)> == States,
                      "Number of state names doesn't match number of states");
        static_assert(std::tuple_size_v<decltype(inputNames)> == Inputs,
                      "Number of input names doesn't match number of inputs");
        static_assert(std::tuple_size_v<decltype(outputNames)> == Outputs,
                      "Number of output names doesn't match number of outputs");
    }

    wpi::Twine EstLabel(wpi::StringRef name, wpi::StringRef unit) {
        return name + " estimate (" + unit + ")";
    }

    wpi::Twine RefLabel(wpi::StringRef name, wpi::StringRef unit) {
        return name + " reference (" + unit + ")";
    }

    wpi::Twine MeasurementLabel(wpi::StringRef name, wpi::StringRef unit) {
        return name + " measurement (" + unit + ")";
    }

    /**
     * Returns tuple of reference labels from state names.
     */
    template <typename... StateNames>
    auto MakeReferenceLabels(const std::tuple<StateNames...>& stateNames) {
        std::tuple<StateNames...> refNames = stateNames;
    }

    /**
     * Returns the reference vector.
     */
    virtual const Eigen::Matrix<double, States, 1>& GetReferences() const = 0;

    /**
     * Returns the state vector.
     */
    virtual const Eigen::Matrix<double, States, 1>& GetStates() const = 0;

    /**
     * Returns the input vector.
     */
    virtual const Eigen::Matrix<double, Inputs, 1>& GetInputs() const = 0;

    /**
     * Returns the output vector.
     */
    virtual const Eigen::Matrix<double, Outputs, 1>& GetOutputs() const = 0;

    void Log(units::second_t time) {
        const auto& r = GetReferences();
        const auto& x = GetStates();
        const auto& u = GetInputs();
        const auto& y = GetOutputs();

        m_stateLogger.Log(time, a2t(x));
        m_inputLogger.Log(time, a2t(u));
        m_outputLogger.Log(time, a2t(y));
    }

    void Log() {
        const auto& r = GetReferences();
        const auto& x = GetStates();
        const auto& u = GetInputs();
        const auto& y = GetOutputs();

        m_stateLogger.Log(a2t(x));
        m_inputLogger.Log(a2t(u));
        m_outputLogger.Log(a2t(y));
    }

private:
    frc::CSVLogFile m_stateLogger;
    frc::CSVLogFile m_inputLogger;
    frc::CSVLogFile m_outputLogger;
};

}  // namespace frc3512
