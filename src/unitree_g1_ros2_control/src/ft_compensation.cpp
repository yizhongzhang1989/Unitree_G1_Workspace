#include "unitree_g1_ros2_control/ft_compensation.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace unitree_g1_ros2_control {

namespace {

Vector3 read_vector(const YAML::Node& node, const std::string& key) {
    const YAML::Node entry = node[key];
    if (!entry || !entry.IsSequence() || entry.size() != 3) {
        throw std::runtime_error(key + " must hold three values");
    }
    Vector3 values{};
    for (std::size_t index = 0; index < 3; ++index) {
        values[index] = entry[index].as<double>();
        if (!std::isfinite(values[index])) {
            throw std::runtime_error(key + " holds a non-finite value");
        }
    }
    return values;
}

/// Rotate the force and the torque halves of a wrench by the same rotation.
Wrench rotate_wrench(const Matrix3& rotation, const Wrench& wrench) {
    const Vector3 force = rotate(rotation, {wrench[0], wrench[1], wrench[2]});
    const Vector3 torque = rotate(rotation, {wrench[3], wrench[4], wrench[5]});
    return {force[0], force[1], force[2], torque[0], torque[1], torque[2]};
}

/// Move the point the torque is taken about by `offset`. Force is unaffected.
Wrench shift_wrench(const Vector3& offset, const Wrench& wrench) {
    const Vector3 transfer =
        cross(offset, {wrench[0], wrench[1], wrench[2]});
    return {
        wrench[0], wrench[1], wrench[2],
        wrench[3] + transfer[0], wrench[4] + transfer[1],
        wrench[5] + transfer[2],
    };
}

/// The tool's weight expressed the way the sensor reports it.
Wrench tool_reading(const FtCalibration& calibration, const Vector3& gravity) {
    const Vector3 back = {
        -calibration.origin[0], -calibration.origin[1], -calibration.origin[2]};
    Wrench reading = rotate_wrench(
        calibration.rotation,
        shift_wrench(back, tool_wrench(calibration, gravity)));
    for (double& value : reading) value *= calibration.polarity;
    return reading;
}

}  // namespace

bool FtCalibration::load(
    const std::string& path, const std::string& side, FtCalibration& calibration) {
    const std::string resolved = resolve_path(path);
    YAML::Node document;
    try {
        document = YAML::LoadFile(resolved);
    } catch (const std::exception& error) {
        throw std::runtime_error(
            "cannot read force sensor calibration " + resolved + ": " + error.what());
    }
    YAML::Node table = document;
    if (document.size() == 1 && document.begin()->second["ros__parameters"]) {
        table = document.begin()->second["ros__parameters"];
    }
    const YAML::Node entry = table[side];
    if (!entry) return false;

    calibration.force_bias = read_vector(entry, "force_bias");
    calibration.torque_bias = read_vector(entry, "torque_bias");
    calibration.com = read_vector(entry, "tool_com");
    calibration.mass = entry["tool_mass"].as<double>();
    if (!std::isfinite(calibration.mass) || calibration.mass < 0.0) {
        throw std::runtime_error(side + " tool_mass must be finite and non-negative");
    }
    calibration.polarity = entry["polarity"] ? entry["polarity"].as<double>() : 1.0;
    if (calibration.polarity != 1.0 && calibration.polarity != -1.0) {
        throw std::runtime_error(side + " polarity must be +1 or -1");
    }
    const YAML::Node rotation = entry["rotation"];
    if (rotation) {
        if (!rotation.IsSequence() || rotation.size() != 9) {
            throw std::runtime_error(side + " rotation must hold nine values");
        }
        for (std::size_t index = 0; index < 9; ++index) {
            calibration.rotation[index] = rotation[index].as<double>();
        }
    }
    calibration.frame = entry["frame"] ? entry["frame"].as<std::string>() : side;
    calibration.origin = entry["measurement_origin"]
        ? read_vector(entry, "measurement_origin")
        : Vector3{};
    return true;
}

Wrench tool_wrench(const FtCalibration& calibration, const Vector3& gravity) {
    const Vector3 moment = cross(
        {calibration.mass * calibration.com[0], calibration.mass * calibration.com[1],
         calibration.mass * calibration.com[2]},
        gravity);
    return {
        calibration.mass * gravity[0], calibration.mass * gravity[1],
        calibration.mass * gravity[2], moment[0], moment[1], moment[2],
    };
}

Wrench net_wrench(
    const Wrench& raw, const FtCalibration& calibration, const Vector3& gravity) {
    Wrench unbiased{};
    for (std::size_t index = 0; index < 3; ++index) {
        unbiased[index] = raw[index] - calibration.force_bias[index];
        unbiased[index + 3] = raw[index + 3] - calibration.torque_bias[index];
    }
    Wrench physical = rotate_wrench(transpose(calibration.rotation), unbiased);
    for (double& value : physical) value *= calibration.polarity;
    physical = shift_wrench(calibration.origin, physical);
    const Wrench tool = tool_wrench(calibration, gravity);
    Wrench net{};
    for (std::size_t index = 0; index < 6; ++index) {
        net[index] = physical[index] - tool[index];
    }
    return net;
}

FtCalibration rezeroed(
    const Wrench& raw, const FtCalibration& calibration, const Vector3& gravity) {
    const Wrench tool = tool_reading(calibration, gravity);
    FtCalibration updated = calibration;
    for (std::size_t index = 0; index < 3; ++index) {
        updated.force_bias[index] = raw[index] - tool[index];
        updated.torque_bias[index] = raw[index + 3] - tool[index + 3];
    }
    return updated;
}

}  // namespace unitree_g1_ros2_control
