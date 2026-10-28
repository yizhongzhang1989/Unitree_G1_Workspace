#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
using Vector6 = Eigen::Matrix<double, 6, 1>;

class ReleasedGIL {
 public:
  ReleasedGIL() : state_(PyEval_SaveThread()) {}
  ~ReleasedGIL() { PyEval_RestoreThread(state_); }
  ReleasedGIL(const ReleasedGIL&) = delete;
  ReleasedGIL& operator=(const ReleasedGIL&) = delete;

 private:
  PyThreadState* state_;
};

struct Object {
  PyObject* value;
  explicit Object(PyObject* v) : value(v) {
    if (!value) throw std::invalid_argument("Missing IK configuration value");
  }
  ~Object() { Py_DECREF(value); }
  Object(const Object&) = delete;
  Object& operator=(const Object&) = delete;
};

struct Buffer {
  Py_buffer view{};
  explicit Buffer(PyObject* object) {
    if (PyObject_GetBuffer(object, &view, PyBUF_FORMAT | PyBUF_STRIDES) < 0)
      throw std::invalid_argument("Expected a numeric IK buffer");
  }
  ~Buffer() { PyBuffer_Release(&view); }
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;
  Vector vector() const {
    if (!view.format || std::string(view.format) != "d" || view.itemsize != 8 ||
        view.ndim != 1 || view.strides[0] != 8)
      throw std::invalid_argument("Expected a contiguous float64 IK vector");
    return Eigen::Map<const Vector>(static_cast<const double*>(view.buf), view.shape[0]);
  }
  Eigen::Matrix3d rotation() const {
    if (!view.format || std::string(view.format) != "d" || view.itemsize != 8 ||
        view.ndim != 2 || view.shape[0] != 3 || view.shape[1] != 3)
      throw std::invalid_argument("Expected a float64 3x3 target rotation");
    Eigen::Matrix3d result;
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
      result(r, c) = *reinterpret_cast<const double*>(
          static_cast<const char*>(view.buf) + r * view.strides[0] + c * view.strides[1]);
    return result;
  }
};

double number(PyObject* self, const char* name) {
  Object value(PyObject_GetAttrString(self, name));
  const double result = PyFloat_AsDouble(value.value);
  if (PyErr_Occurred()) throw std::invalid_argument("Invalid numeric IK configuration");
  return result;
}

Vector array(PyObject* self, const char* name) {
  Object value(PyObject_GetAttrString(self, name));
  return Buffer(value.value).vector();
}

struct Solver {
  pinocchio::Model model;
  pinocchio::Data data;
  std::vector<bool> elbows;
  explicit Solver(pinocchio::Model m) : model(std::move(m)), data(model), elbows(model.nq, false) {
    for (pinocchio::JointIndex j = 1; j < model.njoints; ++j)
      if (model.names[j].find("elbow") != std::string::npos) elbows[model.idx_qs[j]] = true;
  }
};

constexpr const char* capsuleName = "g1_motion_session.ArmIK";
Solver& solver(PyObject* object) {
  auto* value = static_cast<Solver*>(PyCapsule_GetPointer(object, capsuleName));
  if (!value) throw std::invalid_argument("Invalid native IK model");
  return *value;
}

struct Goal {
  pinocchio::FrameIndex frame;
  std::vector<int> columns;
  pinocchio::SE3 target;
  std::vector<bool> ungated;
  Vector reference, gains;
};

struct Config {
  Vector lower, upper, nullLower, nullUpper;
  Vector6 scale;
  double lambda2, tolPos, tolOri, maxPos, maxOri, weight, gateLo, gateSpan;
  int iterations;
  std::vector<Goal> goals;
  Config(PyObject* self, PyObject* targets, int nq) {
    lower = array(self, "lower"); upper = array(self, "upper");
    if (lower.size() != nq || upper.size() != nq)
      throw std::invalid_argument("IK limits do not match model size");
    lambda2 = number(self, "_lambda2"); tolPos = number(self, "_tol_pos");
    tolOri = number(self, "_tol_ori"); maxPos = number(self, "_max_step_pos");
    maxOri = number(self, "_max_step_ori"); weight = number(self, "_rotation_weight");
    gateLo = number(self, "_gate_lo"); gateSpan = number(self, "_gate_span");
    iterations = static_cast<int>(number(self, "_iters"));
    scale << 1., 1., 1., std::sqrt(weight), std::sqrt(weight), std::sqrt(weight);
    Object tips(PyObject_GetAttrString(self, "_tip"));
    Object terms(PyObject_GetAttrString(self, "_null_terms"));
    if (!PyDict_Check(targets) || !PyDict_Check(tips.value) || !PyDict_Check(terms.value))
      throw std::invalid_argument("Expected IK target/configuration dictionaries");
    PyObject *side, *pose;
    Py_ssize_t position = 0;
    while (PyDict_Next(targets, &position, &side, &pose)) {
      auto* tip = PyDict_GetItemWithError(tips.value, side);
      auto* term = PyDict_GetItemWithError(terms.value, side);
      if (!tip || !term || !PyTuple_Check(tip) || !PyTuple_Check(term))
        throw std::invalid_argument("Unknown IK side or invalid null-space configuration");
      Goal goal{};
      goal.frame = static_cast<pinocchio::FrameIndex>(PyLong_AsUnsignedLong(PyTuple_GET_ITEM(tip, 0)));
      Buffer columns(PyTuple_GET_ITEM(tip, 1));
      if (columns.view.ndim != 1 || !columns.view.format ||
          (std::string(columns.view.format) != "l" && std::string(columns.view.format) != "q") ||
          columns.view.itemsize != 8 || columns.view.strides[0] != 8)
        throw std::invalid_argument("Expected contiguous int64 IK joint columns");
      for (Py_ssize_t i = 0; i < columns.view.shape[0]; ++i) {
        const auto index = static_cast<const std::int64_t*>(columns.view.buf)[i];
        if (index < 0 || index >= nq) throw std::invalid_argument("IK joint column out of range");
        goal.columns.push_back(static_cast<int>(index));
      }
      Object rotation(PyObject_GetAttrString(pose, "rotation"));
      Object translation(PyObject_GetAttrString(pose, "translation"));
      const auto xyz = Buffer(translation.value).vector();
      if (xyz.size() != 3) throw std::invalid_argument("Target translation must have three values");
      const auto targetRotation = Buffer(rotation.value).rotation();
      if (!xyz.allFinite() || !targetRotation.allFinite())
        throw std::invalid_argument("IK target must contain finite values");
      goal.target = pinocchio::SE3(targetRotation, xyz);
      Buffer ungated(PyTuple_GET_ITEM(term, 0));
      if (ungated.view.ndim != 1 || ungated.view.itemsize != 1 ||
          ungated.view.strides[0] != 1 || ungated.view.shape[0] != columns.view.shape[0])
        throw std::invalid_argument("Invalid null-space gate shape");
      for (Py_ssize_t i = 0; i < ungated.view.shape[0]; ++i)
        goal.ungated.push_back(static_cast<const unsigned char*>(ungated.view.buf)[i] != 0);
      goal.reference = Buffer(PyTuple_GET_ITEM(term, 1)).vector();
      auto* gains = PyTuple_GET_ITEM(term, 2);
      if (gains != Py_None) goal.gains = Buffer(gains).vector();
      if (goal.reference.size() != static_cast<int>(goal.columns.size()) ||
          (goal.gains.size() && goal.gains.size() != goal.reference.size()))
        throw std::invalid_argument("Null-space vector does not match joint columns");
      goals.push_back(std::move(goal));
    }
    if (PyErr_Occurred()) throw std::invalid_argument("Invalid IK configuration");
  }
};

void place(Solver& s, const Vector& q) {
  pinocchio::computeJointJacobians(s.model, s.data, q);
  pinocchio::updateFramePlacements(s.model, s.data);
}

Vector6 errorFor(const pinocchio::SE3& target, const pinocchio::SE3& actual) {
  Vector6 error;
  error.head<3>() = target.translation() - actual.translation();
  error.tail<3>() = pinocchio::log3(target.rotation() * actual.rotation().transpose());
  return error;
}

void cap(Vector6& error, const Config& cfg) {
  const double pos = error.head<3>().norm(), ori = error.tail<3>().norm();
  if (pos > cfg.maxPos && cfg.maxPos > 0.) error.head<3>() *= cfg.maxPos / pos;
  if (ori > cfg.maxOri && cfg.maxOri > 0.) error.tail<3>() *= cfg.maxOri / ori;
}

Matrix jacobian(Solver& s, const Goal& goal, const Vector6& scale) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> all(6, s.model.nv);
  all.setZero();
  pinocchio::getFrameJacobian(s.model, s.data, goal.frame, pinocchio::LOCAL_WORLD_ALIGNED, all);
  Matrix result(6, goal.columns.size());
  for (size_t i = 0; i < goal.columns.size(); ++i)
    result.col(i) = scale.array() * all.col(goal.columns[i]).array();
  return result;
}

Vector preference(const Vector& q, const Goal& goal, const Config& cfg) {
  if (!goal.gains.size()) return {};
  Vector result(goal.columns.size());
  bool nonzero = false;
  for (size_t i = 0; i < goal.columns.size(); ++i) {
    const double offset = q[goal.columns[i]] - goal.reference[i];
    const double gate = goal.ungated[i] ? 1. :
        std::min(std::max((std::abs(offset) - cfg.gateLo) / cfg.gateSpan, 0.), 1.);
    result[i] = -goal.gains[i] * gate * offset;
    nonzero = nonzero || result[i] != 0.;
  }
  return nonzero ? result : Vector();
}

void update(Vector& q, const Goal& goal, const Vector& step, const Config& cfg) {
  for (size_t i = 0; i < goal.columns.size(); ++i) {
    const int column = goal.columns[i];
    q[column] = std::min(std::max(q[column] + step[i], cfg.lower[column]), cfg.upper[column]);
  }
}

Matrix nullBasis(const Matrix& weighted) {
  Eigen::JacobiSVD<Matrix> svd(weighted, Eigen::ComputeFullV);
  const auto& singular = svd.singularValues();
  const double threshold = std::numeric_limits<double>::epsilon() *
      std::max(weighted.rows(), weighted.cols()) * singular[0];
  const int rank = static_cast<int>((singular.array() > threshold).count());
  return svd.matrixV().rightCols(weighted.cols() - rank);
}

bool chasing(const Vector6& error, const Config& cfg) {
  return error.head<3>().norm() > cfg.tolPos ||
      (cfg.weight > 0. && error.tail<3>().norm() > cfg.tolOri);
}

Vector6 evaluate(Solver& s, const Vector& q, const Goal& goal) {
  pinocchio::forwardKinematics(s.model, s.data, q);
  pinocchio::updateFramePlacements(s.model, s.data);
  return errorFor(goal.target, s.data.oMf[goal.frame]);
}

bool correctPrimary(Solver& s, Vector& q, const Goal& goal, const Config& cfg) {
  place(s, q);
  const auto error = errorFor(goal.target, s.data.oMf[goal.frame]);
  if (!chasing(error, cfg) || goal.columns.empty()) return false;
  auto limited = error;
  cap(limited, cfg);
  const auto weighted = jacobian(s, goal, cfg.scale);
  const Vector6 rhs = cfg.scale.array() * limited.array();
  const double before = (cfg.scale.array() * error.array()).matrix().squaredNorm();
  double bestAfter = before;
  Vector bestCandidate = q;
  for (int method = 0; method < 2; ++method) {
    if (method == 1 && cfg.lambda2 <= 0.) break;
    Matrix active = weighted;
    std::vector<bool> fixed(goal.columns.size(), false);
    Vector step = Vector::Zero(goal.columns.size());
    for (size_t pass = 0; pass <= goal.columns.size(); ++pass) {
      Eigen::JacobiSVD<Matrix> svd(active, Eigen::ComputeThinU | Eigen::ComputeThinV);
      const auto& singular = svd.singularValues();
      const double cutoff = singular.size() ? (method == 0 ? 1e-2 : 1e-3) * singular[0] : 0.;
      step = Vector::Zero(goal.columns.size());
      for (Eigen::Index i = 0; i < singular.size(); ++i) {
        if (singular[i] <= cutoff) continue;
        const double inv = method == 0
            ? 1. / singular[i]
            : singular[i] / (singular[i] * singular[i] + cfg.lambda2);
        step += inv * svd.matrixV().col(i) * svd.matrixU().col(i).dot(rhs);
      }
      if (!step.allFinite()) throw std::runtime_error("Non-finite high-precision IK correction");
      bool added = false;
      for (size_t i = 0; i < goal.columns.size(); ++i) {
        const int column = goal.columns[i];
        if (!fixed[i] && ((q[column] <= cfg.lower[column] + 1e-10 && step[i] < -1e-12) ||
                         (q[column] >= cfg.upper[column] - 1e-10 && step[i] > 1e-12))) {
          active.col(i).setZero();
          fixed[i] = true;
          added = true;
        }
      }
      if (!added) break;
    }
    double fraction = 1.;
    for (int trial = 0; trial < 8; ++trial, fraction *= .5) {
      Vector candidate = q;
      update(candidate, goal, fraction * step, cfg);
      const auto after = evaluate(s, candidate, goal);
      const double score = (cfg.scale.array() * after.array()).matrix().squaredNorm();
      if (score < bestAfter) {
        bestAfter = score;
        bestCandidate = std::move(candidate);
        if (method == 0 && trial == 0 && step.norm() <= .5) {
          q = std::move(bestCandidate);
          return true;
        }
        break;
      }
    }
  }
  if (bestAfter < before) {
    q = std::move(bestCandidate);
    return true;
  }
  return false;
}

int refinePrimary(Solver& s, Vector& q, const Config& cfg, int budget) {
  int iterations = 0;
  for (; iterations < budget; ++iterations) {
    bool active = false, changed = false;
    for (const auto& goal : cfg.goals) {
      if (chasing(evaluate(s, q, goal), cfg)) {
        active = true;
        changed = correctPrimary(s, q, goal, cfg) || changed;
      }
    }
    if (!active) break;
    if (!changed) return iterations + 1;
  }
  return iterations;
}

double preferenceCost(const Vector& q, const Config& cfg, const std::vector<bool>* mask = nullptr) {
  double cost = 0.;
  for (const auto& goal : cfg.goals) {
    if (!goal.gains.size()) continue;
    for (size_t i = 0; i < goal.columns.size(); ++i) {
      if (mask && !(*mask)[goal.columns[i]]) continue;
      const double distance = std::abs(q[goal.columns[i]] - goal.reference[i]);
      double energy = .5 * distance * distance;
      if (!goal.ungated[i]) {
        // Integral of the existing gated preference, so acceptance preserves its objective.
        const double extent = std::clamp(distance - cfg.gateLo, 0., cfg.gateSpan);
        energy = (.5 * cfg.gateLo * extent * extent + extent * extent * extent / 3.) /
            cfg.gateSpan;
        const double hi = cfg.gateLo + cfg.gateSpan;
        if (distance > hi) energy += .5 * (distance * distance - hi * hi);
      }
      cost += goal.gains[i] * energy;
    }
  }
  return cost;
}

std::vector<Vector6> poseErrors(Solver& s, const Vector& q, const Config& cfg) {
  pinocchio::forwardKinematics(s.model, s.data, q);
  pinocchio::updateFramePlacements(s.model, s.data);
  std::vector<Vector6> errors;
  for (const auto& goal : cfg.goals)
    errors.push_back(errorFor(goal.target, s.data.oMf[goal.frame]));
  return errors;
}

std::pair<double, double> errorNorms(const std::vector<Vector6>& errors) {
  double position = 0., orientation = 0.;
  for (const auto& error : errors) {
    position = std::max(position, error.head<3>().norm());
    orientation = std::max(orientation, error.tail<3>().norm());
  }
  return {position, orientation};
}

struct NullProgress {
  const char* state = "not_applicable";
  int iterations = 0;
  double initialCost = 0., finalCost = 0., initialElbow = 0., finalElbow = 0., gradient = 0.;
  double taskFraction = 1.;
};

struct NullMove {
  Vector step;
  double gradient = 0.;
};

NullMove preferenceMove(const Vector& q, const Goal& goal, const Config& cfg,
                        const Matrix& weighted) {
  const Vector pref = preference(q, goal, cfg);
  if (!pref.size()) return {Vector::Zero(goal.columns.size()), 0.};
  Matrix constrained = weighted;
  std::vector<bool> fixed(pref.size(), false);
  for (size_t pass = 0; pass <= goal.columns.size(); ++pass) {
    const Matrix basis = nullBasis(constrained);
    if (!basis.cols()) return {Vector::Zero(pref.size()), 0.};
    const Vector tangent = basis.transpose() * pref;
    const double gradient = tangent.norm();
    if (gradient <= 1e-8) return {Vector::Zero(pref.size()), gradient};
    Vector step = basis * tangent;
    const double length = step.norm();
    if (length > .2) step *= .2 / length;
    bool added = false;
    for (size_t i = 0; i < goal.columns.size(); ++i) {
      const int column = goal.columns[i];
      if (!fixed[i] && ((q[column] <= cfg.lower[column] + 1e-10 && step[i] < -1e-12) ||
                       (q[column] >= cfg.upper[column] - 1e-10 && step[i] > 1e-12))) {
        const auto row = constrained.rows();
        constrained.conservativeResize(row + 1, Eigen::NoChange);
        constrained.row(row).setZero();
        constrained(row, i) = 1.;
        fixed[i] = true;
        added = true;
      }
    }
    if (!added) return {std::move(step), gradient};
  }
  return {Vector::Zero(pref.size()), 0.};
}

Vector nullDirection(Solver& s, const Vector& q, const Config& cfg, double& gradient) {
  place(s, q);
  Vector direction = Vector::Zero(q.size());
  double norm2 = 0.;
  for (const auto& goal : cfg.goals) {
    if (!goal.gains.size() || goal.columns.empty()) continue;
    const auto weighted = jacobian(s, goal, cfg.scale);
    const auto move = preferenceMove(q, goal, cfg, weighted);
    norm2 += move.gradient * move.gradient;
    for (size_t i = 0; i < goal.columns.size(); ++i) direction[goal.columns[i]] += move.step[i];
  }
  gradient = std::sqrt(norm2);
  return direction;
}

bool preservesPose(const std::vector<Vector6>& after, const std::vector<Vector6>& primary,
                   const Config& cfg) {
  for (size_t i = 0; i < after.size(); ++i)
    if (after[i].head<3>().norm() > std::max(cfg.tolPos, primary[i].head<3>().norm()) + 1e-12 ||
        (cfg.weight > 0. &&
         after[i].tail<3>().norm() > std::max(cfg.tolOri, primary[i].tail<3>().norm()) + 1e-12))
      return false;
  return true;
}

void optimizePreferences(Solver& s, Vector& q, const Config& cfg, int& iterations,
                         NullProgress& progress) {
  const auto primaryErrors = poseErrors(s, q, cfg);
  progress.initialCost = progress.finalCost = preferenceCost(q, cfg);
  progress.initialElbow = progress.finalElbow = preferenceCost(q, cfg, &s.elbows);
  progress.state = "iteration_limit";
  int remainingRetractions = cfg.iterations;
  for (int pass = 0; pass < cfg.iterations; ++pass) {
    const Vector direction = nullDirection(s, q, cfg, progress.gradient);
    if (progress.gradient <= 1e-8) {
      progress.state = "stationary";
      break;
    }
    ++progress.iterations;
    ++iterations;
    double fraction = 1.;
    for (int i = 0; i < q.size(); ++i) {
      if (direction[i] > 0.) fraction = std::min(fraction, (cfg.upper[i] - q[i]) / direction[i]);
      if (direction[i] < 0.) fraction = std::min(fraction, (cfg.lower[i] - q[i]) / direction[i]);
    }
    bool accepted = false;
    for (int trial = 0; trial < 8 && fraction > 1e-12; ++trial, fraction *= .5) {
      Vector candidate = (q + fraction * direction).cwiseMax(cfg.lower).cwiseMin(cfg.upper);
      auto after = poseErrors(s, candidate, cfg);
      if (!preservesPose(after, primaryErrors, cfg) && remainingRetractions > 0) {
        const int used = refinePrimary(s, candidate, cfg, std::min(4, remainingRetractions));
        iterations += used;
        remainingRetractions -= used;
        after = poseErrors(s, candidate, cfg);
      }
      const double cost = preferenceCost(candidate, cfg);
      const double elbowCost = preferenceCost(candidate, cfg, &s.elbows);
      if (preservesPose(after, primaryErrors, cfg) && cost < progress.finalCost) {
        q = std::move(candidate);
        progress.finalCost = cost;
        progress.finalElbow = elbowCost;
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      progress.state = remainingRetractions == 0 ? "iteration_limit" : "blocked";
      break;
    }
  }
  nullDirection(s, q, cfg, progress.gradient);
  if (progress.gradient <= 1e-8) {
    progress.state = "stationary";
  }
}

std::pair<double, double> highPrecision(Solver& s, Vector& q, const Config& cfg, int& iterations,
                                      NullProgress& progress) {
  for (const auto& goal : cfg.goals)
    if (goal.frame >= s.model.frames.size()) throw std::invalid_argument("IK frame out of range");
  if (cfg.goals.empty()) return errorNorms(poseErrors(s, q, cfg));
  Config executable = cfg;
  if (cfg.nullLower.size()) {
    if ((cfg.lower.cwiseMax(cfg.nullLower).array() >
         cfg.upper.cwiseMin(cfg.nullUpper).array()).any())
      throw std::invalid_argument("Joint window does not intersect model limits");
  }
  const bool windowed = cfg.nullLower.size() &&
      ((q.array() >= cfg.nullLower.array() - 1e-9) &&
       (q.array() <= cfg.nullUpper.array() + 1e-9)).all();
  if (!windowed) {
    iterations = refinePrimary(s, q, cfg, cfg.iterations);
  } else {
    executable.lower = cfg.lower.cwiseMax(cfg.nullLower);
    executable.upper = cfg.upper.cwiseMin(cfg.nullUpper);
    const Vector start = q.cwiseMax(executable.lower).cwiseMin(executable.upper);
    place(s, start);
    std::vector<pinocchio::SE3> origins;
    std::vector<Vector6> twists;
    for (const auto& goal : cfg.goals) {
      origins.push_back(s.data.oMf[goal.frame]);
      auto target = goal.target;
      if (cfg.weight == 0.) target.rotation() = origins.back().rotation();
      twists.push_back(pinocchio::log6(origins.back().actInv(target)).toVector());
    }
    auto weightedObjective = [&cfg](const std::vector<Vector6>& errs) {
      double sum = 0.;
      for (const auto& e : errs)
        sum += (cfg.scale.array() * e.array()).matrix().squaredNorm();
      return sum;
    };
    const double startObjective = weightedObjective(poseErrors(s, start, cfg));
    double bestObjective = startObjective;
    Vector bestCandidate = start;
    double bestFraction = 0.;
    bool feasible = false;
    double fraction = 1.;
    for (int trial = 0; trial < 8; ++trial, fraction *= .5) {
      executable.tolPos = cfg.tolPos * fraction;
      executable.tolOri = cfg.tolOri * fraction;
      for (size_t i = 0; i < executable.goals.size(); ++i) {
        const Vector6 twist = fraction * twists[i];
        executable.goals[i].target = origins[i] * pinocchio::exp6(twist);
      }
      Vector candidate = start;
      iterations += refinePrimary(s, candidate, executable, cfg.iterations);
      const auto errors = poseErrors(s, candidate, executable);
      if (std::none_of(errors.begin(), errors.end(),
                       [&executable](const auto& error) { return chasing(error, executable); })) {
        q = std::move(candidate);
        progress.taskFraction = fraction;
        feasible = true;
        break;
      }
      const double fullObjective = weightedObjective(poseErrors(s, candidate, cfg));
      if (fullObjective < bestObjective - 1e-12) {
        bestObjective = fullObjective;
        bestCandidate = candidate;
        bestFraction = fraction;
      }
      if (cfg.iterations <= 0) break;
    }
    if (!feasible && bestFraction > 0.) {
      q = std::move(bestCandidate);
      progress.taskFraction = bestFraction;
      for (size_t i = 0; i < executable.goals.size(); ++i) {
        const Vector6 twist = bestFraction * twists[i];
        executable.goals[i].target = origins[i] * pinocchio::exp6(twist);
      }
      feasible = true;
    }
    if (!feasible) {
      q = start;
      progress.taskFraction = 0.;
      progress.state = "blocked";
      progress.initialCost = progress.finalCost = preferenceCost(q, cfg);
      progress.initialElbow = progress.finalElbow = preferenceCost(q, cfg, &s.elbows);
      return errorNorms(poseErrors(s, q, cfg));
    }
  }
  optimizePreferences(s, q, executable, iterations, progress);
  return errorNorms(poseErrors(s, q, cfg));
}

void destroy(PyObject* capsule) {
  delete static_cast<Solver*>(PyCapsule_GetPointer(capsule, capsuleName));
}

PyObject* create(PyObject*, PyObject* args) {
  const char* xml;
  PyObject* names;
  if (!PyArg_ParseTuple(args, "sO", &xml, &names)) return nullptr;
  try {
    if (!PyTuple_Check(names)) throw std::invalid_argument("Expected a tuple of arm joint names");
    pinocchio::Model full;
    pinocchio::urdf::buildModelFromXML(xml, full);
    std::vector<pinocchio::JointIndex> keep, locked;
    for (Py_ssize_t i = 0; i < PyTuple_GET_SIZE(names); ++i) {
      const char* name = PyUnicode_AsUTF8(PyTuple_GET_ITEM(names, i));
      if (!name || !full.existJointName(name)) throw std::invalid_argument("Missing native IK joint");
      keep.push_back(full.getJointId(name));
    }
    for (pinocchio::JointIndex j = 1; j < full.njoints; ++j)
      if (std::find(keep.begin(), keep.end(), j) == keep.end()) locked.push_back(j);
    auto model = pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full));
    if (model.nq != PyTuple_GET_SIZE(names)) throw std::invalid_argument("Native IK model size mismatch");
    auto instance = std::make_unique<Solver>(std::move(model));
    auto* capsule = PyCapsule_New(instance.get(), capsuleName, destroy);
    if (capsule) instance.release();
    return capsule;
  } catch (const std::exception& error) {
    if (!PyErr_Occurred()) PyErr_SetString(PyExc_RuntimeError, error.what());
    return nullptr;
  }
}

PyObject* solvePython(PyObject*, PyObject* args) {
  PyObject *capsule, *self, *qObject, *goals, *nullLower = Py_None, *nullUpper = Py_None;
  if (!PyArg_ParseTuple(args, "OOOO|OO", &capsule, &self, &qObject, &goals, &nullLower, &nullUpper))
    return nullptr;
  try {
    auto& instance = solver(capsule);
    Buffer output(qObject);
    Vector q = output.vector();
    if (output.view.readonly || q.size() != instance.model.nq || !q.allFinite())
      throw std::invalid_argument("Native IK output must be writable and match model size");
    Config config(self, goals, instance.model.nq);
    if (nullLower != Py_None || nullUpper != Py_None) {
      if (nullLower == Py_None || nullUpper == Py_None)
        throw std::invalid_argument("Both null-space joint-window bounds are required");
      config.nullLower = Buffer(nullLower).vector();
      config.nullUpper = Buffer(nullUpper).vector();
      if (config.nullLower.size() != q.size() || config.nullUpper.size() != q.size() ||
          !config.nullLower.allFinite() || !config.nullUpper.allFinite() ||
          (config.nullLower.array() > config.nullUpper.array()).any())
        throw std::invalid_argument("Invalid null-space joint window");
    }
    int iterations = 0;
    NullProgress progress;
    std::pair<double, double> errors;
    {
      ReleasedGIL release;
      errors = highPrecision(instance, q, config, iterations, progress);
    }
    if (!q.allFinite()) throw std::runtime_error("Native IK produced non-finite joint targets");
    Object status(Py_BuildValue("{s:s,s:i,s:d,s:d,s:d,s:d,s:d,s:O,s:O,s:d}",
        "state", progress.state, "iterations", progress.iterations,
        "initial_cost", progress.initialCost, "final_cost", progress.finalCost,
        "initial_elbow_cost", progress.initialElbow, "final_elbow_cost", progress.finalElbow,
        "projected_gradient", progress.gradient,
        "elbow_priority", Py_False, "rate_limited", Py_False,
        "task_fraction", progress.taskFraction));
    if (PyObject_SetAttrString(self, "_null_status", status.value) < 0)
      throw std::runtime_error("Cannot report null-space status");
    Eigen::Map<Vector>(static_cast<double*>(output.view.buf), q.size()) = q;
    return Py_BuildValue("(ddi)", errors.first, errors.second, iterations);
  } catch (const std::invalid_argument& error) {
    if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, error.what());
  } catch (const std::exception& error) {
    if (!PyErr_Occurred()) PyErr_SetString(PyExc_RuntimeError, error.what());
  }
  return nullptr;
}

PyMethodDef methods[] = {
  {"create", create, METH_VARARGS, "Create one persistent reduced Pinocchio model."},
  {"solve", solvePython, METH_VARARGS,
   "Solve the primary pose, then accept only precision-preserving strict-null preference moves."},
  {nullptr, nullptr, 0, nullptr}
};
PyModuleDef module = {PyModuleDef_HEAD_INIT, "_arm_ik_native", nullptr, -1, methods,
                      nullptr, nullptr, nullptr, nullptr};
}  // namespace
PyMODINIT_FUNC PyInit__arm_ik_native() { return PyModule_Create(&module); }
