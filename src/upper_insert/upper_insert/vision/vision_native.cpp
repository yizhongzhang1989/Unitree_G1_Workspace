#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Quad = std::array<cv::Point2f, 4>;
using Polygon = std::vector<cv::Point2d>;
using Template = std::array<unsigned char, 36>;
struct Marker { int id; std::array<Template, 4> rotations; };
struct Box { double x, y; int w, h; };
struct Best { int id; Quad full; cv::Rect roi; double score; };
struct Match { int id = -1; Quad quad{}; double score = -1.; };

class ReleasedGIL {
 public:
  ReleasedGIL() : state_(PyEval_SaveThread()) {}
  ~ReleasedGIL() { PyEval_RestoreThread(state_); }
  ReleasedGIL(const ReleasedGIL&) = delete;
  ReleasedGIL& operator=(const ReleasedGIL&) = delete;

 private:
  PyThreadState* state_;
};

struct Buffer {
  Py_buffer view{};
  explicit Buffer(PyObject* object) {
    if (PyObject_GetBuffer(object, &view, PyBUF_FORMAT | PyBUF_STRIDES) < 0)
      throw std::invalid_argument("Expected a NumPy-compatible buffer");
  }
  ~Buffer() { PyBuffer_Release(&view); }
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;
  cv::Mat image(int channels) const {
    if (view.itemsize != 1 || !view.format || std::string(view.format) != "B" ||
        view.ndim != (channels == 1 ? 2 : 3) ||
        view.strides[view.ndim - 1] != 1 || view.strides[0] <= 0 ||
        (channels == 3 && (view.shape[2] != 3 || view.strides[1] != 3)))
      throw std::invalid_argument("Expected a row-contiguous uint8 image");
    return cv::Mat(static_cast<int>(view.shape[0]), static_cast<int>(view.shape[1]),
                   CV_MAKETYPE(CV_8U, channels), view.buf,
                   static_cast<size_t>(view.strides[0]));
  }
  Quad quad() const {
    if (view.len / view.itemsize != 8 || !PyBuffer_IsContiguous(&view, 'C') ||
        !view.format || (std::string(view.format) != "f" && std::string(view.format) != "d"))
      throw std::invalid_argument("Expected four contiguous floating-point corners");
    Quad result{};
    for (int i = 0; i < 4; ++i) {
      if (view.itemsize == 4) {
        const auto* p = static_cast<const float*>(view.buf);
        result[i] = {p[2 * i], p[2 * i + 1]};
      } else {
        const auto* p = static_cast<const double*>(view.buf);
        result[i] = {static_cast<float>(p[2 * i]), static_cast<float>(p[2 * i + 1])};
      }
    }
    return result;
  }
};

int rounded(double x) { return static_cast<int>(std::nearbyint(x)); }
void fillSaturation(const cv::Mat& bgr, cv::Mat& output) {
  static const auto table = [] {
    std::array<unsigned char, 65536> values{};
    for (int v = 1; v < 256; ++v) {
      const int divisor = rounded(double(255 << 12) / v);
      for (int d = 0; d <= v; ++d) {
        const int sat = (d * divisor + 2048) >> 12;
        values[v * 256 + d] = static_cast<unsigned char>(
            (sat * std::min(v, 115) + 57) / 115);
      }
    }
    return values;
  }();
  for (int y = 0; y < bgr.rows; ++y) {
    const auto* source = bgr.ptr<unsigned char>(y);
    auto* dest = output.ptr<unsigned char>(y);
    for (int x = 0; x < bgr.cols; ++x) {
      const int b = source[3 * x], g = source[3 * x + 1], r = source[3 * x + 2];
      const int maximum = std::max({b, g, r}), minimum = std::min({b, g, r});
      dest[x] = table[maximum * 256 + maximum - minimum];
    }
  }
}

cv::Mat saturation(const cv::Mat& bgr) {
  cv::Mat output(bgr.size(), CV_8U);
  fillSaturation(bgr, output);
  return output;
}

float sum36(const std::array<float, 36>& a) {
  // Match NumPy's contiguous float32 pairwise reduction, including threshold ties.
  std::array<float, 8> r{};
  std::copy_n(a.begin(), 8, r.begin());
  for (int i = 8; i < 32; i += 8)
    for (int j = 0; j < 8; ++j) r[j] += a[i + j];
  float sum = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
  for (int i = 32; i < 36; ++i) sum += a[i];
  return sum;
}

Quad ordered(Quad q) {
  cv::Point2f center{};
  for (const auto& p : q) { center.x += p.x; center.y += p.y; }
  center *= .25f;
  std::stable_sort(q.begin(), q.end(), [&](const auto& a, const auto& b) {
    return std::atan2(a.y - center.y, a.x - center.x) <
           std::atan2(b.y - center.y, b.x - center.x);
  });
  int first = 0;
  for (int i = 1; i < 4; ++i)
    if (q[i].x + q[i].y < q[first].x + q[first].y) first = i;
  std::rotate(q.begin(), q.begin() + first, q.end());
  return q;
}

Match verify(const cv::Mat& gray, const cv::Mat& sat, Quad q,
             const std::vector<Marker>& markers) {
  q = ordered(q);
  std::array<double, 4> sides{};
  for (int i = 0; i < 4; ++i) {
    const auto d = q[(i + 1) % 4] - q[i];
    sides[i] = std::sqrt(d.x * d.x + d.y * d.y);
  }
  const double lo = *std::min_element(sides.begin(), sides.end());
  const double hi = *std::max_element(sides.begin(), sides.end());
  if (lo < 13. || hi / std::max(1., lo) > 3.6 ||
      std::max(sides[0], sides[2]) / std::max(1., std::min(sides[0], sides[2])) > 1.55 ||
      std::max(sides[1], sides[3]) / std::max(1., std::min(sides[1], sides[3])) > 1.55) return {};
  const auto d1 = q[2] - q[0], d2 = q[3] - q[1];
  const double diag1 = std::sqrt(d1.x * d1.x + d1.y * d1.y);
  const double diag2 = std::sqrt(d2.x * d2.x + d2.y * d2.y);
  if (std::max(diag1, diag2) / std::max(1., std::min(diag1, diag2)) > 2.2) return {};
  float xmin = q[0].x, xmax = q[0].x, ymin = q[0].y, ymax = q[0].y;
  for (const auto& p : q) {
    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
  }
  const int x0 = std::max(0, static_cast<int>(std::floor(xmin)) - 2);
  const int y0 = std::max(0, static_cast<int>(std::floor(ymin)) - 2);
  const int x1 = std::min(gray.cols, static_cast<int>(std::ceil(xmax)) + 3);
  const int y1 = std::min(gray.rows, static_cast<int>(std::ceil(ymax)) + 3);
  if (x1 - x0 < 16 || y1 - y0 < 16) return {};
  Quad local = q;
  for (auto& p : local) p -= cv::Point2f(static_cast<float>(x0), static_cast<float>(y0));
  const Quad dest{{{0.f, 0.f}, {60.f, 0.f}, {60.f, 60.f}, {0.f, 60.f}}};
  const auto transform = cv::getPerspectiveTransform(local.data(), dest.data());
  cv::Mat wg, ws;
  const cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
  cv::warpPerspective(gray(roi), wg, transform, {60, 60});
  cv::warpPerspective(sat(roi), ws, transform, {60, 60});
  std::array<float, 36> cellGray{};
  std::array<bool, 36> valid{}, confident{};
  std::vector<float> values;
  for (int cy = 0; cy < 6; ++cy) for (int cx = 0; cx < 6; ++cx) {
    std::array<unsigned char, 36> pg{}, ps{};
    std::array<float, 36> samples{}, squares{};
    int n = 0;
    for (int y = 2; y < 8; ++y) for (int x = 2; x < 8; ++x) {
      pg[n] = wg.at<unsigned char>(cy * 10 + y, cx * 10 + x);
      ps[n] = ws.at<unsigned char>(cy * 10 + y, cx * 10 + x);
      samples[n] = pg[n]; ++n;
    }
    const float mean = sum36(samples) / 36.f;
    for (int i = 0; i < 36; ++i) {
      const float d = samples[i] - mean;
      squares[i] = d * d;
    }
    const float deviation = std::sqrt(sum36(squares) / 36.f);
    std::nth_element(pg.begin(), pg.begin() + 18, pg.end());
    std::nth_element(ps.begin(), ps.begin() + 18, ps.end());
    const int index = cy * 6 + cx;
    cellGray[index] = pg[18];
    valid[index] = ps[18] < 75 && deviation < 48.f;
    if (valid[index]) values.push_back(cellGray[index]);
  }
  if (values.size() < 12) return {};
  std::sort(values.begin(), values.end());
  const float p15 = values[(15 * (values.size() - 1)) / 100];
  const float p85 = values[(85 * (values.size() - 1)) / 100];
  if (p85 - p15 < 32.f) return {};
  const double threshold = .5 * (p15 + p85);
  int total = 0, inner = 0, border = 0, black = 0;
  Template bits{};
  for (int i = 0; i < 36; ++i) {
    confident[i] = valid[i] && std::abs(cellGray[i] - threshold) > 10.;
    bits[i] = cellGray[i] > threshold;
    if (!confident[i]) continue;
    ++total;
    if (i / 6 > 0 && i / 6 < 5 && i % 6 > 0 && i % 6 < 5) ++inner;
    else { ++border; if (!bits[i]) ++black; }
  }
  if (total < 12 || inner < 4 || border < 6 || double(black) / border < .85) return {};
  Match best;
  for (const auto& marker : markers) for (int rot = 0; rot < 4; ++rot) {
    int matches = 0, innerMatches = 0;
    for (int i = 0; i < 36; ++i) if (confident[i] && bits[i] == marker.rotations[rot][i]) {
      ++matches;
      if (i / 6 > 0 && i / 6 < 5 && i % 6 > 0 && i % 6 < 5) ++innerMatches;
    }
    const double acc = double(matches) / total, innerAcc = double(innerMatches) / inner;
    if (acc < .88 || innerAcc < .82) continue;
    const double sideMean = (sides[0] + sides[1] + sides[2] + sides[3]) / 4.;
    const double score = innerAcc * 1.2 + acc + .015 * inner + .15 * std::min(1., sideMean / 120.);
    if (score > best.score) {
      best.id = marker.id; best.score = score; best.quad = q;
      std::rotate(best.quad.begin(), best.quad.begin() + rot, best.quad.end());
    }
  }
  return best;
}

std::vector<Best> recover(const cv::Mat& gray, const cv::Mat& suppliedSat,
                          const cv::Mat& frame, const std::vector<Quad>& found,
                          const std::vector<Marker>& markers, int passIndex) {
  const int height = gray.rows, width = gray.cols;
  const double res = std::max(.5, double(std::min(height, width)) / 720.);
  std::vector<cv::Point2f> centers;
  for (const auto& q : found) {
    cv::Point2f c{};
    for (const auto& p : q) { c.x += p.x; c.y += p.y; }
    centers.push_back(c * .25f);
  }
  double scale = 1.;
  cv::Mat smallGray, smallSat;
  if (std::min(height, width) >= 480) {
    scale = double(std::min(height, width)) / 360.;
    const int sw = std::max(160, rounded(width / scale));
    const int sh = std::max(120, rounded(height / scale));
    scale = double(width) / sw;
    cv::resize(gray, smallGray, {sw, sh}, 0, 0, cv::INTER_NEAREST);
    if (!suppliedSat.empty()) cv::resize(suppliedSat, smallSat, {sw, sh}, 0, 0, cv::INTER_NEAREST);
    else if (!frame.empty()) {
      cv::Mat smallFrame;
      cv::resize(frame, smallFrame, {sw, sh}, 0, 0, cv::INTER_NEAREST);
      smallSat = saturation(smallFrame);
    } else smallSat = cv::Mat::zeros(smallGray.size(), CV_8U);
  } else {
    smallGray = gray;
    smallSat = !suppliedSat.empty() ? suppliedSat :
        (!frame.empty() ? saturation(frame) : cv::Mat::zeros(gray.size(), CV_8U));
  }
  const int hs = smallGray.rows, ws = smallGray.cols;
  const double coarse = std::max(.5, double(std::min(hs, ws)) / 360.);
  cv::Mat fullSat = !suppliedSat.empty() ? suppliedSat : (scale == 1. ? smallSat : cv::Mat());
  std::vector<bool> cached(height, !fullSat.empty());
  auto fullROI = [&](const cv::Rect& roi) {
    if (fullSat.empty() && !frame.empty()) fullSat.create(height, width, CV_8U);
    if (!fullSat.empty() && !frame.empty()) {
      int row = roi.y;
      while (row < roi.y + roi.height) {
        if (cached[row]) { ++row; continue; }
        const int start = row;
        while (row < roi.y + roi.height && !cached[row]) cached[row++] = true;
        saturation(frame.rowRange(start, row)).copyTo(fullSat.rowRange(start, row));
      }
    }
    return fullSat.empty() ? cv::Mat(cv::Mat::zeros(roi.size(), CV_8U)) : fullSat(roi);
  };
  const int large = std::max(3, rounded(5.5 * coarse)) | 1;
  const int envSize = std::max(3, rounded(3. * coarse)) | 1;
  const int smooth = std::max(5, rounded(5. * coarse)) | 1;
  const auto env = cv::getStructuringElement(cv::MORPH_RECT, {envSize, envSize});
  auto mid = [&](const cv::Mat& image, double a, double b) {
    cv::Mat high, low, result;
    cv::dilate(image, high, env); cv::blur(high, high, {smooth, smooth});
    cv::erode(image, low, env); cv::blur(low, low, {smooth, smooth});
    cv::addWeighted(high, a, low, b, -2., result);
    return result;
  };
  const auto localMid = mid(smallGray, .52, .48);
  const auto achromatic = smallSat < 100;
  cv::Mat wideGray, wideMid, broadMid;
  std::vector<int> passes = passIndex >= 0 ?
      std::vector<int>{passIndex & 1, 2 + passIndex % 4} : std::vector<int>{0, 1, 2, 3, 4, 5};
  const double minArea = 36. * coarse * coarse, maxArea = .82 * hs * ws;
  const int minDim = static_cast<int>(std::max(5., double(rounded(6. * coarse))));
  const int maxDim = static_cast<int>(.92 * std::max(hs, ws));
  const double minSide = 13. * res, centerDistance = 35. * res;
  const int margin = std::max(24, rounded(24. * res));
  std::vector<Box> decoded, seen;
  std::vector<Best> best;
  for (const int pass : passes) {
    cv::Mat source = smallGray, threshold = localMid;
    int kernel = 1;
    if (pass == 1) {
      if (wideGray.empty()) {
        if (scale > 1.) {
          cv::Mat dilated;
          cv::dilate(gray, dilated, cv::Mat::ones(2, 2, CV_8U));
          cv::resize(dilated, wideGray, {ws, hs}, 0, 0, cv::INTER_NEAREST);
        } else wideGray = smallGray;
        wideMid = mid(wideGray, .48, .52);
      }
      source = wideGray; threshold = wideMid;
    } else if (pass == 2) {
      if (broadMid.empty()) {
        const int size = std::max(45, rounded(75. * coarse)) | 1;
        cv::Mat blurred;
        cv::blur(smallGray, blurred, {size, size});
        cv::addWeighted(blurred, 1., blurred, 0., -6., broadMid);
      }
      threshold = broadMid;
    } else if (pass == 3 || pass == 4) {
      threshold = cv::Mat(smallGray.size(), CV_8U, cv::Scalar(pass == 3 ? 130 : 108));
      if (pass == 4) kernel = 3;
    } else if (pass == 5) kernel = large;
    cv::Mat dark, closed;
    cv::bitwise_and(source < threshold, achromatic, dark);
    if (kernel > 1) cv::morphologyEx(dark, closed, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, {kernel, kernel}));
    else closed = dark;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closed, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
      const double signedArea = cv::contourArea(contour, true), area = -signedArea;
      if (signedArea >= 0. || !(minArea < area && area < maxArea)) continue;
      const auto box = cv::boundingRect(contour);
      const int x = box.x, y = box.y, bw = box.width, bh = box.height;
      if (bw > maxDim || bh > maxDim || bw < minDim || bh < minDim ||
          double(std::max(bw, bh)) / std::max(1., double(std::min(bw, bh))) > 3.6) continue;
      const double cx = (x + .5 * bw) * scale, cy = (y + .5 * bh) * scale;
      if (std::any_of(centers.begin(), centers.end(), [&](const auto& c) {
        return std::hypot(cx - c.x, cy - c.y) < centerDistance;
      })) continue;
      if (std::any_of(decoded.begin(), decoded.end(), [&](const auto& b) {
        return std::hypot(cx - b.x, cy - b.y) < 12. * res &&
               std::abs(bw - b.w) < 6 && std::abs(bh - b.h) < 6;
      })) continue;
      if (std::any_of(seen.begin(), seen.end(), [&](const auto& b) {
        return std::abs(cx - b.x) < 3.5 * res && std::abs(cy - b.y) < 3.5 * res &&
               std::abs(bw - b.w) <= 2 && std::abs(bh - b.h) <= 2;
      })) continue;
      seen.push_back({cx, cy, bw, bh});
      cv::Rect innerBox = box;
      const int iy = bh / 6, ix = bw / 6;
      if (bh - 2 * iy >= 3 && bw - 2 * ix >= 3)
        innerBox = {x + ix, y + iy, bw - 2 * ix, bh - 2 * iy};
      const auto innerGray = smallGray(innerBox), innerSat = smallSat(innerBox);
      double low, high;
      cv::minMaxLoc(innerGray, &low, &high);
      if (high - low < 28.) continue;
      // NumPy compares uint8 against the fractional threshold without rounding it.
      cv::Mat bright;
      cv::compare(innerGray, std::floor(.5 * (low + high)), bright, cv::CMP_GT);
      const double white = double(cv::countNonZero(bright & (innerSat < 75))) / innerGray.total();
      if (!(.08 < white && white < .85)) continue;
      std::vector<cv::Point2f> full;
      for (const auto& p : contour)
        full.emplace_back(static_cast<float>(p.x) * static_cast<float>(scale),
                          static_cast<float>(p.y) * static_cast<float>(scale));
      const double perimeter = cv::arcLength(full, true);
      const int bx0 = std::max(0, int(x * scale) - margin);
      const int by0 = std::max(0, int(y * scale) - margin);
      const int bx1 = std::min(width, int((x + bw) * scale) + margin);
      const int by1 = std::min(height, int((y + bh) * scale) + margin);
      const cv::Rect roi(bx0, by0, bx1 - bx0, by1 - by0);
      const cv::Point2f offset(static_cast<float>(bx0), static_cast<float>(by0));
      cv::Mat localSat;
      bool wasDecoded = false, expanded = false;
      std::vector<std::array<double, 3>> tested;
      auto candidate = [&](const Polygon& points, double span, double side,
                           double bonus, bool allowExpand) {
        double px = 0., py = 0.;
        for (const auto& p : points) { px += p.x; py += p.y; }
        px *= .25; py *= .25;
        if (std::any_of(centers.begin(), centers.end(), [&](const auto& c) {
          return std::hypot(px - c.x, py - c.y) < centerDistance;
        })) return false;
        if (std::any_of(tested.begin(), tested.end(), [&](const auto& t) {
          return std::hypot(px - t[0], py - t[1]) < 10. * res &&
                 std::abs(span - t[2]) < 14. * res;
        })) return false;
        tested.push_back({px, py, span});
        if (localSat.empty()) localSat = fullROI(roi);
        Quad q{};
        for (int i = 0; i < 4; ++i)
          q[i] = cv::Point2f(static_cast<float>(points[i].x),
                             static_cast<float>(points[i].y)) - offset;
        auto match = verify(gray(roi), localSat, q, markers);
        if (match.id < 0 && allowExpand && !expanded && side < 42. * res) {
          expanded = true;
          cv::Point2f mean{};
          for (const auto& p : q) { mean.x += p.x; mean.y += p.y; }
          mean *= .25f;
          const float amount = static_cast<float>(.65 * scale);
          for (auto& p : q) {
            p.x += static_cast<float>((p.x > mean.x) - (p.x < mean.x)) * amount;
            p.y += static_cast<float>((p.y > mean.y) - (p.y < mean.y)) * amount;
          }
          match = verify(gray(roi), localSat, q, markers);
        }
        if (match.id < 0) return false;
        match.score += bonus; wasDecoded = true;
        auto saved = std::find_if(best.begin(), best.end(),
                                 [&](const auto& b) { return b.id == match.id; });
        if (saved == best.end() || match.score > saved->score) {
          for (auto& p : match.quad) p += offset;
          const Best next{match.id, match.quad, roi, match.score};
          if (saved == best.end()) best.push_back(next); else *saved = next;
        }
        return true;
      };
      std::vector<cv::Point2f> hull;
      cv::convexHull(full, hull);
      const double hullPerimeter = cv::arcLength(hull, true);
      std::vector<Polygon> polygons;
      for (double epsilon : {.02, .025, .032, .045}) {
        std::vector<cv::Point2f> approximation;
        cv::approxPolyDP(epsilon == .045 ? hull : full, approximation,
                         epsilon * (epsilon == .045 ? hullPerimeter : perimeter), true);
        Polygon poly;
        for (const auto& p : approximation) poly.emplace_back(p.x, p.y);
        polygons.push_back(std::move(poly));
      }
      for (const auto& poly : polygons) {
        const int n = static_cast<int>(poly.size());
        if (n == 4) {
          const double ab = cv::norm(poly[1] - poly[0]), bc = cv::norm(poly[2] - poly[1]);
          if (ab >= minSide && bc >= minSide && std::max(ab, bc) / std::min(ab, bc) <= 3.6 &&
              candidate(poly, ab + bc, std::min(ab, bc), .25, true)) break;
          continue;
        }
        if (n < 5 || n > 8) continue;
        for (int i = 0; i < n; ++i) {
          const auto A = poly[i], B = poly[(i + 1) % n], C = poly[(i + 2) % n];
          const auto D = poly[(i + 3) % n], E = poly[(i + n - 1) % n];
          const auto ab = B - A, bc = C - B;
          const double lenAB = cv::norm(ab), lenBC = cv::norm(bc);
          if (lenAB < minSide || lenBC < minSide || std::max(lenAB, lenBC) / std::min(lenAB, lenBC) > 3.6) continue;
          const auto dc = D - C, ea = E - A;
          const bool l1 = cv::norm(dc) > 10. * res && dc.dot(ab) > .92718 * cv::norm(dc) * lenAB;
          const bool l2 = cv::norm(ea) > 10. * res && ea.dot(bc) > .92718 * cv::norm(ea) * lenBC;
          if (!l1 && !l2) continue;
          const auto a1 = C, a2 = l1 ? D : C + (A - B), b1 = A, b2 = l2 ? E : A + (C - B);
          const auto da = a2 - a1, db = b2 - b1, dp = a1 - b1;
          const cv::Point2d perpendicular(-da.y, da.x);
          const double denom = perpendicular.dot(db);
          if (std::abs(denom) < 1e-6) continue;
          const auto Q = b1 + (perpendicular.dot(dp) / denom) * db;
          candidate({A, B, C, Q}, lenAB + lenBC, std::min(lenAB, lenBC), 0., false);
        }
      }
      if (wasDecoded) decoded.push_back({cx, cy, bw, bh});
    }
    if (std::all_of(markers.begin(), markers.end(), [&](const auto& marker) {
      return std::any_of(best.begin(), best.end(), [&](const auto& b) {
        return b.id == marker.id && b.score > 2.50;
      });
    })) break;
  }
  return best;
}

PyObject* recoverPython(PyObject*, PyObject* args) {
  PyObject *grayObject, *satObject, *frameObject, *foundObject, *idsObject, *templatesObject;
  int passIndex;
  if (!PyArg_ParseTuple(args, "OOOOOOi", &grayObject, &satObject, &frameObject,
                        &foundObject, &idsObject, &templatesObject, &passIndex)) return nullptr;
  try {
    if (!PyDict_Check(foundObject) || !PyDict_Check(templatesObject) || !PyTuple_Check(idsObject))
      throw std::invalid_argument("Expected corner/template dictionaries and a tuple of target IDs");
    Buffer grayBuffer(grayObject);
    const cv::Mat gray = grayBuffer.image(1);
    cv::Mat sat, frame;
    // Keep exports alive while the GIL is released.
    std::unique_ptr<Buffer> satBuffer, frameBuffer;
    if (satObject != Py_None) {
      satBuffer = std::make_unique<Buffer>(satObject); sat = satBuffer->image(1);
      if (sat.size() != gray.size()) throw std::invalid_argument("Saturation shape differs from gray");
    }
    if (frameObject != Py_None) {
      frameBuffer = std::make_unique<Buffer>(frameObject); frame = frameBuffer->image(3);
      if (frame.size() != gray.size()) throw std::invalid_argument("BGR shape differs from gray");
    }
    std::vector<Quad> found;
    std::vector<Marker> markers;
    PyObject *key, *value;
    Py_ssize_t position = 0;
    while (PyDict_Next(foundObject, &position, &key, &value)) found.push_back(Buffer(value).quad());
    for (Py_ssize_t i = 0; i < PyTuple_GET_SIZE(idsObject); ++i) {
      auto* idObject = PyTuple_GET_ITEM(idsObject, i);
      const long id = PyLong_AsLong(idObject);
      if (PyErr_Occurred()) return nullptr;
      if (PyDict_Contains(foundObject, idObject) == 1) continue;
      PyObject* rotations = PyDict_GetItemWithError(templatesObject, idObject);
      if (!rotations || !PyList_Check(rotations) || PyList_GET_SIZE(rotations) != 4)
        throw std::invalid_argument("Target ID has no four rotation templates");
      Marker marker{}; marker.id = static_cast<int>(id);
      for (int r = 0; r < 4; ++r) {
        Buffer buffer(PyList_GET_ITEM(rotations, r));
        if (buffer.view.ndim != 2 || buffer.view.shape[0] != 6 || buffer.view.shape[1] != 6 ||
            buffer.view.itemsize != 1 || !buffer.view.format || std::string(buffer.view.format) != "B")
          throw std::invalid_argument("Expected uint8 6x6 marker templates");
        const auto* data = static_cast<const unsigned char*>(buffer.view.buf);
        for (int y = 0; y < 6; ++y) for (int x = 0; x < 6; ++x)
          marker.rotations[r][y * 6 + x] = data[y * buffer.view.strides[0] + x * buffer.view.strides[1]];
      }
      markers.push_back(marker);
    }
    std::vector<Best> result;
    if (!markers.empty()) {
      ReleasedGIL release;
      result = recover(gray, sat, frame, found, markers, passIndex);
    }
    PyObject* output = PyList_New(static_cast<Py_ssize_t>(result.size()));
    if (!output) return nullptr;
    for (size_t i = 0; i < result.size(); ++i) {
      const auto& item = result[i];
      PyObject* corners = PyList_New(4);
      if (!corners) { Py_DECREF(output); return nullptr; }
      for (int j = 0; j < 4; ++j) {
        auto* point = Py_BuildValue("(dd)", double(item.full[j].x), double(item.full[j].y));
        if (!point) { Py_DECREF(corners); Py_DECREF(output); return nullptr; }
        PyList_SET_ITEM(corners, j, point);
      }
      auto* entry = Py_BuildValue("(iNiiii)", item.id, corners, item.roi.x, item.roi.y,
                                  item.roi.x + item.roi.width, item.roi.y + item.roi.height);
      if (!entry) { Py_DECREF(output); return nullptr; }
      PyList_SET_ITEM(output, static_cast<Py_ssize_t>(i), entry);
    }
    return output;
  } catch (const std::invalid_argument& error) {
    if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, error.what());
  } catch (const std::exception& error) {
    PyErr_SetString(PyExc_RuntimeError, error.what());
  }
  return nullptr;
}

PyObject* saturationPython(PyObject*, PyObject* args) {
  PyObject *sourceObject, *outputObject;
  if (!PyArg_ParseTuple(args, "OO", &sourceObject, &outputObject)) return nullptr;
  try {
    Buffer source(sourceObject), output(outputObject);
    const auto bgr = source.image(3);
    auto destination = output.image(1);
    if (output.view.readonly || destination.size() != bgr.size())
      throw std::invalid_argument("Saturation output must be writable and match image shape");
    {
      ReleasedGIL release;
      fillSaturation(bgr, destination);
    }
    Py_RETURN_NONE;
  } catch (const std::invalid_argument& error) {
    if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, error.what());
  } catch (const std::exception& error) {
    PyErr_SetString(PyExc_RuntimeError, error.what());
  }
  return nullptr;
}

PyMethodDef methods[] = {
  {"recover", recoverPython, METH_VARARGS, "Equivalent native occluded-marker candidate search."},
  {"saturation", saturationPython, METH_VARARGS, "Fill exact dark-weighted OpenCV saturation."},
  {nullptr, nullptr, 0, nullptr}
};
PyModuleDef module = {PyModuleDef_HEAD_INIT, "_vision_native", nullptr, -1, methods,
                      nullptr, nullptr, nullptr, nullptr};
}  // namespace

PyMODINIT_FUNC PyInit__vision_native() { return PyModule_Create(&module); }
