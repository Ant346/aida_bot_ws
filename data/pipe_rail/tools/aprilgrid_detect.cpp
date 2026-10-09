// Детектор доски калибровки: AprilTag 36h11 с чёрной рамкой в 2 бита,
// как у Kalibr (blackTagBorder). OpenCV DICT_APRILTAG_36h11 ждёт 1 бит и
// эту доску не читает.
#include "apriltags/TagDetector.h"
#include "apriltags/Tag36h11.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <vector>

namespace {
AprilTags::TagDetector& detector() {
  static AprilTags::TagDetector det(AprilTags::tagCodes36h11, 2);
  return det;
}
}  // namespace

// Углы p[0..3]: против часовой от нижнего левого угла тега в системе доски
// (x вправо, y вверх), как у Kalibr: BL, BR, TR, TL.
extern "C" int detect_april36h11_corners(const unsigned char* gray, int width, int height,
                                         int step, int* out_ids, float* out_xy, int max_tags) {
  if (gray == nullptr || out_ids == nullptr || out_xy == nullptr || width < 8 || height < 8 ||
      max_tags <= 0) {
    return 0;
  }
  cv::Mat view(height, width, CV_8UC1, const_cast<unsigned char*>(gray),
               static_cast<size_t>(step));
  cv::Mat owned;
  view.copyTo(owned);
  const auto tags = detector().extractTags(owned);
  int n = 0;
  for (const auto& tag : tags) {
    if (!tag.good || n >= max_tags) {
      continue;
    }
    out_ids[n] = tag.id;
    for (int j = 0; j < 4; ++j) {
      out_xy[n * 8 + j * 2] = tag.p[j].first;
      out_xy[n * 8 + j * 2 + 1] = tag.p[j].second;
    }
    ++n;
  }
  return n;
}

extern "C" int detect_april36h11(const unsigned char* gray, int width, int height,
                                 int step, int* out_ids, int max_ids) {
  if (gray == nullptr || width < 8 || height < 8 || max_ids <= 0) {
    return 0;
  }
  cv::Mat view(height, width, CV_8UC1, const_cast<unsigned char*>(gray),
               static_cast<size_t>(step));
  cv::Mat owned;
  view.copyTo(owned);
  const auto tags = detector().extractTags(owned);
  std::vector<int> ids;
  ids.reserve(tags.size());
  for (const auto& tag : tags) {
    ids.push_back(tag.id);
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  const int n = std::min(static_cast<int>(ids.size()), max_ids);
  for (int i = 0; i < n; ++i) {
    out_ids[i] = ids[i];
  }
  return n;
}
