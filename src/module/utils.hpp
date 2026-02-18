#pragma once
#include <boost/callable_traits.hpp>
#include <boost/range/algorithm/find.hpp>
#include <boost/range/end.hpp>
#include <memory>
#include <string>
#include <tuple>

namespace utils {

// Generic RAII cleanup wrapper for C-style cleanup functions
// Usage: CleanupPtr<cleanup_function> ptr(resource);
// Example: CleanupPtr<curl_easy_cleanup> curl(curl_easy_init());
template <auto cleanup_fp> struct Cleanup {
  using pointer_type = std::tuple_element_t<
      0, boost::callable_traits::args_t<decltype(cleanup_fp)>>;
  using value_type = std::remove_pointer_t<pointer_type>;

  void operator()(pointer_type p) {
    if (p != nullptr) {
      cleanup_fp(p);
    }
  }
};

template <auto cleanup_fp>
using CleanupPtr = std::unique_ptr<typename Cleanup<cleanup_fp>::value_type,
                                   Cleanup<cleanup_fp>>;

template <typename ElementT, typename ContainerT>
inline bool contains(ElementT const &element, ContainerT const &container) {
  return boost::range::find(container, element) != boost::end(container);
}

// Helper function to create a safe view from RealSense frame data
template <typename FrameT>
auto createFrameDataView(const FrameT &frame, const std::string &frame_type) {
  std::uint8_t *data = static_cast<std::uint8_t *>(frame.get_data());
  uint32_t dataSize = frame.get_data_size();

  if (data == nullptr || dataSize == 0) {
    throw std::runtime_error("[get_point_cloud] " + frame_type +
                             " data is null");
  }

  return boost::make_iterator_range(data, data + dataSize);
}

} // namespace utils
