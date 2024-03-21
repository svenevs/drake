#pragma once
#include "drake/geometry/render_vtk/internal_render_engine_vtk.h"

#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

#include <Eigen/Dense>
#include <fmt/format.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkOpenGLTexture.h>  // vtkRenderingOpenGL2
#include <vtkPNGReader.h>      // vtkIOImage
#include <vtkProperty.h>       // vtkRenderingCore

#include "drake/common/drake_copyable.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_io.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"
#include "drake/visualization/colorize_depth_image.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

// Use friend access to grab actors.
class RenderEngineVtkTester {
 public:
  // This returns the first color actor associated with the given `id` (if there
  // are multiple actors for the geometry).
  static vtkActor* GetColorActor(const RenderEngineVtk& renderer,
                                 GeometryId id);

  // Return all of the colors actors associated with the given geometry id.
  static std::vector<vtkActor*> GetColorActors(const RenderEngineVtk& renderer,
                                               GeometryId id);
};


using Eigen::AngleAxisd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using math::RotationMatrixd;
using render::ColorRenderCamera;
using render::DepthRange;
using render::DepthRenderCamera;
using render::LightParameter;
using render::RenderCameraCore;
using render::RenderEngine;
using render::RenderLabel;
using std::make_unique;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using systems::sensors::CameraInfo;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageGrey8U;
using systems::sensors::ImageIo;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;
using visualization::ColorizeDepthImage;

// Default camera properties.
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr double kClipNear = 0.1;
constexpr double kClipFar = 100.0;
constexpr double kZNear = 0.5;
constexpr double kZFar = 5.;
constexpr double kFovY = M_PI_4;

// The following tolerance is used due to a precision difference between Ubuntu
// Linux and Mac OSX.
constexpr double kColorPixelTolerance = 1.001;
// NOTE: The depth tolerance is this large mostly due to the combination of
// several factors:
//   - the sphere test (sphere against terrain)
//   - The even-valued window dimensions
//   - the tests against various camera properties
// The explanation is as follows. The distance to the sphere is only exactly
// 2 at the point of the sphere directly underneath the camera (the sphere's
// "peak"). However, with an even-valued window dimension, we never really
// sample that point. We sample the center of pixels all evenly arrayed around
// that point. So, that introduces some error. This error is further increased
// in ellipsoid tests when sampling around the elongated ends. As the image gets
// *smaller* the pixels get bigger and so the distance away from the peak center
// increases, which, in turn, increase the measured distance for the fragment.
// This tolerance accounts for the test case where one image has pixels that are
// *4X* larger (in area) than the default image size.
constexpr double kDepthTolerance = 1e-3;

// An RGBA color denoted using four `int`s, offering nice conversion
// constructors and operators to ease the pain of creating test values.
struct TestColor {
  // Constructs from three or four `int`s.
  constexpr TestColor(int r_in, int g_in, int b_in, int a_in = 255)
      : r(r_in), g(g_in), b(b_in), a(a_in) {}

  // Constructs from an array of four bytes.
  explicit TestColor(const uint8_t* p) : r(p[0]), g(p[1]), b(p[2]), a(p[3]) {}

  // Constructs from a vector of four doubles (each in the range [0..1]).
  explicit TestColor(const Vector4d& norm_color)
      : r(static_cast<int>(norm_color(0) * 255)),
        g(static_cast<int>(norm_color(1) * 255)),
        b(static_cast<int>(norm_color(2) * 255)),
        a(static_cast<int>(norm_color(3) * 255)) {}

  // This implicit conversion is extremely convenient.
  // NOLINTNEXTLINE(runtime/explicit)
  TestColor(const Rgba& rgba) : TestColor(rgba.rgba()) {}

  // Converts back to an Rgba.
  Rgba ToRgba() const {
    return Rgba(r / 255.0, g / 255.0, b / 255.0, a / 255.0);
  }

  bool operator==(const TestColor& c) const {
    return r == c.r && g == c.g && b == c.b && a == c.a;
  }

  bool operator!=(const TestColor& c) const { return !(*this == c); }

  int r{0};
  int g{0};
  int b{0};
  int a{255};
};

std::ostream& operator<<(std::ostream& out, const TestColor& c);

// Background (sky) and terrain colors.
constexpr TestColor kBgColor{254, 127, 0};

// We need a color that we can see the effects of illumination on.
constexpr TestColor kTerrainColor{127, 127, 153};

// box.png contains a single pixel with the color (4, 241, 33). If the image
// changes, the expected color would likewise have to change.
constexpr TestColor kTextureColor{4, 241, 33};

// Provide a default visual color for these tests -- it is intended to be
// different from the default color of the VTK render engine.
constexpr TestColor kDefaultVisualColor{229, 229, 229};

constexpr float kDefaultDistance{3.f};

const RenderLabel kDefaultLabel{13531};

// Values to be used with the "centered shape" tests.
// The amount inset from the edge of the images to *still* expect ground plane
// values.
static constexpr int kInset{10};

// Holds `(x, y)` indices of the screen coordinate system where the ranges of
// `x` and `y` are [0, image_width) and [0, image_height) respectively.
struct ScreenCoord {
  int x{};
  int y{};
};

std::ostream& operator<<(std::ostream& out, const ScreenCoord& c);

// Tests color within tolerance.
bool IsColorNear(const TestColor& expected, const TestColor& tested,
                 double tolerance = kColorPixelTolerance);

// Tests that the color in the given `image` located at screen coordinate `p`
// matches the `expected` color to within the given `tolerance`.
::testing::AssertionResult CompareColor(
    const TestColor& expected, const ImageRgba8U& image, const ScreenCoord& p,
    double tolerance = kColorPixelTolerance);

// This test suite facilitates a test with a ground plane and floating shape.
// The camera is positioned above the shape looking straight down. All
// of the images produced from these tests should have the following properties:
//   1. The shape is centered.
//   2. The ground plane fills the whole background (i.e., no background color
//      should be visible), except for noted exceptions.
//   3. The rendered shape should be smaller than the full image size with a
//      minimum number of pixels of ground plane between the shape and the edge
//      of the image. The minimum number of pixels is defined by kInset.
//
// The tests examine the rendered images and tests some discrete pixels, mapped
// to the image size (w, h):
//   1. A "center" pixel (x, y) such that x = w / 2 and y = h / 2.
//   2. Border pixels (xᵢ, yᵢ) which are pixels inset from each corner:
//      e.g., (i, i), (w - i - 1, i), (w - i - 1, h - i - 1), (i, h - i - 1),
//      for an inset value of `i` pixels.
class RenderEngineVtkTest : public ::testing::Test {
 public:
  RenderEngineVtkTest();

 protected:
  // Method to allow the normal case (render with the built-in renderer against
  // the default camera) to the member images with default window visibility.
  // This interface allows that to be completely reconfigured by the calling
  // test.
  void Render(RenderEngineVtk* renderer = nullptr,
              const DepthRenderCamera* camera_in = nullptr,
              ImageRgba8U* color_out = nullptr,
              ImageDepth32F* depth_out = nullptr,
              ImageLabel16I* label_out = nullptr);

  // Confirms that all pixels in the member color image have the same value.
  void VerifyUniformColor(const TestColor& pixel,
                          const ImageRgba8U* color = nullptr);

  // Confirms that all pixels in the member label image have the same value.
  void VerifyUniformLabel(int16_t value, const ImageLabel16I* label = nullptr);

  // Confirms that all pixels in the member depth image have the same value.
  void VerifyUniformDepth(float value, const ImageDepth32F* depth = nullptr);

  // Compute the set of outliers for a given set of camera properties.
  static std::vector<ScreenCoord> GetOutliers(const CameraInfo& intrinsics);

  // Compute the inlier for the given set of camera properties.
  static ScreenCoord GetInlier(const CameraInfo& intrinsics);

  // Tests that the depth value in the given `image` at the given `coord` is
  // the expected depth to within a tolerance. Handles the special case where
  // the expected distance is infinity.
  static ::testing::AssertionResult IsExpectedDepth(const ImageDepth32F& image,
                                                    const ScreenCoord& coord,
                                                    float expected_depth,
                                                    float tolerance);

  // Verifies the "outlier" pixels for the given camera belong to the ground
  // plane. If images are provided, the given images will be tested, otherwise
  // the member images will be tested.
  void VerifyOutliers(const RenderEngineVtk& renderer,
                      const DepthRenderCamera& camera, const char* name,
                      const ImageRgba8U* color_in = nullptr,
                      const ImageDepth32F* depth_in = nullptr,
                      const ImageLabel16I* label_in = nullptr) const;

  void SetUp() override;

  // Tests that don't instantiate their own renderers should invoke this.
  void Init(const RigidTransformd& X_WR, bool add_terrain = false);

  // Tests that instantiate their own renderers can initialize their renderers
  // with this method.
  void InitializeRenderer(const RigidTransformd& X_WR, bool add_terrain,
                          RenderEngineVtk* engine);

  // Creates a simple perception properties set for fixed, known results. The
  // material color can be modified by setting default_color_ prior to invoking
  // this method.
  PerceptionProperties simple_material(bool use_texture = false) const;

  // Resets all expected values to the initial, default values.
  void ResetExpectations();

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void PopulateSphereTest(RenderEngineVtk* renderer, bool use_texture = false);

  void PopulateSimpleBoxTest(RenderEngineVtk* renderer);

  // Performs the work to test the rendering with a shape centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compatible shape and camera configuration (e.g., PopulateSphereTest()).
  void PerformCenterShapeTest(RenderEngineVtk* renderer, const char* name,
                              const DepthRenderCamera* camera = nullptr);

  void VerifyCenterShapeTest(const RenderEngineVtk& renderer, const char* name,
                             const DepthRenderCamera& camera,
                             const ImageRgba8U& color,
                             const ImageDepth32F& depth,
                             const ImageLabel16I& label) const;

  TestColor expected_color_{kDefaultVisualColor};
  TestColor expected_outlier_color_{kDefaultVisualColor};
  float expected_outlier_depth_{3.f};
  float expected_object_depth_{2.f};
  RenderLabel expected_label_;
  RenderLabel expected_outlier_label_{RenderLabel::kDontCare};
  TestColor default_color_{kDefaultVisualColor};

  // We store a reference depth camera; we can always derive a color camera
  // from it; they have the same intrinsics and we grab the global
  // FLAGS_show_window.
  const DepthRenderCamera depth_camera_{
      {"unused", {kWidth, kHeight, kFovY}, {kClipNear, kClipFar}, {}},
      {kZNear, kZFar}};

  ImageRgba8U color_;
  ImageDepth32F depth_;
  ImageLabel16I label_;
  RigidTransformd X_WC_;
  GeometryId geometry_id_;

  // The pose of the sphere created in PopulateSphereTest().
  unordered_map<GeometryId, RigidTransformd> X_WV_;

  unique_ptr<RenderEngineVtk> renderer_;
};

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
