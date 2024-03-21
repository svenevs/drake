#define SUPERTEST_IMPL
#include "drake/geometry/render_vtk/internal_render_engine_vtk_tester.h"

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

vtkActor* RenderEngineVtkTester::GetColorActor(const RenderEngineVtk& renderer,
                                 GeometryId id) {
    // First 0 is the color index, second is the first actor.
    vtkActor* actor = renderer.props_.at(id).at(0).parts.at(0).actor.Get();
    DRAKE_DEMAND(actor != nullptr);
    return actor;
  }

  // Return all of the colors actors associated with the given geometry id.
std::vector<vtkActor*> RenderEngineVtkTester::GetColorActors(const RenderEngineVtk& renderer,
                                               GeometryId id) {
    const auto& color_prop = renderer.props_.at(id).at(0);
    std::vector<vtkActor*> actors;
    for (const auto& part : color_prop.parts) {
      actors.push_back(part.actor.Get());
    }
    return actors;
}

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


std::ostream& operator<<(std::ostream& out, const TestColor& c) {
  out << "(" << c.r << ", " << c.g << ", " << c.b << ", " << c.a << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, const ScreenCoord& c) {
  out << "(" << c.x << ", " << c.y << ")";
  return out;
}

// Tests color within tolerance.
bool IsColorNear(const TestColor& expected, const TestColor& tested,
                 double tolerance) {
  using std::abs;
  return (abs(expected.r - tested.r) < tolerance &&
          abs(expected.g - tested.g) < tolerance &&
          abs(expected.b - tested.b) < tolerance &&
          abs(expected.a - tested.a) < tolerance);
}

// Tests that the color in the given `image` located at screen coordinate `p`
// matches the `expected` color to within the given `tolerance`.
::testing::AssertionResult CompareColor(
    const TestColor& expected, const ImageRgba8U& image, const ScreenCoord& p,
    double tolerance) {
  TestColor tested(image.at(p.x, p.y));
  if (IsColorNear(expected, tested, tolerance)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Expected: " << expected << " at " << p << ", tested: " << tested
         << " with tolerance: " << tolerance;
}


RenderEngineVtkTest::RenderEngineVtkTest()
      : color_(kWidth, kHeight),
        depth_(kWidth, kHeight),
        label_(kWidth, kHeight),
        // Looking straight down from kDefaultDistance meters above the ground.
        X_WC_(RotationMatrixd{AngleAxisd(M_PI, Vector3d::UnitY()) *
                              AngleAxisd(-M_PI_2, Vector3d::UnitZ())},
              {0, 0, kDefaultDistance}),
        geometry_id_(GeometryId::get_new_id()) {}

  void RenderEngineVtkTest::Render(RenderEngineVtk* renderer,
              const DepthRenderCamera* camera_in,
              ImageRgba8U* color_out,
              ImageDepth32F* depth_out,
              ImageLabel16I* label_out) {
    if (!renderer) renderer = renderer_.get();
    const DepthRenderCamera& depth_camera =
        camera_in ? *camera_in : depth_camera_;
    const ColorRenderCamera color_camera(depth_camera.core(),
                                         // TODO(svenevs): sharing flags ...
                                         /* FLAGS_show_window */ true);
    ImageRgba8U* color = color_out ? color_out : &color_;
    ImageDepth32F* depth = depth_out ? depth_out : &depth_;
    ImageLabel16I* label = label_out ? label_out : &label_;
    EXPECT_NO_THROW(renderer->RenderDepthImage(depth_camera, depth));
    EXPECT_NO_THROW(renderer->RenderLabelImage(color_camera, label));
    EXPECT_NO_THROW(renderer->RenderColorImage(color_camera, color));
    // TODO(svenevs): sharing flags seems challenging...
    // if (FLAGS_sleep > 0) sleep(FLAGS_sleep);
  }

  // Confirms that all pixels in the member color image have the same value.
  void RenderEngineVtkTest::VerifyUniformColor(const TestColor& pixel,
                          const ImageRgba8U* color) {
    if (color == nullptr) color = &color_;
    for (int y = 0; y < color->height(); ++y) {
      for (int x = 0; x < color->width(); ++x) {
        ASSERT_TRUE(CompareColor(pixel, *color, ScreenCoord{x, y}));
      }
    }
  }

  // Confirms that all pixels in the member label image have the same value.
  void RenderEngineVtkTest::VerifyUniformLabel(int16_t value, const ImageLabel16I* label) {
    if (label == nullptr) label = &label_;
    for (int y = 0; y < label->height(); ++y) {
      for (int x = 0; x < label->width(); ++x) {
        ASSERT_EQ(label->at(x, y)[0], value)
            << "At pixel (" << x << ", " << y << ")";
      }
    }
  }

  // Confirms that all pixels in the member depth image have the same value.
  void RenderEngineVtkTest::VerifyUniformDepth(float value, const ImageDepth32F* depth) {
    if (depth == nullptr) depth = &depth_;
    if (value == std::numeric_limits<float>::infinity()) {
      for (int y = 0; y < depth->height(); ++y) {
        for (int x = 0; x < depth->width(); ++x) {
          ASSERT_EQ(depth->at(x, y)[0], value);
        }
      }
    } else {
      for (int y = 0; y < depth->height(); ++y) {
        for (int x = 0; x < depth->width(); ++x) {
          ASSERT_NEAR(depth->at(x, y)[0], value, kDepthTolerance);
        }
      }
    }
  }

  // Compute the set of outliers for a given set of camera properties.
  std::vector<ScreenCoord> RenderEngineVtkTest::GetOutliers(const CameraInfo& intrinsics) {
    return std::vector<ScreenCoord>{
        {kInset, kInset},
        {kInset, intrinsics.height() - kInset - 1},
        {intrinsics.width() - kInset - 1, intrinsics.height() - kInset - 1},
        {intrinsics.width() - kInset - 1, kInset}};
  }

  // Compute the inlier for the given set of camera properties.
  ScreenCoord RenderEngineVtkTest::GetInlier(const CameraInfo& intrinsics) {
    return {intrinsics.width() / 2, intrinsics.height() / 2};
  }

  // Tests that the depth value in the given `image` at the given `coord` is
  // the expected depth to within a tolerance. Handles the special case where
  // the expected distance is infinity.
  ::testing::AssertionResult RenderEngineVtkTest::IsExpectedDepth(const ImageDepth32F& image,
                                                    const ScreenCoord& coord,
                                                    float expected_depth,
                                                    float tolerance) {
    const float actual_depth = image.at(coord.x, coord.y)[0];
    if (expected_depth == std::numeric_limits<float>::infinity()) {
      if (actual_depth == expected_depth) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
               << "Expected depth at " << coord
               << " to be infinity. Found: " << actual_depth;
      }
    } else {
      float delta = std::abs(expected_depth - actual_depth);
      if (delta <= tolerance) {
        return ::testing::AssertionSuccess();
      } else {
        return ::testing::AssertionFailure()
               << "Expected depth at " << coord << " to be " << expected_depth
               << ". Found " << actual_depth << ". Difference " << delta
               << " is greater than tolerance " << tolerance;
      }
    }
  }

  // Verifies the "outlier" pixels for the given camera belong to the ground
  // plane. If images are provided, the given images will be tested, otherwise
  // the member images will be tested.
  void RenderEngineVtkTest::VerifyOutliers(const RenderEngineVtk& renderer,
                      const DepthRenderCamera& camera, const char* name,
                      const ImageRgba8U* color_in,
                      const ImageDepth32F* depth_in,
                      const ImageLabel16I* label_in ) const {
    const ImageRgba8U& color = color_in ? *color_in : color_;
    const ImageDepth32F& depth = depth_in ? *depth_in : depth_;
    const ImageLabel16I& label = label_in ? *label_in : label_;

    for (const auto& screen_coord : GetOutliers(camera.core().intrinsics())) {
      const int x = screen_coord.x;
      const int y = screen_coord.y;
      EXPECT_TRUE(CompareColor(expected_outlier_color_, color, screen_coord))
          << "Color at: " << screen_coord << " for test: " << name;
      EXPECT_TRUE(IsExpectedDepth(depth, screen_coord, expected_outlier_depth_,
                                  kDepthTolerance))
          << "Depth at: " << screen_coord << " for test: " << name;
      EXPECT_EQ(label.at(x, y)[0], expected_outlier_label_)
          << "Label at: " << screen_coord << " for test: " << name;
    }
  }

  void RenderEngineVtkTest::SetUp() { ResetExpectations(); }

  // Tests that don't instantiate their own renderers should invoke this.
  void RenderEngineVtkTest::Init(const RigidTransformd& X_WR, bool add_terrain) {
    const Vector3d bg_rgb{kBgColor.r / 255., kBgColor.g / 255.,
                          kBgColor.b / 255.};
    RenderEngineVtkParams params{{}, bg_rgb};
    renderer_ = make_unique<RenderEngineVtk>(params);
    InitializeRenderer(X_WR, add_terrain, renderer_.get());
    // Ensure that we truly have a non-default color.
    EXPECT_FALSE(IsColorNear(kDefaultVisualColor,
                             TestColor(renderer_->default_diffuse())));
  }

  // Tests that instantiate their own renderers can initialize their renderers
  // with this method.
  void RenderEngineVtkTest::InitializeRenderer(const RigidTransformd& X_WR, bool add_terrain,
                          RenderEngineVtk* engine) {
    engine->UpdateViewpoint(X_WR);

    if (add_terrain) {
      PerceptionProperties material;
      material.AddProperty("label", "id", RenderLabel::kDontCare);
      material.AddProperty("phong", "diffuse", kTerrainColor.ToRgba());
      engine->RegisterVisual(GeometryId::get_new_id(), HalfSpace(), material,
                             RigidTransformd::Identity(),
                             false /* needs update */);
    }
  }

  // Creates a simple perception properties set for fixed, known results. The
  // material color can be modified by setting default_color_ prior to invoking
  // this method.
  PerceptionProperties RenderEngineVtkTest::simple_material(bool use_texture) const {
    PerceptionProperties material;
    material.AddProperty("label", "id", expected_label_);
    if (use_texture) {
      // The simple material's texture should always reproduce the texture
      // perfectly -- so the diffuse color must be opaque white.
      material.AddProperty("phong", "diffuse", Rgba(1, 1, 1));

      material.AddProperty(
          "phong", "diffuse_map",
          FindResourceOrThrow("drake/geometry/render/test/meshes/box.png"));
    } else {
      const Rgba default_color(
          default_color_.r / 255.0, default_color_.g / 255.0,
          default_color_.b / 255.0, default_color_.a / 255.0);
      material.AddProperty("phong", "diffuse", default_color);
    }
    return material;
  }

  // Resets all expected values to the initial, default values.
  void RenderEngineVtkTest::ResetExpectations() {
    expected_color_ = kDefaultVisualColor;
    expected_outlier_color_ = kTerrainColor;
    expected_outlier_depth_ = 3.f;
    expected_object_depth_ = 2.f;
    // We expect each test to explicitly set this.
    expected_label_ = RenderLabel();
    expected_outlier_label_ = RenderLabel::kDontCare;
  }

  // Populates the given renderer with the sphere required for
  // PerformCenterShapeTest().
  void RenderEngineVtkTest::PopulateSphereTest(RenderEngineVtk* renderer, bool use_texture) {
    Sphere sphere{0.5};
    expected_label_ = RenderLabel(12345);  // an arbitrary value.
    renderer->RegisterVisual(geometry_id_, sphere, simple_material(use_texture),
                             RigidTransformd::Identity(),
                             true /* needs update */);
    RigidTransformd X_WV{Vector3d{0, 0, 0.5}};
    X_WV_.clear();
    X_WV_.insert({geometry_id_, X_WV});
    renderer->UpdatePoses(X_WV_);
  }

  void RenderEngineVtkTest::PopulateSimpleBoxTest(RenderEngineVtk* renderer) {
    // Simple cube.
    const double length = 1.0;
    const Box box = Box::MakeCube(length);
    expected_label_ = kDefaultLabel;
    const GeometryId id = GeometryId::get_new_id();
    PerceptionProperties props = simple_material(false);
    renderer->RegisterVisual(id, box, props, RigidTransformd::Identity(),
                             true /* needs update */);
    // Leave the box centered on the xy plane, but raise it up for the expected
    // depth in the camera (distance from eye to near surface):
    //      expected depth = p_WC.z - length / 2 - p_WV.z;
    const double p_WVo_z =
        X_WC_.translation()(2) - length / 2 - expected_object_depth_;
    RigidTransformd X_WV{Vector3d{0, 0, p_WVo_z}};
    renderer->UpdatePoses(
        unordered_map<GeometryId, RigidTransformd>{{id, X_WV}});
    expected_color_ = default_color_;
  }

  // Performs the work to test the rendering with a shape centered in the
  // image. To pass, the renderer will have to have been populated with a
  // compatible shape and camera configuration (e.g., PopulateSphereTest()).
  void RenderEngineVtkTest::PerformCenterShapeTest(RenderEngineVtk* renderer, const char* name,
                              const DepthRenderCamera* camera) {
    const DepthRenderCamera& cam = camera ? *camera : depth_camera_;
    const int w = cam.core().intrinsics().width();
    const int h = cam.core().intrinsics().height();
    // Can't use the member images in case the camera has been configured to a
    // different size than the default camera_ configuration.
    ImageRgba8U color(w, h);
    ImageDepth32F depth(w, h);
    ImageLabel16I label(w, h);
    Render(renderer, &cam, &color, &depth, &label);

    VerifyCenterShapeTest(*renderer, name, cam, color, depth, label);
  }

  void RenderEngineVtkTest::VerifyCenterShapeTest(const RenderEngineVtk& renderer, const char* name,
                             const DepthRenderCamera& camera,
                             const ImageRgba8U& color,
                             const ImageDepth32F& depth,
                             const ImageLabel16I& label) const {
    VerifyOutliers(renderer, camera, name, &color, &depth, &label);

    // Verifies inside the sphere.
    const ScreenCoord inlier = GetInlier(camera.core().intrinsics());
    const int x = inlier.x;
    const int y = inlier.y;
    EXPECT_TRUE(CompareColor(expected_color_, color, inlier))
        << "Color at: " << inlier << " for test: " << name;
    EXPECT_TRUE(
        IsExpectedDepth(depth, inlier, expected_object_depth_, kDepthTolerance))
        << "Depth at: " << inlier << " for test: " << name;
    EXPECT_EQ(label.at(x, y)[0], static_cast<int>(expected_label_))
        << "Label at: " << inlier << " for test: " << name;
  }

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
