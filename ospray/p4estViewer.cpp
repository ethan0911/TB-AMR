#include <iterator>
#include <memory>
#include <random>
#include "GLFWOSPRayWindow.h"

#include <imgui.h>

using namespace ospcommon;

struct Sphere {
  vec3f pos;
  float radius;
};

int main(int argc, const char **argv) {
  // initialize OSPRay; OSPRay parses (and removes) its commandline parameters,
  // e.g. "--osp:debug"
  OSPError initError = ospInit(&argc, argv);

  if (initError != OSP_NO_ERROR)
    return initError;

  // set an error callback to catch any OSPRay errors and exit the application
  ospDeviceSetErrorFunc(
      ospGetCurrentDevice(), [](OSPError error, const char *errorDetails) {
        std::cerr << "OSPRay error: " << errorDetails << std::endl;
        exit(error);
      });

  std::vector<Sphere> spheres(10);

  std::random_device rd;
  std::mt19937 rng(rd());

  const vec3f brickLower(-1.f);
  const vec3f brickUpper(1.f);

  // Generate spheres within the box padded by the radius, so we don't need
  // to worry about ghost bounds
  std::uniform_real_distribution<float> distX(brickLower.x, brickUpper.x);
  std::uniform_real_distribution<float> distY(brickLower.y, brickUpper.y);
  std::uniform_real_distribution<float> distZ(brickLower.z, brickUpper.z);
  std::uniform_real_distribution<float> radii(0.05, 0.25);

  for (auto &s : spheres) {
    s.pos.x = distX(rng);
    s.pos.y = distY(rng);
    s.pos.z = distZ(rng);
    s.radius = radii(rng);
  }

  OSPData sphereData = ospNewData(spheres.size() * sizeof(Sphere), OSP_UCHAR,
                                  spheres.data());
  ospCommit(sphereData);

  OSPMaterial material = ospNewMaterial2("scivis", "OBJMaterial");
  ospSet3f(material, "Kd", 0.f, 0.f, 1.f);
  ospSet3f(material, "Ks", 1.f, 1.f, 1.f);
  ospCommit(material);

  OSPGeometry sphereGeom = ospNewGeometry("spheres");
  ospSet1i(sphereGeom, "bytes_per_sphere", int(sizeof(Sphere)));
  ospSet1i(sphereGeom, "offset_radius", int(sizeof(vec3f)));
  ospSetData(sphereGeom, "spheres", sphereData);
  ospSetMaterial(sphereGeom, material);
  ospRelease(material);
  ospRelease(sphereData);
  ospCommit(sphereGeom);

  // create the "world" model which will contain all of our geometries
  OSPModel world = ospNewModel();
  ospAddGeometry(world, sphereGeom);
  ospCommit(world);
  ospRelease(sphereGeom);

  // create and setup an ambient light
  std::array<OSPLight, 2> lights = {
    ospNewLight3("ambient"),
    ospNewLight3("distant")
  };
  ospCommit(lights[0]);

  ospSet3f(lights[1], "direction", -1.f, -1.f, 0.5f);
  ospCommit(lights[1]);

  OSPData lightData = ospNewData(lights.size(), OSP_LIGHT, lights.data(), 0);
  ospCommit(lightData);

  // create OSPRay renderer
  OSPRenderer renderer = ospNewRenderer("scivis");
  ospSetObject(renderer, "lights", lightData);
  ospSet3f(renderer, "bgColor", 1.0, 1.0, 1.0);
  ospCommit(renderer);
  ospRelease(lightData);

  // create a GLFW OSPRay window: this object will create and manage the OSPRay
  // frame buffer and camera directly
  auto glfwOSPRayWindow =
      std::unique_ptr<GLFWOSPRayWindow>(new GLFWOSPRayWindow(
          vec2i{1024, 768}, box3f(vec3f(-1.f), vec3f(1.f)), world, renderer));

  glfwOSPRayWindow->registerImGuiCallback([&]() {
    static int spp = 1;
    if (ImGui::SliderInt("spp", &spp, 1, 64)) {
      ospSet1i(renderer, "spp", spp);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }
  });

  // start the GLFW main loop, which will continuously render
  glfwOSPRayWindow->mainLoop();

  // cleanup remaining objects
  ospRelease(world);
  ospRelease(renderer);

  // cleanly shut OSPRay down
  ospShutdown();

  return 0;
}

