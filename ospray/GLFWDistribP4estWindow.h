#pragma once

#include <GLFW/glfw3.h>
#include <functional>
#include <vector>
#include "ArcballCamera.h"
// ospcommon
#include "ospcommon/box.h"
#include "ospcommon/vec.h"
#include "ospcommon/containers/TransactionalBuffer.h"
// ospray
#include "ospray/ospray.h"

struct WindowState {
  bool quit;
  bool cameraChanged;
  bool fbSizeChanged;
  int spp;
  ospcommon::vec2i windowSize;
  ospcommon::vec3f eyePos;
  ospcommon::vec3f lookDir;
  ospcommon::vec3f upDir;

  WindowState();
};

class GLFWDistribP4estWindow
{
 public:
  GLFWDistribP4estWindow(const ospcommon::vec2i &windowSize,
                   const ospcommon::box3f &worldBounds,
                   const std::vector<OSPWorld> &models,
                   OSPRenderer renderer);

  ~GLFWDistribP4estWindow();

  static GLFWDistribP4estWindow *getActiveWindow();

  std::vector<OSPWorld> getModels();
  void setModels(const std::vector<OSPWorld> &newModel);

  void resetAccumulation();

  void registerDisplayCallback(
      std::function<void(GLFWDistribP4estWindow *)> callback);

  void registerImGuiCallback(std::function<void()> callback);

  void mainLoop();

  void addObjectToCommit(OSPObject obj);

 protected:
  void reshape(const ospcommon::vec2i &newWindowSize);
  void motion(const ospcommon::vec2f &position);
  void display();
  void startNewOSPRayFrame();
  void waitOnOSPRayFrame();
  void updateTitleBar();

  static GLFWDistribP4estWindow *activeWindow;

  ospcommon::vec2i windowSize;
  ospcommon::box3f worldBounds;
  std::vector<OSPWorld> models;
  OSPRenderer renderer = nullptr;

  int mpiRank = -1;
  int mpiWorldSize = -1;

  // GLFW window instance
  GLFWwindow *glfwWindow = nullptr;

  // Arcball camera instance
  std::unique_ptr<ArcballCamera> arcballCamera;

  // OSPRay objects managed by this class
  OSPCamera camera           = nullptr;
  OSPFrameBuffer framebuffer = nullptr;
  OSPFuture currentFrame     = nullptr;

  // List of OSPRay handles to commit before the next frame
  ospcommon::TransactionalBuffer<OSPObject> objectsToCommit;

  // OpenGL framebuffer texture
  GLuint framebufferTexture = 0;

  // optional registered display callback, called before every display()
  std::function<void(GLFWDistribP4estWindow *)> displayCallback;

  // toggles display of ImGui UI, if an ImGui callback is provided
  bool showUi = true;

  // optional registered ImGui callback, called during every frame to build UI
  std::function<void()> uiCallback;

  // FPS measurement of last frame
  float latestFPS{0.f};

  // The window state to be sent out over MPI to the other rendering processes
  WindowState windowState;
};
