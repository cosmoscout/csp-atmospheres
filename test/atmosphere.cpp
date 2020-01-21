////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../src/cs-utils/doctest.hpp"
#include "../src/VistaAtmosphere.hpp"

// Graphical tests rely on Xvfb and image magick and therefore they are only available on Linux
#ifdef __linux__
/*
#include <VistaBase/VistaExceptionBase.h>
#include <VistaBase/VistaStreamUtils.h>
#include <VistaKernel/EventManager/VistaEventHandler.h>
#include <VistaKernel/EventManager/VistaEventManager.h>
#include <VistaKernel/EventManager/VistaSystemEvent.h>
#include <VistaKernel/GraphicsManager/VistaGeometryFactory.h>
#include <VistaKernel/GraphicsManager/VistaLightNode.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/GraphicsManager/VistaTransformNode.h>
#include <VistaKernel/InteractionManager/VistaKeyboardSystemControl.h>
#include <VistaKernel/VistaFrameLoop.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaTools/VistaRandomNumberGenerator.h>

namespace csp::atmospheres {

const float       PLANET_RADIUS(10.f);
const std::string referenceImage("test/reference/csp-atmospheres-VistaAtmosphere-01.png");
const std::string testImage("test/csp-atmospheres-VistaAtmosphere-01.png");
const std::string differenceImage("test/csp-atmospheres-VistaAtmosphere-01-diff.png");

std::string exec(std::string const& cmd) {
  std::array<char, 128>                    buffer;
  std::string                              result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

class FrameCapture : public VistaEventHandler {

 public:
  FrameCapture(int32_t atFrame)
      : mAtFrame(atFrame) {
  }

  virtual void HandleEvent(VistaEvent* pEvent) {
    if (pEvent->GetId() == VistaSystemEvent::VSE_POSTGRAPHICS) {
      if (GetVistaSystem()->GetFrameLoop()->GetFrameCount() == mAtFrame) {
        exec("import -window 'CosmoScout VR' " + testImage);
      }
    }
  }

 private:
  int32_t mAtFrame = 1;
};

bool renderImage() {

  VistaSystem* pVistaSystem = new VistaSystem();
  pVistaSystem->SetIniSearchPaths({"../share/config/vista"});

  int          argc     = 5;
  const char*  n_argv[] = {"", "-vistaini", "vista_test.ini", "-kill_after_frame", "1"};
  const char** argv     = n_argv;

  std::ostringstream vistaOutput;
  vstr::SetOutStream(&vistaOutput);
  vstr::SetWarnStream(&vistaOutput);
  vstr::SetDebugStream(&vistaOutput);

  if (!pVistaSystem->Init(argc, const_cast<char**>(argv))) {
    return false;
  }

  pVistaSystem->GetGraphicsManager()->SetBackgroundColor(VistaColor(0, 0, 0));

  FrameCapture capture(1);
  pVistaSystem->GetEventManager()->AddEventHandler(
      &capture, VistaSystemEvent::GetTypeId(), VistaSystemEvent::VSE_POSTGRAPHICS);

  VistaSceneGraph*     pSG = pVistaSystem->GetGraphicsManager()->GetSceneGraph();
  VistaGeometryFactory oGeometryFactory(pSG);

  // planet
  VistaGeometry*      pPlanet          = oGeometryFactory.CreateSphere(1, 300, VistaColor::GRAY);
  VistaTransformNode* pPlanetTransform = pSG->NewTransformNode(pSG->GetRoot());
  pPlanetTransform->SetScale(PLANET_RADIUS, PLANET_RADIUS, PLANET_RADIUS);
  pPlanetTransform->Translate(0, 0, -2 * PLANET_RADIUS);
  pSG->NewGeomNode(pPlanetTransform, pPlanet);

  // "mountains"
  VistaRandomNumberGenerator* pRnd = VistaRandomNumberGenerator::GetStandardRNG();
  pRnd->SetSeed(0);

  for (int i(0); i < 1000; ++i) {
    VistaGeometry*      pMountain      = oGeometryFactory.CreateSphere(1, 50, VistaColor::GRAY);
    VistaTransformNode* pTransformNode = pSG->NewTransformNode(pPlanetTransform);
    pTransformNode->SetScale(1, 1, 1);

    float size  = pRnd->GenerateFloat(0.2f, 0.22f);
    float phi   = pRnd->GenerateFloat(0.0f, 2 * Vista::Pi);
    float theta = acos(pRnd->GenerateFloat(-1.f, 1.f));
    float x     = sin(theta) * cos(phi) * 0.8;
    float y     = sin(theta) * sin(phi) * 0.8;
    float z     = cos(theta) * 0.8;

    pTransformNode->Scale(size, size, size);
    pTransformNode->Translate(x, y, z);
    pSG->NewGeomNode(pTransformNode, pMountain);
  }

  // atmosphere
  VistaAtmosphere* pAtmosphere = new VistaAtmosphere();
  pAtmosphere->loadPreset(VistaAtmosphere::Preset::eMars);

  pSG->NewOpenGLNode(pPlanetTransform, pAtmosphere);

  pVistaSystem->Run();

  delete pAtmosphere;
  delete pVistaSystem;

  return true;
}

TEST_CASE("[graphical] csp::atmospheres::VistaAtmosphere") {
  CHECK_UNARY(renderImage());

  std::string compareCommand =
      "compare -metric PAE " + testImage + " " + referenceImage + " " + differenceImage + " 2>&1";
  std::string result = exec(compareCommand);
  CHECK(result == "0 (0)");
}
} // namespace csp::atmospheres
*/
#endif