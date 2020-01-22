////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../src/cs-utils/TestImageCompare.hpp"
#include "../../../src/cs-utils/doctest.hpp"
#include "../src/VistaAtmosphere.hpp"

// Graphical tests rely on Xvfb and image magick and therefore they are only available on Linux
#ifdef __linux__

#include <VistaKernel/GraphicsManager/VistaGeometryFactory.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/GraphicsManager/VistaTransformNode.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaTools/VistaRandomNumberGenerator.h>

namespace csp::atmospheres {

TEST_CASE("[graphical] csp::atmospheres::VistaAtmosphere") {

  const float PLANET_RADIUS(10.f);

  // The TestImageCompare initializes a VistaSystem for us. Once it's doComparison method gets
  // called, a single frame will be rendered, caputered with imagemagick and compared to a reference
  // image (using imagemagick's compare tool).
  cs::utils::TestImageCompare compare("csp-atmospheres-VistaAtmosphere-01", 1);

  VistaSceneGraph*     pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  VistaGeometryFactory oGeometryFactory(pSG);

  // Create a "planet".
  VistaGeometry*      pPlanet          = oGeometryFactory.CreateSphere(1, 300, VistaColor::GRAY);
  VistaTransformNode* pPlanetTransform = pSG->NewTransformNode(pSG->GetRoot());
  pPlanetTransform->SetScale(PLANET_RADIUS, PLANET_RADIUS, PLANET_RADIUS);
  pPlanetTransform->Translate(0, 0, -2 * PLANET_RADIUS);
  pSG->NewGeomNode(pPlanetTransform, pPlanet);

  // Create some "mountains".
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

  // Create the atmosphere.
  auto atmosphere = new VistaAtmosphere();
  atmosphere->loadPreset(VistaAtmosphere::Preset::eMars);
  pSG->NewOpenGLNode(pPlanetTransform, atmosphere);

  // Do the image comparision, allowing for a slight maximum per-pixel difference of 0.1%.
  CHECK(compare.doComparison(0.1) == "0");
}
} // namespace csp::atmospheres

#endif