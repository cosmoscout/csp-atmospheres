////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Atmosphere.hpp"

#include "../../../src/cs-core/GraphicsEngine.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-graphics/TextureLoader.hpp"
#include "../../../src/cs-utils/utils.hpp"

#include <VistaKernel/GraphicsManager/VistaGroupNode.h>
#include <VistaKernel/GraphicsManager/VistaOpenGLNode.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaKernelOpenSGExt/VistaOpenSGMaterialTools.h>

namespace csp::atmospheres {

////////////////////////////////////////////////////////////////////////////////////////////////////

Atmosphere::Atmosphere(std::shared_ptr<Plugin::Properties> const& pProperties,
    Plugin::Settings::Atmosphere const& settings, std::string const& sCenterName,
    std::string const& sFrameName, double tStartExistence, double tEndExistence)
    : cs::scene::CelestialObject(sCenterName, sFrameName, tStartExistence, tEndExistence)
    , mRenderer(pProperties)
    , mProperties(pProperties) {

  auto radii(cs::core::SolarSystem::getRadii(sCenterName));
  pVisibleRadius = radii[0];

  if (settings.mCloudTexture) {
    mRenderer.setCloudTexture(
        cs::graphics::TextureLoader::loadFromFile(*settings.mCloudTexture), *settings.mCloudHeight);
  }

  mRenderer.setRadii(radii);
  mRenderer.setUseLinearDepthBuffer(true);
  mRenderer.setDrawSun(false);
  mRenderer.setSecondaryRaySteps(3);
  mRenderer.setPrimaryRaySteps(mProperties->mQuality.get());
  mRenderer.setAtmosphereHeight(settings.mAtmosphereHeight);
  mRenderer.setMieHeight(settings.mMieHeight);
  mRenderer.setMieScattering(
      glm::vec3(settings.mMieScatteringR, settings.mMieScatteringG, settings.mMieScatteringB));
  mRenderer.setMieAnisotropy(settings.mMieAnisotropy);
  mRenderer.setRayleighHeight(settings.mRayleighHeight);
  mRenderer.setRayleighScattering(glm::vec3(
      settings.mRayleighScatteringR, settings.mRayleighScatteringG, settings.mRayleighScatteringB));
  mRenderer.setRayleighAnisotropy(settings.mRayleighAnisotropy);

  VistaSceneGraph* pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  mAtmosphereNode      = pSG->NewOpenGLNode(pSG->GetRoot(), &mRenderer);
  mAtmosphereNode->SetIsEnabled(false);
  VistaOpenSGMaterialTools::SetSortKeyOnSubtree(
      mAtmosphereNode, static_cast<int>(cs::utils::DrawOrder::eAtmospheres));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Atmosphere::~Atmosphere() {
  VistaSceneGraph* pSG = GetVistaSystem()->GetGraphicsManager()->GetSceneGraph();
  pSG->GetRoot()->DisconnectChild(mAtmosphereNode);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

AtmosphereRenderer& Atmosphere::getRenderer() {
  return mRenderer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Atmosphere::update(double time, cs::scene::CelestialObserver const& oObs) {
  cs::scene::CelestialObject::update(time, oObs);
  if (mProperties->mEnabled.get() && getIsInExistence() && pVisible.get()) {
    mAtmosphereNode->SetIsEnabled(true);
    mRenderer.setWorldTransform(matWorldTransform);
  } else {
    mAtmosphereNode->SetIsEnabled(false);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
