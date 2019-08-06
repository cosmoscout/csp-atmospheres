////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Atmosphere.hpp"

#include "../../../src/cs-core/GraphicsEngine.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-utils/FrameTimings.hpp"

#include <VistaKernel/VistaSystem.h>
#include <glm/gtc/type_ptr.hpp>

namespace csp::atmospheres {

////////////////////////////////////////////////////////////////////////////////////////////////////

Atmosphere::Atmosphere(std::shared_ptr<cs::core::GraphicsEngine> const& pGraphicsEngine,
    std::shared_ptr<Plugin::Properties> const& pProperties, std::string const& sCenterName,
    std::string const& sFrameName, double tStartExistence, double tEndExistence)
    : cs::scene::CelestialObject(sCenterName, sFrameName, tStartExistence, tEndExistence)
    , mGraphicsEngine(pGraphicsEngine)
    , mProperties(pProperties)
    , mRadii(cs::core::SolarSystem::getRadii(sCenterName))
    , mAtmosphere() {

  pVisibleRadius = mRadii[0];

  mAtmosphere.setUseLinearDepthBuffer(true);
  mAtmosphere.setUseToneMapping(true, 0.5, 2.4);
  mAtmosphere.setDrawSun(false);
  mAtmosphere.setSecondaryRaySteps(3);
  mAtmosphere.setPrimaryRaySteps(mProperties->mQuality.get());

  // scene-wide settings -----------------------------------------------------
  mGraphicsEngine->pAmbientBrightness.onChange().connect(
      [this](float val) { mAtmosphere.setAmbientBrightness(val * 0.4f); });

  mProperties->mQuality.onChange().connect(
      [this](int val) { mAtmosphere.setPrimaryRaySteps(val); });

  mProperties->mEnableWater.onChange().connect([this](bool val) { mAtmosphere.setDrawWater(val); });

  mProperties->mEnableClouds.onChange().connect([this](bool val) {
    mAtmosphere.setCloudTexture(val ? mCloudTexture.get() : nullptr, (float)mCloudHeight);
  });

  mProperties->mWaterLevel.onChange().connect(
      [this](float val) { mAtmosphere.setWaterLevel(val / 1000); });

  mGraphicsEngine->pEnableShadows.onChange().connect([this](bool val) {
    mAtmosphere.setShadowMap(
        (mGraphicsEngine->pEnableShadows.get() && mProperties->mEnableLightShafts.get())
            ? mGraphicsEngine->getShadowMap()
            : nullptr);
  });

  mProperties->mEnableLightShafts.onChange().connect([this](bool val) {
    mAtmosphere.setShadowMap(
        (mGraphicsEngine->pEnableShadows.get() && mProperties->mEnableLightShafts.get())
            ? mGraphicsEngine->getShadowMap()
            : nullptr);
  });
}

////////////////////////////////////////////////////////////////////////////////////////////////////

VistaAtmosphere& Atmosphere::getAtmosphere() {
  return mAtmosphere;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Atmosphere::setSun(std::shared_ptr<const cs::scene::CelestialObject> const& sun) {
  mSun = sun;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Atmosphere::setCloudTexture(std::shared_ptr<VistaTexture> const& texture, double height) {
  mCloudTexture = texture;
  mCloudHeight  = height;

  if (mProperties->mEnableClouds.get()) {
    mAtmosphere.setCloudTexture(mCloudTexture.get(), (float)mCloudHeight);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Atmosphere::update(double tTime, cs::scene::CelestialObserver const& oObs) {
  cs::scene::CelestialObject::update(tTime, oObs);

  if (mSun && getIsInExistence()) {
    auto sunDir = glm::normalize(
        glm::inverse(matWorldTransform) * (mSun->getWorldPosition() - getWorldPosition()));

    mAtmosphere.setSunDirection(VistaVector3D(sunDir[0], sunDir[1], sunDir[2]));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool Atmosphere::Do() {
  if (mProperties->mEnabled.get() && getIsInExistence() && pVisible.get()) {
    cs::utils::FrameTimings::ScopedTimer timer("Atmosphere " + getCenterName());
    VistaTransformMatrix                 matM(glm::value_ptr(matWorldTransform), true);
    VistaTransformMatrix                 matScale;
    matScale.SetToScaleMatrix((float)mRadii[0], (float)mRadii[0], (float)mRadii[0]);
    mAtmosphere.draw(matM * matScale);
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool Atmosphere::GetBoundingBox(VistaBoundingBox& bb) {
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
