////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Plugin.hpp"

#include "Atmosphere.hpp"

#include "../../../src/cs-core/GraphicsEngine.hpp"
#include "../../../src/cs-core/GuiManager.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-graphics/TextureLoader.hpp"

#include <VistaKernel/GraphicsManager/VistaOpenGLNode.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/GraphicsManager/VistaTransformNode.h>
#include <VistaKernelOpenSGExt/VistaOpenSGMaterialTools.h>

////////////////////////////////////////////////////////////////////////////////////////////////////

EXPORT_FN cs::core::PluginBase* create() {
  return new csp::atmospheres::Plugin;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

EXPORT_FN void destroy(cs::core::PluginBase* pluginBase) {
  delete pluginBase;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

namespace csp::atmospheres {

////////////////////////////////////////////////////////////////////////////////////////////////////

void from_json(const nlohmann::json& j, Plugin::Settings::Atmosphere& o) {
  o.mAtmosphereHeight    = cs::core::parseProperty<double>("atmosphereHeight", j);
  o.mMieHeight           = cs::core::parseProperty<double>("mieHeight", j);
  o.mMieScatteringR      = cs::core::parseProperty<double>("mieScatteringR", j);
  o.mMieScatteringG      = cs::core::parseProperty<double>("mieScatteringG", j);
  o.mMieScatteringB      = cs::core::parseProperty<double>("mieScatteringB", j);
  o.mMieAnisotropy       = cs::core::parseProperty<double>("mieAnisotropy", j);
  o.mRayleighHeight      = cs::core::parseProperty<double>("rayleighHeight", j);
  o.mRayleighScatteringR = cs::core::parseProperty<double>("rayleighScatteringR", j);
  o.mRayleighScatteringG = cs::core::parseProperty<double>("rayleighScatteringG", j);
  o.mRayleighScatteringB = cs::core::parseProperty<double>("rayleighScatteringB", j);
  o.mRayleighAnisotropy  = cs::core::parseProperty<double>("rayleighAnisotropy", j);

  o.mCloudTexture = cs::core::parseOptional<std::string>("cloudTexture", j);

  o.mCloudHeight = cs::core::parseOptional<double>("cloudHeight", j);
  if (!o.mCloudHeight) {
    o.mCloudHeight = 0.001;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void from_json(const nlohmann::json& j, Plugin::Settings& o) {
  cs::core::parseSection("csp-atmospheres", [&] {
    o.mAtmospheres =
        cs::core::parseMap<std::string, Plugin::Settings::Atmosphere>("atmospheres", j);
  });
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Plugin::Plugin()
    : mProperties(std::make_shared<Properties>()) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::init() {
  std::cout << "Loading: CosmoScout VR Plugin Atmosphere" << std::endl;

  mPluginSettings = mAllSettings->mPlugins.at("csp-atmospheres");

  mGuiManager->addSettingsSectionToSideBarFromHTML(
      "Atmospheres", "blur_circular", "../share/resources/gui/atmospheres_settings.html");
  mGuiManager->addScriptToSideBarFromJS("../share/resources/gui/js/atmospheres_settings.js");

  for (auto const& atmoSettings : mPluginSettings.mAtmospheres) {
    auto anchor = mAllSettings->mAnchors.find(atmoSettings.first);

    if (anchor == mAllSettings->mAnchors.end()) {
      throw std::runtime_error(
          "There is no Anchor \"" + atmoSettings.first + "\" defined in the settings.");
    }

    auto   existence       = cs::core::getExistenceFromSettings(*anchor);
    double tStartExistence = existence.first;
    double tEndExistence   = existence.second;

    auto atmosphere = std::make_shared<Atmosphere>(mGraphicsEngine, mProperties,
        anchor->second.mCenter, anchor->second.mFrame, tStartExistence, tEndExistence);

    mSolarSystem->registerAnchor(atmosphere);

    if (atmoSettings.second.mCloudTexture) {
      atmosphere->setCloudTexture(
          cs::graphics::TextureLoader::loadFromFile(*atmoSettings.second.mCloudTexture),
          *atmoSettings.second.mCloudHeight);
    }

    atmosphere->setHDRBuffer(mGraphicsEngine->getHDRBuffer());
    atmosphere->setAtmosphereHeight(atmoSettings.second.mAtmosphereHeight);
    atmosphere->setMieHeight(atmoSettings.second.mMieHeight);
    atmosphere->setMieScattering(glm::vec3(atmoSettings.second.mMieScatteringR,
        atmoSettings.second.mMieScatteringG, atmoSettings.second.mMieScatteringB));
    atmosphere->setMieAnisotropy(atmoSettings.second.mMieAnisotropy);
    atmosphere->setRayleighHeight(atmoSettings.second.mRayleighHeight);
    atmosphere->setRayleighScattering(glm::vec3(atmoSettings.second.mRayleighScatteringR,
        atmoSettings.second.mRayleighScatteringG, atmoSettings.second.mRayleighScatteringB));
    atmosphere->setRayleighAnisotropy(atmoSettings.second.mRayleighAnisotropy);

    VistaOpenGLNode* atmosphereNode =
        mSceneGraph->NewOpenGLNode(mSceneGraph->GetRoot(), atmosphere.get());
    VistaOpenSGMaterialTools::SetSortKeyOnSubtree(
        atmosphereNode, static_cast<int>(cs::utils::DrawOrder::eAtmospheres));

    mAtmosphereNodes.push_back(atmosphereNode);
    mAtmospheres.push_back(atmosphere);
  }

  mGuiManager->getSideBar()->registerCallback<bool>(
      "set_enable_water", ([this](bool enable) { mProperties->mEnableWater = enable; }));

  mGuiManager->getSideBar()->registerCallback<bool>(
      "set_enable_clouds", ([this](bool enable) { mProperties->mEnableClouds = enable; }));

  mGuiManager->getSideBar()->registerCallback<bool>(
      "set_enable_atmosphere", ([this](bool value) { mProperties->mEnabled = value; }));

  mGuiManager->getSideBar()->registerCallback<bool>(
      "set_enable_light_shafts", ([this](bool value) { mProperties->mEnableLightShafts = value; }));

  mGuiManager->getSideBar()->registerCallback<double>(
      "set_atmosphere_quality", ([this](const int value) { mProperties->mQuality = value; }));

  mGuiManager->getSideBar()->registerCallback<double>(
      "set_water_level", ([this](double value) { mProperties->mWaterLevel = value; }));

  mGraphicsEngine->pEnableShadows.onChange().connect([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      if (mGraphicsEngine->pEnableShadows.get() && mProperties->mEnableLightShafts.get()) {
        atmosphere->setShadowMap(mGraphicsEngine->getShadowMap());
      } else {
        atmosphere->setShadowMap(nullptr);
      }
    }
  });

  mGraphicsEngine->pEnableHDR.onChange().connect([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      if (val) {
        atmosphere->setHDRBuffer(mGraphicsEngine->getHDRBuffer());
      } else {
        atmosphere->setHDRBuffer(nullptr);
      }
    }
  });

  mProperties->mEnableLightShafts.onChange().connect([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      if (mGraphicsEngine->pEnableShadows.get() && mProperties->mEnableLightShafts.get()) {
        atmosphere->setShadowMap(mGraphicsEngine->getShadowMap());
      } else {
        atmosphere->setShadowMap(nullptr);
      }
    }
  });

  mGraphicsEngine->pAmbientBrightness.onChange().connect([this](float val) {
    for (auto const& atmosphere : mAtmospheres) {
      atmosphere->setAmbientBrightness(val * 0.4f);
    }
  });

  mGraphicsEngine->pEnableHDR.onChange().connect([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      atmosphere->setUseToneMapping(!val, 0.6f, 2.2f);
    }
  });
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::deInit() {
  for (auto const& atmosphere : mAtmospheres) {
    mSolarSystem->unregisterAnchor(atmosphere);
  }

  for (auto const& atmosphereNode : mAtmosphereNodes) {
    mSceneGraph->GetRoot()->DisconnectChild(atmosphereNode);
  }

  mGuiManager->getSideBar()->unregisterCallback("set_enable_water");
  mGuiManager->getSideBar()->unregisterCallback("set_enable_clouds");
  mGuiManager->getSideBar()->unregisterCallback("set_enable_atmosphere");
  mGuiManager->getSideBar()->unregisterCallback("set_enable_light_shafts");
  mGuiManager->getSideBar()->unregisterCallback("set_atmosphere_quality");
  mGuiManager->getSideBar()->unregisterCallback("set_water_level");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::update() {
  float fIntensity = 1.f;
  for (auto const& atmosphere : mAtmospheres) {
    if (mProperties->mEnabled.get()) {
      float brightness = atmosphere->getApproximateSceneBrightness();
      fIntensity *= (1.f - brightness);
    }
  }

  mGraphicsEngine->pApproximateSceneBrightness = fIntensity;

  for (auto const& atmosphere : mAtmospheres) {
    double sunIlluminance = 10.0;

    if (mGraphicsEngine->pEnableHDR.get()) {
      sunIlluminance = mSolarSystem->getSunIlluminance(atmosphere->getWorldTransform()[3]);
    }

    auto sunDirection = mSolarSystem->getSunDirection(atmosphere->getWorldTransform()[3]);

    atmosphere->setSun(sunDirection, sunIlluminance);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
