////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Plugin.hpp"

#include "Atmosphere.hpp"
#include "AtmosphereRenderer.hpp"

#include "../../../src/cs-core/GraphicsEngine.hpp"
#include "../../../src/cs-core/GuiManager.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-utils/logger.hpp"

#include <VistaKernel/GraphicsManager/VistaOpenGLNode.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/GraphicsManager/VistaTransformNode.h>

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

  // Create default logger for this plugin.
  spdlog::set_default_logger(cs::utils::logger::createLogger("csp-atmospheres"));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::init() {

  spdlog::info("Loading plugin...");

  mPluginSettings = mAllSettings->mPlugins.at("csp-atmospheres");

  mGuiManager->addSettingsSectionToSideBarFromHTML(
      "Atmospheres", "blur_circular", "../share/resources/gui/atmospheres_settings.html");
  mGuiManager->addScriptToGuiFromJS("../share/resources/gui/js/csp-atmospheres.js");

  for (auto const& atmoSettings : mPluginSettings.mAtmospheres) {
    auto anchor = mAllSettings->mAnchors.find(atmoSettings.first);

    if (anchor == mAllSettings->mAnchors.end()) {
      throw std::runtime_error(
          "There is no Anchor \"" + atmoSettings.first + "\" defined in the settings.");
    }

    auto   existence       = cs::core::getExistenceFromSettings(*anchor);
    double tStartExistence = existence.first;
    double tEndExistence   = existence.second;

    auto atmosphere = std::make_shared<Atmosphere>(mProperties, atmoSettings.second,
        anchor->second.mCenter, anchor->second.mFrame, tStartExistence, tEndExistence);

    atmosphere->getRenderer().setHDRBuffer(mGraphicsEngine->getHDRBuffer());

    mSolarSystem->registerAnchor(atmosphere);

    mAtmospheres.emplace_back(atmosphere);
  }

  mGuiManager->getGui()->registerCallback("atmosphere.setEnableWater",
      "Enables or disables rendering of a water surface.",
      std::function([this](bool enable) { mProperties->mEnableWater = enable; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setEnableClouds",
      "Enables or disables rendering of a cloud layer.",
      std::function([this](bool enable) { mProperties->mEnableClouds = enable; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setEnable",
      "Enables or disables rendering of atmospheres.",
      std::function([this](bool value) { mProperties->mEnabled = value; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setEnableLightShafts",
      "If shadows are enabled, this enables or disables rendering of light shafts in the "
      "atmosphere.",
      std::function([this](bool value) { mProperties->mEnableLightShafts = value; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setQuality",
      "Higher values create a more realistic atmosphere.",
      std::function([this](double value) { mProperties->mQuality = static_cast<int>(value); }));

  mGuiManager->getGui()->registerCallback("atmosphere.setWaterLevel",
      "Sets the height of the water surface relative to the planet's radius.",
      std::function(
          [this](double value) { mProperties->mWaterLevel = static_cast<float>(value); }));

  mEnableShadowsConnection = mGraphicsEngine->pEnableShadows.connectAndTouch([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      if (mGraphicsEngine->pEnableShadows.get() && mProperties->mEnableLightShafts.get()) {
        atmosphere->getRenderer().setShadowMap(mGraphicsEngine->getShadowMap());
      } else {
        atmosphere->getRenderer().setShadowMap(nullptr);
      }
    }
  });

  mEnableHDRConnection = mGraphicsEngine->pEnableHDR.connectAndTouch([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      atmosphere->getRenderer().setUseToneMapping(!val, 0.6f, 2.2f);
      if (val) {
        atmosphere->getRenderer().setHDRBuffer(mGraphicsEngine->getHDRBuffer());
      } else {
        atmosphere->getRenderer().setHDRBuffer(nullptr);
      }
    }
  });

  mAmbientBrightnessConnection =
      mGraphicsEngine->pAmbientBrightness.connectAndTouch([this](float val) {
        for (auto const& atmosphere : mAtmospheres) {
          atmosphere->getRenderer().setAmbientBrightness(val * 0.4f);
        }
      });

  mProperties->mEnableLightShafts.connectAndTouch([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      if (mGraphicsEngine->pEnableShadows.get() && mProperties->mEnableLightShafts.get()) {
        atmosphere->getRenderer().setShadowMap(mGraphicsEngine->getShadowMap());
      } else {
        atmosphere->getRenderer().setShadowMap(nullptr);
      }
    }
  });

  spdlog::info("Loading done.");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::deInit() {
  spdlog::info("Unloading plugin...");

  for (auto const& atmosphere : mAtmospheres) {
    mSolarSystem->unregisterAnchor(atmosphere);
  }

  mGuiManager->removeSettingsSection("Atmospheres");

  mGuiManager->getGui()->unregisterCallback("atmosphere.setEnableWater");
  mGuiManager->getGui()->unregisterCallback("atmosphere.setEnableClouds");
  mGuiManager->getGui()->unregisterCallback("atmosphere.setEnable");
  mGuiManager->getGui()->unregisterCallback("atmosphere.setEnableLightShafts");
  mGuiManager->getGui()->unregisterCallback("atmosphere.setQuality");
  mGuiManager->getGui()->unregisterCallback("atmosphere.setWaterLevel");

  mGraphicsEngine->pEnableShadows.disconnect(mEnableShadowsConnection);
  mGraphicsEngine->pEnableHDR.disconnect(mEnableHDRConnection);
  mGraphicsEngine->pAmbientBrightness.disconnect(mAmbientBrightnessConnection);

  spdlog::info("Unloading done.");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::update() {
  float fIntensity = 1.f;
  for (auto const& atmosphere : mAtmospheres) {
    if (mProperties->mEnabled.get()) {
      float brightness = atmosphere->getRenderer().getApproximateSceneBrightness();
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

    atmosphere->getRenderer().setSun(sunDirection, static_cast<float>(sunIlluminance));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
