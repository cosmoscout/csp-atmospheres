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

void from_json(nlohmann::json const& j, Plugin::Settings::Atmosphere& o) {
  cs::core::Settings::deserialize(j, "atmosphereHeight", o.mAtmosphereHeight);
  cs::core::Settings::deserialize(j, "mieHeight", o.mMieHeight);
  cs::core::Settings::deserialize(j, "mieScatteringR", o.mMieScatteringR);
  cs::core::Settings::deserialize(j, "mieScatteringG", o.mMieScatteringG);
  cs::core::Settings::deserialize(j, "mieScatteringB", o.mMieScatteringB);
  cs::core::Settings::deserialize(j, "mieAnisotropy", o.mMieAnisotropy);
  cs::core::Settings::deserialize(j, "rayleighHeight", o.mRayleighHeight);
  cs::core::Settings::deserialize(j, "rayleighScatteringR", o.mRayleighScatteringR);
  cs::core::Settings::deserialize(j, "rayleighScatteringG", o.mRayleighScatteringG);
  cs::core::Settings::deserialize(j, "rayleighScatteringB", o.mRayleighScatteringB);
  cs::core::Settings::deserialize(j, "rayleighAnisotropy", o.mRayleighAnisotropy);
  cs::core::Settings::deserialize(j, "cloudTexture", o.mCloudTexture);
  cs::core::Settings::deserialize(j, "cloudHeight", o.mCloudHeight);
}

void to_json(nlohmann::json& j, Plugin::Settings::Atmosphere const& o) {
  cs::core::Settings::serialize(j, "atmosphereHeight", o.mAtmosphereHeight);
  cs::core::Settings::serialize(j, "mieHeight", o.mMieHeight);
  cs::core::Settings::serialize(j, "mieScatteringR", o.mMieScatteringR);
  cs::core::Settings::serialize(j, "mieScatteringG", o.mMieScatteringG);
  cs::core::Settings::serialize(j, "mieScatteringB", o.mMieScatteringB);
  cs::core::Settings::serialize(j, "mieAnisotropy", o.mMieAnisotropy);
  cs::core::Settings::serialize(j, "rayleighHeight", o.mRayleighHeight);
  cs::core::Settings::serialize(j, "rayleighScatteringR", o.mRayleighScatteringR);
  cs::core::Settings::serialize(j, "rayleighScatteringG", o.mRayleighScatteringG);
  cs::core::Settings::serialize(j, "rayleighScatteringB", o.mRayleighScatteringB);
  cs::core::Settings::serialize(j, "rayleighAnisotropy", o.mRayleighAnisotropy);
  cs::core::Settings::serialize(j, "cloudTexture", o.mCloudTexture);
  cs::core::Settings::serialize(j, "cloudHeight", o.mCloudHeight);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void from_json(nlohmann::json const& j, Plugin::Settings& o) {
  cs::core::Settings::deserialize(j, "atmospheres", o.mAtmospheres);
  cs::core::Settings::deserialize(j, "enabled", o.mEnabled);
  cs::core::Settings::deserialize(j, "quality", o.mQuality);
  cs::core::Settings::deserialize(j, "waterLevel", o.mWaterLevel);
  cs::core::Settings::deserialize(j, "enableClouds", o.mEnableClouds);
  cs::core::Settings::deserialize(j, "enableLightShafts", o.mEnableLightShafts);
  cs::core::Settings::deserialize(j, "enableWater", o.mEnableWater);
}

void to_json(nlohmann::json& j, Plugin::Settings const& o) {
  cs::core::Settings::serialize(j, "atmospheres", o.mAtmospheres);
  cs::core::Settings::serialize(j, "enabled", o.mEnabled);
  cs::core::Settings::serialize(j, "quality", o.mQuality);
  cs::core::Settings::serialize(j, "waterLevel", o.mWaterLevel);
  cs::core::Settings::serialize(j, "enableClouds", o.mEnableClouds);
  cs::core::Settings::serialize(j, "enableLightShafts", o.mEnableLightShafts);
  cs::core::Settings::serialize(j, "enableWater", o.mEnableWater);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

Plugin::Plugin()
    : mPluginSettings(std::make_shared<Settings>()) {

  // Create default logger for this plugin.
  spdlog::set_default_logger(cs::utils::logger::createLogger("csp-atmospheres"));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::init() {

  spdlog::info("Loading plugin...");

  mAllSettings->onLoad().connect([this]() { onLoad(); });
  mAllSettings->onSave().connect(
      [this]() { to_json(mAllSettings->mPlugins.at("csp-atmospheres"), *mPluginSettings); });

  mGuiManager->addSettingsSectionToSideBarFromHTML(
      "Atmospheres", "blur_circular", "../share/resources/gui/atmospheres_settings.html");
  mGuiManager->addScriptToGuiFromJS("../share/resources/gui/js/csp-atmospheres.js");

  mGuiManager->getGui()->registerCallback("atmosphere.setEnableWater",
      "Enables or disables rendering of a water surface.",
      std::function([this](bool enable) { mPluginSettings->mEnableWater = enable; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setEnableClouds",
      "Enables or disables rendering of a cloud layer.",
      std::function([this](bool enable) { mPluginSettings->mEnableClouds = enable; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setEnable",
      "Enables or disables rendering of atmospheres.",
      std::function([this](bool value) { mPluginSettings->mEnabled = value; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setEnableLightShafts",
      "If shadows are enabled, this enables or disables rendering of light shafts in the "
      "atmosphere.",
      std::function([this](bool value) { mPluginSettings->mEnableLightShafts = value; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setQuality",
      "Higher values create a more realistic atmosphere.",
      std::function([this](double value) { mPluginSettings->mQuality = value; }));

  mGuiManager->getGui()->registerCallback("atmosphere.setWaterLevel",
      "Sets the height of the water surface relative to the planet's radius.",
      std::function([this](double value) { mPluginSettings->mWaterLevel = value; }));

  mEnableShadowsConnection = mAllSettings->mGraphics.pEnableShadows.connect([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      if (mAllSettings->mGraphics.pEnableShadows.get() &&
          mPluginSettings->mEnableLightShafts.get()) {
        atmosphere->getRenderer().setShadowMap(mGraphicsEngine->getShadowMap());
      } else {
        atmosphere->getRenderer().setShadowMap(nullptr);
      }
    }
  });

  mEnableHDRConnection = mAllSettings->mGraphics.pEnableHDR.connect([this](bool val) {
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
      mAllSettings->mGraphics.pAmbientBrightness.connect([this](float val) {
        for (auto const& atmosphere : mAtmospheres) {
          atmosphere->getRenderer().setAmbientBrightness(val * 0.4f);
        }
      });

  mPluginSettings->mEnableLightShafts.connect([this](bool val) {
    for (auto const& atmosphere : mAtmospheres) {
      if (mAllSettings->mGraphics.pEnableShadows.get() &&
          mPluginSettings->mEnableLightShafts.get()) {
        atmosphere->getRenderer().setShadowMap(mGraphicsEngine->getShadowMap());
      } else {
        atmosphere->getRenderer().setShadowMap(nullptr);
      }
    }
  });

  // Load settings.
  onLoad();

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

  mAllSettings->mGraphics.pEnableShadows.disconnect(mEnableShadowsConnection);
  mAllSettings->mGraphics.pEnableHDR.disconnect(mEnableHDRConnection);
  mAllSettings->mGraphics.pAmbientBrightness.disconnect(mAmbientBrightnessConnection);

  spdlog::info("Unloading done.");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::update() {
  float fIntensity = 1.f;
  for (auto const& atmosphere : mAtmospheres) {
    if (mPluginSettings->mEnabled.get()) {
      float brightness = atmosphere->getRenderer().getApproximateSceneBrightness();
      fIntensity *= (1.f - brightness);
    }
  }

  mGraphicsEngine->pApproximateSceneBrightness = fIntensity;

  for (auto const& atmosphere : mAtmospheres) {
    double sunIlluminance = 10.0;

    if (mAllSettings->mGraphics.pEnableHDR.get()) {
      sunIlluminance = mSolarSystem->getSunIlluminance(atmosphere->getWorldTransform()[3]);
    }

    auto sunDirection = mSolarSystem->getSunDirection(atmosphere->getWorldTransform()[3]);

    atmosphere->getRenderer().setSun(sunDirection, sunIlluminance);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::onLoad() {
  from_json(mAllSettings->mPlugins.at("csp-atmospheres"), *mPluginSettings);

  // For now, we simply reload all atmospheres. This can definitely optimized by re-using existing
  // atmospheres.
  for (auto const& atmosphere : mAtmospheres) {
    mSolarSystem->unregisterAnchor(atmosphere);
  }
  mAtmospheres.clear();

  for (auto const& atmoSettings : mPluginSettings->mAtmospheres) {
    auto anchor = mAllSettings->mAnchors.find(atmoSettings.first);

    if (anchor == mAllSettings->mAnchors.end()) {
      throw std::runtime_error(
          "There is no Anchor \"" + atmoSettings.first + "\" defined in the settings.");
    }

    auto [tStartExistence, tEndExistence] = anchor->second.getExistence();

    auto atmosphere = std::make_shared<Atmosphere>(mPluginSettings, anchor->second.mCenter,
        anchor->second.mFrame, tStartExistence, tEndExistence);

    atmosphere->getRenderer().setHDRBuffer(mGraphicsEngine->getHDRBuffer());
    atmosphere->configure(atmoSettings.second);

    mSolarSystem->registerAnchor(atmosphere);

    mAtmospheres.emplace_back(atmosphere);
  }

  mAllSettings->mGraphics.pEnableShadows.touch(mEnableShadowsConnection);
  mAllSettings->mGraphics.pEnableHDR.touch(mEnableHDRConnection);
  mAllSettings->mGraphics.pAmbientBrightness.touch(mAmbientBrightnessConnection);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
