////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Plugin.hpp"

#include "Atmosphere.hpp"

#include "../../../src/cs-core/GraphicsEngine.hpp"
#include "../../../src/cs-core/GuiManager.hpp"
#include "../../../src/cs-core/MessageBus.hpp"
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
  o.mSunIntensity        = cs::core::parseProperty<double>("sunIntensity", j);

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

    auto atmosphere = std::make_shared<Atmosphere>(mGraphicsEngine, mProperties,
        anchor->second.mCenter, anchor->second.mFrame, tStartExistence, tEndExistence);

    mSolarSystem->registerAnchor(atmosphere);

    if (atmoSettings.second.mCloudTexture) {
      atmosphere->setCloudTexture(
          cs::graphics::TextureLoader::loadFromFile(*atmoSettings.second.mCloudTexture),
          *atmoSettings.second.mCloudHeight);
    }

    atmosphere->getAtmosphere().setAtmosphereHeight(atmoSettings.second.mAtmosphereHeight);
    atmosphere->getAtmosphere().setMieHeight(atmoSettings.second.mMieHeight);
    atmosphere->getAtmosphere().setMieScattering(VistaVector3D(
        (float)atmoSettings.second.mMieScatteringR, (float)atmoSettings.second.mMieScatteringG,
        (float)atmoSettings.second.mMieScatteringB));
    atmosphere->getAtmosphere().setMieAnisotropy(atmoSettings.second.mMieAnisotropy);
    atmosphere->getAtmosphere().setRayleighHeight(atmoSettings.second.mRayleighHeight);
    atmosphere->getAtmosphere().setRayleighScattering(
        VistaVector3D((float)atmoSettings.second.mRayleighScatteringR,
            (float)atmoSettings.second.mRayleighScatteringG,
            (float)atmoSettings.second.mRayleighScatteringB));
    atmosphere->getAtmosphere().setRayleighAnisotropy(atmoSettings.second.mRayleighAnisotropy);
    atmosphere->getAtmosphere().setSunIntensity((float)atmoSettings.second.mSunIntensity);

    VistaOpenGLNode* atmosphereNode =
        mSceneGraph->NewOpenGLNode(mSceneGraph->GetRoot(), atmosphere.get());
    VistaOpenSGMaterialTools::SetSortKeyOnSubtree(
        atmosphereNode, static_cast<int>(cs::utils::DrawOrder::eAtmospheres));
    atmosphere->setSun(mSolarSystem->getSun());

    mAtmosphereNodes.push_back(atmosphereNode);
    mAtmospheres.push_back(atmosphere);
  }

  mGuiManager->getGui()->registerCallback<bool>("set_enable_water", ([this](bool enable) {
    mProperties->mEnableWater = enable;
    mMessageBus->send({cs::core::MessageBus::Response::Type::eChanged, "csp::atmospheres", "water",
        std::to_string(enable)});
  }));

  mGuiManager->getGui()->registerCallback<bool>("set_enable_clouds", ([this](bool enable) {
    mProperties->mEnableClouds = enable;
    mMessageBus->send({cs::core::MessageBus::Response::Type::eChanged, "csp::atmospheres", "clouds",
        std::to_string(enable)});
  }));

  mGuiManager->getGui()->registerCallback<bool>("set_enable_atmosphere", ([this](bool enable) {
    mProperties->mEnabled = enable;
    mMessageBus->send({cs::core::MessageBus::Response::Type::eChanged, "csp::atmospheres",
        "atmosphere", std::to_string(enable)});
  }));

  mGuiManager->getGui()->registerCallback<bool>("set_enable_light_shafts", ([this](bool enable) {
    mProperties->mEnableLightShafts = enable;
    mMessageBus->send({cs::core::MessageBus::Response::Type::eChanged, "csp::atmospheres",
        "light_shafts", std::to_string(enable)});
  }));

  mGuiManager->getGui()->registerCallback<double>(
      "set_atmosphere_quality", ([this](const int value) {
        mProperties->mQuality = value;
        mMessageBus->send({cs::core::MessageBus::Response::Type::eChanged, "csp::atmospheres",
            "atmosphere_quality", std::to_string(value)});
      }));

  mGuiManager->getGui()->registerCallback<double>("set_water_level", ([this](double value) {
    mProperties->mWaterLevel = value;
    mMessageBus->send({cs::core::MessageBus::Response::Type::eChanged, "csp::atmospheres",
        "water_level", std::to_string(value)});
  }));

  mMessageBus->onRequest().connect([this](cs::core::MessageBus::Request const& request) {
    std::string scope = "csp::atmospheres";
    if (request.mReceiver != scope) {
      return;
    }

    if (request.mType == cs::core::MessageBus::Request::Type::eGet) {
      cs::core::MessageBus::Response response;
      response.mType  = cs::core::MessageBus::Response::Type::eInfo;
      response.mSender = scope;
      response.mName  = request.mName;

      if (request.mName == "water") {
        response.mData = std::to_string(mProperties->mEnableWater.get());
      } else if (request.mName == "clouds") {
        response.mData = std::to_string(mProperties->mEnableClouds.get());
      } else if (request.mName == "atmosphere") {
        response.mData = std::to_string(mProperties->mEnabled.get());
      } else if (request.mName == "light_shafts") {
        response.mData = std::to_string(mProperties->mEnableLightShafts.get());
      } else if (request.mName == "atmosphere_quality") {
        response.mData = std::to_string(mProperties->mQuality.get());
      } else if (request.mName == "water_level") {
        response.mData = std::to_string(mProperties->mWaterLevel.get());
      } else {
        return;
      }

      mMessageBus->send(response);
    } else if (request.mType == cs::core::MessageBus::Request::Type::eSet) {
      cs::core::MessageBus::Response response;
      response.mType  = cs::core::MessageBus::Response::Type::eChanged;
      response.mSender = scope;
      response.mName  = request.mName;

      if (request.mName == "water") {
        if (mProperties->mEnableWater.get() == std::stoi(request.mData)) {
          response.mType = cs::core::MessageBus::Response::Type::eInfo;
        } else {
          mGuiManager->getGui()->callJavascript(
              "CosmoScout.setCheckboxValue", "set_enable_water", request.mData);
          mProperties->mEnableWater.set(std::stoi(request.mData));
        }

        response.mData = std::to_string(mProperties->mEnableWater.get());
      } else if (request.mName == "clouds") {
        if (mProperties->mEnableClouds.get() == std::stoi(request.mData)) {
          response.mType = cs::core::MessageBus::Response::Type::eInfo;
        } else {
          mGuiManager->getGui()->callJavascript(
              "CosmoScout.setCheckboxValue", "set_enable_clouds", request.mData);
          mProperties->mEnableClouds.set(std::stoi(request.mData));
        }

        response.mData = std::to_string(mProperties->mEnableClouds.get());
      } else if (request.mName == "atmosphere") {
        if (mProperties->mEnabled.get() == std::stoi(request.mData)) {
          response.mType = cs::core::MessageBus::Response::Type::eInfo;
        } else {
          mGuiManager->getGui()->callJavascript(
              "CosmoScout.setCheckboxValue", "set_enable_atmosphere", request.mData);
          mProperties->mEnabled.set(std::stoi(request.mData));
        }

        response.mData = std::to_string(mProperties->mEnabled.get());
      } else if (request.mName == "light_shafts") {
        if (mProperties->mEnableLightShafts.get() == std::stoi(request.mData)) {
          response.mType = cs::core::MessageBus::Response::Type::eInfo;
        } else {
          mGuiManager->getGui()->callJavascript(
              "CosmoScout.setCheckboxValue", "set_enable_light_shafts", request.mData);
          mProperties->mEnableLightShafts.set(std::stoi(request.mData));
        }

        response.mData = std::to_string(mProperties->mEnableLightShafts.get());
      } else if (request.mName == "atmosphere_quality") {
        if (mProperties->mQuality.get() == std::stoi(request.mData)) {
          response.mType = cs::core::MessageBus::Response::Type::eInfo;
        } else {
          mGuiManager->getGui()->callJavascript(
              "CosmoScout.setSliderValue", "set_atmosphere_quality", std::stoi(request.mData));
          mProperties->mQuality.set(std::stoi(request.mData));
        }

        response.mData = std::to_string(mProperties->mQuality.get());
      } else if (request.mName == "water_level") {
        if (mProperties->mWaterLevel.get() == std::stof(request.mData)) {
          response.mType = cs::core::MessageBus::Response::Type::eInfo;
        } else {
          mGuiManager->getGui()->callJavascript(
              "CosmoScout.setSliderValue", "set_water_level", std::stof(request.mData));
          mProperties->mWaterLevel.set(std::stof(request.mData));
        }

        response.mData = std::to_string(mProperties->mWaterLevel.get());
      } else {
        return;
      }

      mMessageBus->send(response);
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

  mGuiManager->getGui()->unregisterCallback("set_enable_water");
  mGuiManager->getGui()->unregisterCallback("set_enable_clouds");
  mGuiManager->getGui()->unregisterCallback("set_enable_atmosphere");
  mGuiManager->getGui()->unregisterCallback("set_enable_light_shafts");
  mGuiManager->getGui()->unregisterCallback("set_atmosphere_quality");
  mGuiManager->getGui()->unregisterCallback("set_water_level");
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void Plugin::update() {
  float fIntensity = 1.f;
  for (auto const& atmosphere : mAtmospheres) {
    if (mProperties->mEnabled.get()) {
      float brightness = atmosphere->getAtmosphere().getApproximateSceneBrightness();
      fIntensity *= (1.f - brightness);
    }
  }

  mGraphicsEngine->pApproximateSceneBrightness = fIntensity;

  for (auto const& atmosphere : mAtmospheres) {
    atmosphere->setSun(mSolarSystem->getSun());
  }
}

    void Plugin::listenFor(std::string, cs::utils::Property<bool>) {

    }

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
