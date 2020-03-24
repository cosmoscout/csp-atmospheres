////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Atmosphere.hpp"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include "../../../src/cs-core/GraphicsEngine.hpp"
#include "../../../src/cs-core/SolarSystem.hpp"
#include "../../../src/cs-graphics/Shadows.hpp"
#include "../../../src/cs-graphics/TextureLoader.hpp"
#include "../../../src/cs-utils/FrameTimings.hpp"
#include "../../../src/cs-utils/utils.hpp"

#include <VistaKernel/DisplayManager/VistaDisplayManager.h>
#include <VistaKernel/DisplayManager/VistaProjection.h>
#include <VistaKernel/DisplayManager/VistaViewport.h>
#include <VistaKernel/GraphicsManager/VistaGeometryFactory.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaMath/VistaBoundingBox.h>
#include <VistaOGLExt/Rendering/VistaGeometryData.h>
#include <VistaOGLExt/Rendering/VistaGeometryRenderingCore.h>
#include <VistaOGLExt/VistaOGLUtils.h>
#include <VistaOGLExt/VistaTexture.h>
#include <VistaTools/tinyXML/tinyxml.h>

#include <glm/gtc/type_ptr.hpp>

namespace csp::atmospheres {

AtmosphereRenderer::AtmosphereRenderer(std::shared_ptr<Plugin::Settings> const& pSettings)
    : mPluginSettings(pSettings) {

  initData();

  // scene-wide settings -----------------------------------------------------
  mPluginSettings->mQuality.connectAndTouch([this](int val) { setPrimaryRaySteps(val); });

  mPluginSettings->mEnableWater.connectAndTouch([this](bool val) { setDrawWater(val); });

  mPluginSettings->mEnableClouds.connectAndTouch([this](bool val) {
    if (mUseClouds != val) {
      mShaderDirty = true;
      mUseClouds   = val;
    }
  });

  mPluginSettings->mWaterLevel.connectAndTouch([this](float val) { setWaterLevel(val / 1000); });
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setSun(glm::vec3 const& direction, float illuminance) {
  mSunIntensity = illuminance;
  mSunDirection = direction;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setRadii(glm::dvec3 const& radii) {
  mRadii = radii;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setWorldTransform(glm::dmat4 const& transform) {
  mWorldTransform = transform;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setClouds(std::string const& textureFile, float height) {

  if (mCloudTextureFile != textureFile) {
    mCloudTextureFile = textureFile;
    mCloudTexture.reset();
    if (textureFile != "") {
      mCloudTexture = cs::graphics::TextureLoader::loadFromFile(textureFile);
    }
    mShaderDirty = true;
    mUseClouds   = mCloudTexture != nullptr;
  }

  mCloudHeight = height;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setShadowMap(std::shared_ptr<cs::graphics::ShadowMap> const& pShadowMap) {
  if (mShadowMap != pShadowMap) {
    mShadowMap   = pShadowMap;
    mShaderDirty = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setHDRBuffer(std::shared_ptr<cs::graphics::HDRBuffer> const& pHDRBuffer) {
  if (mHDRBuffer != pHDRBuffer) {
    mHDRBuffer   = pHDRBuffer;
    mShaderDirty = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float AtmosphereRenderer::getApproximateSceneBrightness() const {
  return mApproximateBrightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int AtmosphereRenderer::getPrimaryRaySteps() const {
  return mPrimaryRaySteps;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setPrimaryRaySteps(int iValue) {
  mPrimaryRaySteps = iValue;
  mShaderDirty     = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int AtmosphereRenderer::getSecondaryRaySteps() const {
  return mSecondaryRaySteps;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setSecondaryRaySteps(int iValue) {
  mSecondaryRaySteps = iValue;
  mShaderDirty       = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double AtmosphereRenderer::getAtmosphereHeight() const {
  return mAtmosphereHeight;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setAtmosphereHeight(double dValue) {
  mAtmosphereHeight = dValue;
  mShaderDirty      = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double AtmosphereRenderer::getMieHeight() const {
  return mMieHeight;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setMieHeight(double dValue) {
  mMieHeight   = dValue;
  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

glm::vec3 AtmosphereRenderer::getMieScattering() const {
  return mMieScattering;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setMieScattering(const glm::vec3& vValue) {
  mMieScattering = vValue;
  mShaderDirty   = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double AtmosphereRenderer::getMieAnisotropy() const {
  return mMieAnisotropy;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setMieAnisotropy(double dValue) {
  mMieAnisotropy = dValue;
  mShaderDirty   = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double AtmosphereRenderer::getRayleighHeight() const {
  return mRayleighHeight;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setRayleighHeight(double dValue) {
  mRayleighHeight = dValue;
  mShaderDirty    = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

glm::vec3 AtmosphereRenderer::getRayleighScattering() const {
  return mRayleighScattering;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setRayleighScattering(const glm::vec3& vValue) {
  mRayleighScattering = vValue;
  mShaderDirty        = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double AtmosphereRenderer::getRayleighAnisotropy() const {
  return mRayleighAnisotropy;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setRayleighAnisotropy(double dValue) {
  mRayleighAnisotropy = dValue;
  mShaderDirty        = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::getDrawSun() const {
  return mDrawSun;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setDrawSun(bool bEnable) {
  mDrawSun     = bEnable;
  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::getDrawWater() const {
  return mDrawWater;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setDrawWater(bool bEnable) {
  mDrawWater   = bEnable;
  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float AtmosphereRenderer::getWaterLevel() const {
  return mWaterLevel;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setWaterLevel(float fValue) {
  mWaterLevel = fValue;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float AtmosphereRenderer::getAmbientBrightness() const {
  return mAmbientBrightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setAmbientBrightness(float fValue) {
  mAmbientBrightness = fValue;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::getUseToneMapping() const {
  return mUseToneMapping;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setUseToneMapping(bool bEnable, float fExposure, float fGamma) {
  mUseToneMapping = bEnable;
  mExposure       = fExposure;
  mGamma          = fGamma;
  mShaderDirty    = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::getUseLinearDepthBuffer() const {
  return mUseLinearDepthBuffer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::setUseLinearDepthBuffer(bool bEnable) {
  mUseLinearDepthBuffer = bEnable;
  mShaderDirty          = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::updateShader() {
  mAtmoShader = VistaGLSLShader();

  std::string sVert(cAtmosphereVert);
  std::string sFrag(cAtmosphereFrag0 + cAtmosphereFrag1);

  cs::utils::replaceString(sFrag, "HEIGHT_ATMO", cs::utils::toString(mAtmosphereHeight));
  cs::utils::replaceString(sFrag, "ANISOTROPY_R", cs::utils::toString(mRayleighAnisotropy));
  cs::utils::replaceString(sFrag, "ANISOTROPY_M", cs::utils::toString(mMieAnisotropy));
  cs::utils::replaceString(sFrag, "HEIGHT_R", cs::utils::toString(mRayleighHeight));
  cs::utils::replaceString(sFrag, "HEIGHT_M", cs::utils::toString(mMieHeight));
  cs::utils::replaceString(sFrag, "BETA_R_0", cs::utils::toString(mRayleighScattering[0]));
  cs::utils::replaceString(sFrag, "BETA_R_1", cs::utils::toString(mRayleighScattering[1]));
  cs::utils::replaceString(sFrag, "BETA_R_2", cs::utils::toString(mRayleighScattering[2]));
  cs::utils::replaceString(sFrag, "BETA_M_0", cs::utils::toString(mMieScattering[0]));
  cs::utils::replaceString(sFrag, "BETA_M_1", cs::utils::toString(mMieScattering[1]));
  cs::utils::replaceString(sFrag, "BETA_M_2", cs::utils::toString(mMieScattering[2]));
  cs::utils::replaceString(sFrag, "PRIMARY_RAY_STEPS", cs::utils::toString(mPrimaryRaySteps));
  cs::utils::replaceString(sFrag, "SECONDARY_RAY_STEPS", cs::utils::toString(mSecondaryRaySteps));
  cs::utils::replaceString(sFrag, "ENABLE_TONEMAPPING", mUseToneMapping ? "1" : "0");
  cs::utils::replaceString(sFrag, "EXPOSURE", cs::utils::toString(mExposure));
  cs::utils::replaceString(sFrag, "GAMMA", cs::utils::toString(mGamma));
  cs::utils::replaceString(sFrag, "HEIGHT_ATMO", cs::utils::toString(mAtmosphereHeight));
  cs::utils::replaceString(sFrag, "USE_LINEARDEPTHBUFFER", mUseLinearDepthBuffer ? "1" : "0");
  cs::utils::replaceString(sFrag, "DRAW_SUN", mDrawSun ? "1" : "0");
  cs::utils::replaceString(sFrag, "DRAW_WATER", mDrawWater ? "1" : "0");
  cs::utils::replaceString(sFrag, "USE_SHADOWMAP", (mShadowMap != nullptr) ? "1" : "0");
  cs::utils::replaceString(sFrag, "USE_CLOUDMAP", (mUseClouds && mCloudTexture) ? "1" : "0");
  cs::utils::replaceString(sFrag, "ENABLE_HDR", mHDRBuffer ? "1" : "0");

  mAtmoShader.InitVertexShaderFromString(sVert);
  mAtmoShader.InitFragmentShaderFromString(sFrag);

  mAtmoShader.Link();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::Do() {
  cs::utils::FrameTimings::ScopedTimer timer("csp-atmospheres");

  if (mShaderDirty) {
    updateShader();
    mShaderDirty = false;
  }

  // save current lighting and meterial state of the OpenGL state machine ----
  glPushAttrib(GL_LIGHTING_BIT | GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);
  glCullFace(GL_FRONT);
  glDepthMask(GL_FALSE);

  // copy depth buffer -------------------------------------------------------
  if (!mHDRBuffer) {
    GLint iViewport[4];
    glGetIntegerv(GL_VIEWPORT, iViewport);

    auto viewport    = GetVistaSystem()->GetDisplayManager()->GetCurrentRenderInfo()->m_pViewport;
    auto const& data = mGBufferData[viewport];

    data.mDepthBuffer->Bind();
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, iViewport[0], iViewport[1], iViewport[2],
        iViewport[3], 0);
    data.mColorBuffer->Bind();
    glCopyTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGB8, iViewport[0], iViewport[1], iViewport[2], iViewport[3], 0);
  }

  // get matrices and related values -----------------------------------------
  GLfloat glMatMV[16];
  glGetFloatv(GL_MODELVIEW_MATRIX, &glMatMV[0]);
  glm::mat4 matMV(glm::make_mat4x4(glMatMV) * glm::mat4(mWorldTransform) *
                  glm::mat4((float)(mRadii[0] / (1.0 - mAtmosphereHeight)), 0, 0, 0, 0,
                      (float)(mRadii[0] / (1.0 - mAtmosphereHeight)), 0, 0, 0, 0,
                      (float)(mRadii[0] / (1.0 - mAtmosphereHeight)), 0, 0, 0, 0, 1));

  auto matInvMV = glm::inverse(matMV);

  GLfloat glMatP[16];
  glGetFloatv(GL_PROJECTION_MATRIX, &glMatP[0]);
  glm::mat4 matInvP = glm::inverse(glm::make_mat4x4(glMatP));
  glm::mat4 matInvMVP(matInvMV * matInvP);

  glm::vec3 sunDir =
      glm::normalize(glm::vec3(glm::inverse(mWorldTransform) * glm::vec4(mSunDirection, 0)));

  // set uniforms ------------------------------------------------------------
  mAtmoShader.Bind();

  double nearClip, farClip;
  GetVistaSystem()
      ->GetDisplayManager()
      ->GetCurrentRenderInfo()
      ->m_pViewport->GetProjection()
      ->GetProjectionProperties()
      ->GetClippingRange(nearClip, farClip);

  mAtmoShader.SetUniform(mAtmoShader.GetUniformLocation("uSunIntensity"), mSunIntensity);
  mAtmoShader.SetUniform(
      mAtmoShader.GetUniformLocation("uSunDir"), sunDir[0], sunDir[1], sunDir[2]);
  mAtmoShader.SetUniform(mAtmoShader.GetUniformLocation("uFarClip"), (float)farClip);

  mAtmoShader.SetUniform(mAtmoShader.GetUniformLocation("uWaterLevel"), mWaterLevel);
  mAtmoShader.SetUniform(mAtmoShader.GetUniformLocation("uAmbientBrightness"), mAmbientBrightness);

  if (mHDRBuffer) {
    mHDRBuffer->doPingPong();
    mHDRBuffer->bind();
    mHDRBuffer->getDepthAttachment()->Bind(GL_TEXTURE0);
    mHDRBuffer->getCurrentReadAttachment()->Bind(GL_TEXTURE1);
  } else {
    auto viewport    = GetVistaSystem()->GetDisplayManager()->GetCurrentRenderInfo()->m_pViewport;
    auto const& data = mGBufferData[viewport];
    data.mDepthBuffer->Bind(GL_TEXTURE0);
    data.mColorBuffer->Bind(GL_TEXTURE1);
  }

  mAtmoShader.SetUniform(mAtmoShader.GetUniformLocation("uDepthBuffer"), 0);
  mAtmoShader.SetUniform(mAtmoShader.GetUniformLocation("uColorBuffer"), 1);

  if (mUseClouds && mCloudTexture) {
    mCloudTexture->Bind(GL_TEXTURE3);
    mAtmoShader.SetUniform(mAtmoShader.GetUniformLocation("uCloudTexture"), 3);
    mAtmoShader.SetUniform(mAtmoShader.GetUniformLocation("uCloudAltitude"), mCloudHeight);
  }

  if (mShadowMap) {
    int texUnitShadow = 4;
    mAtmoShader.SetUniform(
        mAtmoShader.GetUniformLocation("uShadowCascades"), (int)mShadowMap->getMaps().size());
    for (int i = 0; i < mShadowMap->getMaps().size(); ++i) {
      GLint locSamplers = glGetUniformLocation(
          mAtmoShader.GetProgram(), ("uShadowMaps[" + std::to_string(i) + "]").c_str());
      GLint locMatrices = glGetUniformLocation(mAtmoShader.GetProgram(),
          ("uShadowProjectionViewMatrices[" + std::to_string(i) + "]").c_str());

      mShadowMap->getMaps()[i]->Bind((GLenum)GL_TEXTURE0 + texUnitShadow + i);
      glUniform1i(locSamplers, texUnitShadow + i);

      auto mat = mShadowMap->getShadowMatrices()[i];
      glUniformMatrix4fv(locMatrices, 1, GL_FALSE, mat.GetData());
    }
  }

  // Why is there no set uniform for matrices???
  GLint loc = mAtmoShader.GetUniformLocation("uMatInvMV");
  glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(matInvMV));
  loc = mAtmoShader.GetUniformLocation("uMatInvMVP");
  glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(matInvMVP));
  loc = mAtmoShader.GetUniformLocation("uMatInvP");
  glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(matInvP));
  loc = mAtmoShader.GetUniformLocation("uMatMV");
  glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(matMV));

  // draw --------------------------------------------------------------------
  mQuadVAO.Bind();
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  mQuadVAO.Release();

  // clean up ----------------------------------------------------------------

  if (mHDRBuffer) {
    mHDRBuffer->getDepthAttachment()->Unbind(GL_TEXTURE0);
    mHDRBuffer->getCurrentReadAttachment()->Unbind(GL_TEXTURE1);
  } else {
    auto viewport    = GetVistaSystem()->GetDisplayManager()->GetCurrentRenderInfo()->m_pViewport;
    auto const& data = mGBufferData[viewport];
    data.mDepthBuffer->Unbind(GL_TEXTURE0);
    data.mColorBuffer->Unbind(GL_TEXTURE1);
  }

  if (mUseClouds && mCloudTexture) {
    mCloudTexture->Unbind(GL_TEXTURE3);
  }

  mAtmoShader.Release();

  glDepthMask(GL_TRUE);

  glPopAttrib();

  // update brightness value -------------------------------------------------
  // This is a crude approximation of the overall scene brightness due to
  // atmospheric scattering, camera position and the sun's position.
  // It may be used for fake HDR effects such as dimming stars.

  // some required positions and directions
  glm::vec4 temp            = matInvMVP * glm::vec4(0, 0, 0, 1);
  glm::vec3 vCamera         = glm::vec3(temp) / temp[3];
  glm::vec3 vPlanet         = glm::vec3(0, 0, 0);
  glm::vec3 vCameraToPlanet = glm::normalize(vCamera - vPlanet);

  // [planet surface ... 5x atmosphere boundary] -> [0 ... 1]
  float fHeightInAtmosphere = (float)std::min(1.0,
      std::max(0.0, (glm::length(vCamera) - (1.f - mAtmosphereHeight)) / (mAtmosphereHeight * 5)));

  // [noon ... midnight] -> [1 ... -1]
  float fDaySide = glm::dot(vCameraToPlanet, sunDir);

  // limit brightness when on night side (also in dusk an dawn time)
  float fBrightnessOnSurface = std::pow(std::min(1.f, std::max(0.f, fDaySide + 1.f)), 50.f);

  // reduce brightness in outer space
  mApproximateBrightness = (1.f - fHeightInAtmosphere) * fBrightnessOnSurface;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool AtmosphereRenderer::GetBoundingBox(VistaBoundingBox& bb) {
  // Boundingbox is computed by translation an edge points
  float fMin[3] = {(float)(-1.0 / (1.0 - mAtmosphereHeight)),
      (float)(-1.0 / (1.0 - mAtmosphereHeight)), (float)(-1.0 / (1.0 - mAtmosphereHeight))};
  float fMax[3] = {(float)(1.0 / (1.0 - mAtmosphereHeight)),
      (float)(1.0 / (1.0 - mAtmosphereHeight)), (float)(1.0 / (1.0 - mAtmosphereHeight))};

  bb.SetBounds(fMin, fMax);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void AtmosphereRenderer::initData() {
  // create quad -------------------------------------------------------------
  std::vector<float> data(8);
  data[0] = -1;
  data[1] = 1;
  data[2] = 1;
  data[3] = 1;
  data[4] = -1;
  data[5] = -1;
  data[6] = 1;
  data[7] = -1;

  mQuadVBO.Bind(GL_ARRAY_BUFFER);
  mQuadVBO.BufferData(data.size() * sizeof(float), &(data[0]), GL_STATIC_DRAW);
  mQuadVBO.Release();

  // positions
  mQuadVAO.EnableAttributeArray(0);
  mQuadVAO.SpecifyAttributeArrayFloat(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0, &mQuadVBO);

  // create textures ---------------------------------------------------------
  for (auto const& viewport : GetVistaSystem()->GetDisplayManager()->GetViewports()) {
    GBufferData bufferData;

    bufferData.mDepthBuffer = std::make_unique<VistaTexture>(GL_TEXTURE_2D);
    bufferData.mDepthBuffer->Bind();
    bufferData.mDepthBuffer->SetWrapS(GL_CLAMP);
    bufferData.mDepthBuffer->SetWrapT(GL_CLAMP);
    bufferData.mDepthBuffer->SetMinFilter(GL_NEAREST);
    bufferData.mDepthBuffer->SetMagFilter(GL_NEAREST);
    bufferData.mDepthBuffer->Unbind();

    bufferData.mColorBuffer = std::make_unique<VistaTexture>(GL_TEXTURE_2D);
    bufferData.mColorBuffer->Bind();
    bufferData.mColorBuffer->SetWrapS(GL_CLAMP);
    bufferData.mColorBuffer->SetWrapT(GL_CLAMP);
    bufferData.mColorBuffer->SetMinFilter(GL_NEAREST);
    bufferData.mColorBuffer->SetMagFilter(GL_NEAREST);
    bufferData.mColorBuffer->Unbind();

    mGBufferData.emplace(viewport.second, std::move(bufferData));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
