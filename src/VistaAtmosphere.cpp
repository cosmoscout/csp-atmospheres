////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "VistaAtmosphere.hpp"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include "../../../src/cs-graphics/Shadows.hpp"
#include "../../../src/cs-utils/utils.hpp"

#include <VistaKernel/DisplayManager/VistaDisplayManager.h>
#include <VistaKernel/DisplayManager/VistaProjection.h>
#include <VistaKernel/DisplayManager/VistaViewport.h>
#include <VistaKernel/GraphicsManager/VistaGeometryFactory.h>
#include <VistaKernel/GraphicsManager/VistaOpenGLNode.h>
#include <VistaKernel/GraphicsManager/VistaSceneGraph.h>
#include <VistaKernel/VistaSystem.h>
#include <VistaOGLExt/Rendering/VistaGeometryData.h>
#include <VistaOGLExt/Rendering/VistaGeometryRenderingCore.h>
#include <VistaOGLExt/VistaBufferObject.h>
#include <VistaOGLExt/VistaGLSLShader.h>
#include <VistaOGLExt/VistaOGLUtils.h>
#include <VistaOGLExt/VistaTexture.h>
#include <VistaOGLExt/VistaVertexArrayObject.h>
#include <VistaTools/tinyXML/tinyxml.h>

namespace csp::atmospheres {

////////////////////////////////////////////////////////////////////////////////////////////////////

VistaAtmosphere::VistaAtmosphere(Preset ePreset)
    : mTransmittanceTexture(new VistaTexture(GL_TEXTURE_2D))
    , mAtmoShader(new VistaGLSLShader())
    , mQuadVAO(new VistaVertexArrayObject())
    , mQuadVBO(new VistaBufferObject()) {
  initData();
  loadPreset(ePreset);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

VistaAtmosphere::VistaAtmosphere(const std::string& sConfigFile)
    : mTransmittanceTexture(new VistaTexture(GL_TEXTURE_2D))
    , mAtmoShader(new VistaGLSLShader())
    , mQuadVAO(new VistaVertexArrayObject())
    , mQuadVBO(new VistaBufferObject()) {
  initData();
  loadConfigFile(sConfigFile);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

VistaAtmosphere::~VistaAtmosphere() {
  delete mTransmittanceTexture;
  delete mAtmoShader;
  delete mQuadVAO;
  delete mQuadVBO;

  for (auto data : mGBufferData) {
    delete data.second.mDepthBuffer;
    delete data.second.mColorBuffer;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::loadPreset(Preset ePreset) {
  if (ePreset == Preset::eEarth) {
    mAtmosphereHeight = 80.0 / 6360.0;
    mMieHeight        = 1.2 / 6360.0;
    mMieScattering =
        VistaVector3D(21.0e-6f * 6360000.f, 21.0e-6f * 6360000.f, 21.0e-6f * 6360000.f);
    mMieAnisotropy  = 0.76;
    mRayleighHeight = 8.0 / 6360.0;
    mRayleighScattering =
        VistaVector3D(5.8e-6f * 6360000.f, 13.5e-6f * 6360000.f, 33.1e-6f * 6360000.f);
    mRayleighAnisotropy = 0;
    mSunIntensity       = 15;
  } else if (ePreset == Preset::eMars) {
    mAtmosphereHeight = 70.0 / 3460.0;
    mMieHeight        = 5.0 / 3460.0;
    mMieScattering =
        VistaVector3D(21.0e-6f * 3460000.f, 21.0e-6f * 3460000.f, 21.0e-6f * 3460000.f);
    mMieAnisotropy  = 0.76;
    mRayleighHeight = 11.0 / 3460.0;
    mRayleighScattering =
        VistaVector3D(20.0e-6f * 3460000.f, 13.5e-6f * 3460000.f, 5.75e-6f * 3460000.f);
    mRayleighAnisotropy = 0;
    mSunIntensity       = 15;
  } else if (ePreset == Preset::eSmog) {
    mAtmosphereHeight   = 70.0 / 2000.0;
    mMieHeight          = 10.0 / 2000.0;
    mMieScattering      = VistaVector3D(100.0, 100.0, 100.0);
    mMieAnisotropy      = 0.76;
    mRayleighHeight     = 15.0 / 2000.0;
    mRayleighScattering = VistaVector3D(5.8, 18.5, 33.1);
    mRayleighAnisotropy = 0;
    mSunIntensity       = 15;
  } else // if (ePreset == Preset::eBlue_paradise)
  {
    mAtmosphereHeight   = 0.05;
    mMieHeight          = 0.0005;
    mMieScattering      = VistaVector3D(10.0, 10.0, 10.0);
    mMieAnisotropy      = 0.7;
    mRayleighHeight     = 0.01;
    mRayleighScattering = VistaVector3D(5.8, 18.5, 33.1);
    mRayleighAnisotropy = 0;
    mSunIntensity       = 15;
  }

  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::loadConfigFile(const std::string& sConfigFile) {
  loadPreset(Preset::eEarth);

  VistaXML::TiXmlDocument xDoc(sConfigFile);

  if (!xDoc.LoadFile()) {
    std::cout << "Failed to load atmosphere config file " << sConfigFile << ": "
              << "Cannont open file!" << std::endl;
    return;
  }

  // Get First Element
  const VistaXML::TiXmlElement* pRoot(xDoc.FirstChildElement());

  // Read Data
  if (std::string(pRoot->Value()) != "AtmosphereConfig") {
    std::cout << "Failed to read atmosphere config file " << sConfigFile << "!" << std::endl;
    return;
  }

  const VistaXML::TiXmlElement* pProperty(pRoot->FirstChildElement());

  while (pProperty != nullptr) {
    if (std::string(pProperty->Value()) == "Property") {
      std::string       sName(pProperty->Attribute("Name"));
      std::stringstream ssValue(pProperty->Attribute("Value"));
      if (sName == "AtmosphereHeight") {
        ssValue >> mAtmosphereHeight;
      } else if (sName == "MieHeight") {
        ssValue >> mMieHeight;
      } else if (sName == "MieScatteringR") {
        ssValue >> mMieScattering[0];
      } else if (sName == "MieScatteringG") {
        ssValue >> mMieScattering[1];
      } else if (sName == "MieScatteringB") {
        ssValue >> mMieScattering[2];
      } else if (sName == "MieAnisotropy") {
        ssValue >> mMieAnisotropy;
      } else if (sName == "RayleighHeight") {
        ssValue >> mRayleighHeight;
      } else if (sName == "RayleighScatteringR") {
        ssValue >> mRayleighScattering[0];
      } else if (sName == "RayleighScatteringG") {
        ssValue >> mRayleighScattering[1];
      } else if (sName == "RayleighScatteringB") {
        ssValue >> mRayleighScattering[2];
      } else if (sName == "RayleighAnisotropy") {
        ssValue >> mRayleighAnisotropy;
      } else if (sName == "SunIntensity") {
        ssValue >> mSunIntensity;
      } else {
        std::cout << "Ignoring invalid entity " << sName << " while reading star config file "
                  << sConfigFile << "!" << std::endl;
      }
    } else {
      std::cout << "Ignoring invalid entity " << pProperty->Value()
                << " while reading star config file " << sConfigFile << "!" << std::endl;
    }

    pProperty = pProperty->NextSiblingElement();
  }

  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setShadowMap(cs::graphics::ShadowMap const* pShadowMap) {
  if (mShadowMap != pShadowMap) {
    mShadowMap   = pShadowMap;
    mShaderDirty = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setCloudTexture(VistaTexture* pTexture, float fAltitude) {
  if (mCloudTexture != pTexture) {
    mCloudTexture  = pTexture;
    mCloudAltitude = fAltitude;
    mShaderDirty   = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float VistaAtmosphere::getApproximateSceneBrightness() const {
  return mApproximateBrightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int VistaAtmosphere::getPrimaryRaySteps() const {
  return mPrimaryRaySteps;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setPrimaryRaySteps(int iValue) {
  mPrimaryRaySteps = iValue;
  mShaderDirty     = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int VistaAtmosphere::getSecondaryRaySteps() const {
  return mSecondaryRaySteps;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setSecondaryRaySteps(int iValue) {
  mSecondaryRaySteps = iValue;
  mShaderDirty       = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

VistaVector3D VistaAtmosphere::getSunDirection() const {
  return mSunDirection;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setSunDirection(const VistaVector3D& vValue) {
  mSunDirection = VistaVector3D(vValue[0], vValue[1], vValue[2], 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double VistaAtmosphere::getAtmosphereHeight() const {
  return mAtmosphereHeight;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setAtmosphereHeight(double dValue) {
  mAtmosphereHeight = dValue;
  mShaderDirty      = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float VistaAtmosphere::getSunIntensity() const {
  return mSunIntensity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setSunIntensity(float fValue) {
  mSunIntensity = fValue;
  mShaderDirty  = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double VistaAtmosphere::getMieHeight() const {
  return mMieHeight;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setMieHeight(double dValue) {
  mMieHeight   = dValue;
  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

VistaVector3D VistaAtmosphere::getMieScattering() const {
  return mMieScattering;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setMieScattering(const VistaVector3D& vValue) {
  mMieScattering = vValue;
  mShaderDirty   = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double VistaAtmosphere::getMieAnisotropy() const {
  return mMieAnisotropy;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setMieAnisotropy(double dValue) {
  mMieAnisotropy = dValue;
  mShaderDirty   = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double VistaAtmosphere::getRayleighHeight() const {
  return mRayleighHeight;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setRayleighHeight(double dValue) {
  mRayleighHeight = dValue;
  mShaderDirty    = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

VistaVector3D VistaAtmosphere::getRayleighScattering() const {
  return mRayleighScattering;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setRayleighScattering(const VistaVector3D& vValue) {
  mRayleighScattering = vValue;
  mShaderDirty        = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

double VistaAtmosphere::getRayleighAnisotropy() const {
  return mRayleighAnisotropy;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setRayleighAnisotropy(double dValue) {
  mRayleighAnisotropy = dValue;
  mShaderDirty        = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool VistaAtmosphere::getDrawSun() const {
  return mDrawSun;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setDrawSun(bool bEnable) {
  mDrawSun     = bEnable;
  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool VistaAtmosphere::getDrawWater() const {
  return mDrawWater;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setDrawWater(bool bEnable) {
  mDrawWater   = bEnable;
  mShaderDirty = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float VistaAtmosphere::getWaterLevel() const {
  return mWaterLevel;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setWaterLevel(float fValue) {
  mWaterLevel = fValue;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

float VistaAtmosphere::getAmbientBrightness() const {
  return mAmbientBrightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setAmbientBrightness(float fValue) {
  mAmbientBrightness = fValue;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool VistaAtmosphere::getUseToneMapping() const {
  return mUseToneMapping;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setUseToneMapping(bool bEnable, float fExposure, float fGamma) {
  mUseToneMapping = bEnable;
  mExposure       = fExposure;
  mGamma          = fGamma;
  mShaderDirty    = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool VistaAtmosphere::getUseLinearDepthBuffer() const {
  return mUseLinearDepthBuffer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::setUseLinearDepthBuffer(bool bEnable) {
  mUseLinearDepthBuffer = bEnable;
  mShaderDirty          = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::updateShader() {
  if (mAtmoShader) {
    delete mAtmoShader;
  }

  mAtmoShader = new VistaGLSLShader();

  std::string sVert(cAtmosphereVert);
  std::string sFrag(cAtmosphereFrag0 + cAtmosphereFrag1);

  cs::utils::replaceString(sFrag, "HEIGHT_ATMO", cs::utils::toString(mAtmosphereHeight));
  cs::utils::replaceString(sFrag, "SUN_INTENSITY", cs::utils::toString(mSunIntensity));
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
  cs::utils::replaceString(sFrag, "USE_CLOUDMAP", (mCloudTexture != nullptr) ? "1" : "0");

  mAtmoShader->InitVertexShaderFromString(sVert);
  mAtmoShader->InitFragmentShaderFromString(sFrag);

  mAtmoShader->Link();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::draw(VistaTransformMatrix const& matModelMatrix) {
  if (mShaderDirty) {
    updateShader();
    mShaderDirty = false;
  }

  if (mTransmittanceTextureDirty) {
    updateTransmittanceTexture();
    mTransmittanceTextureDirty = false;
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
  GLint iViewport[4];
  glGetIntegerv(GL_VIEWPORT, iViewport);

  auto        viewport = GetVistaSystem()->GetDisplayManager()->GetCurrentRenderInfo()->m_pViewport;
  auto const& data     = mGBufferData[viewport];

  data.mDepthBuffer->Bind();
  glCopyTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_DEPTH_COMPONENT, iViewport[0], iViewport[1],
      iViewport[2], iViewport[3], 0);
  data.mColorBuffer->Bind();
  glCopyTexImage2D(
      GL_TEXTURE_RECTANGLE, 0, GL_RGB8, iViewport[0], iViewport[1], iViewport[2], iViewport[3], 0);

  // get matrices and related values -----------------------------------------
  GLfloat glMatMV[16];
  glGetFloatv(GL_MODELVIEW_MATRIX, &glMatMV[0]);
  VistaTransformMatrix matMV(VistaTransformMatrix(glMatMV, true) * matModelMatrix *
                             VistaTransformMatrix((float)(1.0 / (1.0 - mAtmosphereHeight)), 0, 0, 0,
                                 0, (float)(1.0 / (1.0 - mAtmosphereHeight)), 0, 0, 0, 0,
                                 (float)(1.0 / (1.0 - mAtmosphereHeight)), 0, 0, 0, 0, 1));

  VistaTransformMatrix matInvMV(matMV.GetInverted());

  GLfloat glMatP[16];
  glGetFloatv(GL_PROJECTION_MATRIX, &glMatP[0]);
  VistaTransformMatrix matInvP(VistaTransformMatrix(glMatP, true).GetInverted());
  VistaTransformMatrix matInvMVP(matInvMV * matInvP);

  // set uniforms ------------------------------------------------------------
  mAtmoShader->Bind();

  mAtmoShader->SetUniform(mAtmoShader->GetUniformLocation("uSunDir"), mSunDirection[0],
      mSunDirection[1], mSunDirection[2]);
  mAtmoShader->SetUniform(
      mAtmoShader->GetUniformLocation("uFarClip"), cs::utils::getCurrentFarClipDistance());

  mAtmoShader->SetUniform(mAtmoShader->GetUniformLocation("uWaterLevel"), mWaterLevel);
  mAtmoShader->SetUniform(
      mAtmoShader->GetUniformLocation("uAmbientBrightness"), mAmbientBrightness);

  data.mDepthBuffer->Bind(GL_TEXTURE0);
  data.mColorBuffer->Bind(GL_TEXTURE1);
  mTransmittanceTexture->Bind(GL_TEXTURE2);

  mAtmoShader->SetUniform(mAtmoShader->GetUniformLocation("uDepthBuffer"), 0);
  mAtmoShader->SetUniform(mAtmoShader->GetUniformLocation("uColorBuffer"), 1);
  mAtmoShader->SetUniform(mAtmoShader->GetUniformLocation("uTransmittanceTexture"), 2);

  if (mCloudTexture) {
    mCloudTexture->Bind(GL_TEXTURE3);
    mAtmoShader->SetUniform(mAtmoShader->GetUniformLocation("uCloudTexture"), 3);
    mAtmoShader->SetUniform(mAtmoShader->GetUniformLocation("uCloudAltitude"), mCloudAltitude);
  }

  if (mShadowMap) {
    int texUnitShadow = 4;
    mAtmoShader->SetUniform(
        mAtmoShader->GetUniformLocation("uShadowCascades"), (int)mShadowMap->getMaps().size());
    for (int i = 0; i < mShadowMap->getMaps().size(); ++i) {
      GLint locSamplers = glGetUniformLocation(
          mAtmoShader->GetProgram(), ("uShadowMaps[" + std::to_string(i) + "]").c_str());
      GLint locMatrices = glGetUniformLocation(mAtmoShader->GetProgram(),
          ("uShadowProjectionViewMatrices[" + std::to_string(i) + "]").c_str());

      mShadowMap->getMaps()[i]->Bind((GLenum)GL_TEXTURE0 + texUnitShadow + i);
      glUniform1i(locSamplers, texUnitShadow + i);

      auto mat = mShadowMap->getShadowMatrices()[i];
      glUniformMatrix4fv(locMatrices, 1, GL_FALSE, mat.GetData());
    }
  }

  // Why is there no set uniform for matrices???
  GLint loc = mAtmoShader->GetUniformLocation("uMatInvMV");
  glUniformMatrix4fv(loc, 1, GL_FALSE, matInvMV.GetData());
  loc = mAtmoShader->GetUniformLocation("uMatInvMVP");
  glUniformMatrix4fv(loc, 1, GL_FALSE, matInvMVP.GetData());
  loc = mAtmoShader->GetUniformLocation("uMatInvP");
  glUniformMatrix4fv(loc, 1, GL_FALSE, matInvP.GetData());
  loc = mAtmoShader->GetUniformLocation("uMatMV");
  glUniformMatrix4fv(loc, 1, GL_FALSE, matMV.GetData());

  // draw --------------------------------------------------------------------
  mQuadVAO->Bind();
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  mQuadVAO->Release();

  // clean up ----------------------------------------------------------------
  data.mDepthBuffer->Unbind(GL_TEXTURE0);
  data.mColorBuffer->Unbind(GL_TEXTURE1);
  mTransmittanceTexture->Unbind(GL_TEXTURE2);

  if (mCloudTexture) {
    mCloudTexture->Unbind(GL_TEXTURE3);
  }

  mAtmoShader->Release();

  glDepthMask(GL_TRUE);

  glPopAttrib();

  // update brightness value -------------------------------------------------
  // This is a crude approximation of the overall scene brightness due to
  // atmospheric scattering, camera position and the sun's position.
  // It may be used for fake HDR effects such as dimming stars.

  // some required positions and directions
  VistaVector3D vCamera         = (matInvMVP * VistaVector3D(0, 0, 0)).GetHomogenized();
  VistaVector3D vPlanet         = VistaVector3D(0, 0, 0);
  VistaVector3D vCameraToPlanet = (vCamera - vPlanet).GetNormalized();

  // [planet surface ... 5x atmosphere boundary] -> [0 ... 1]
  float fHeightInAtmosphere = (float)std::min(1.0,
      std::max(0.0, (vCamera.GetLength() - (1.f - mAtmosphereHeight)) / (mAtmosphereHeight * 5)));

  // [noon ... midnight] -> [1 ... -1]
  float fDaySide = vCameraToPlanet * mSunDirection;

  // limit brightness when on night side (also in dusk an dawn time)
  float fBrightnessOnSurface = std::pow(std::min(1.f, std::max(0.f, fDaySide + 1.f)), 50.f);

  // reduce brightness in outer space
  mApproximateBrightness = (1.f - fHeightInAtmosphere) * fBrightnessOnSurface;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool VistaAtmosphere::Do() {
  draw(VistaTransformMatrix());
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool VistaAtmosphere::GetBoundingBox(VistaBoundingBox& oBoundingBox) {

  // Boundingbox is computed by translation an edge points
  float fMin[3] = {(float)(-1.0 / (1.0 - mAtmosphereHeight)),
      (float)(-1.0 / (1.0 - mAtmosphereHeight)), (float)(-1.0 / (1.0 - mAtmosphereHeight))};
  float fMax[3] = {(float)(1.0 / (1.0 - mAtmosphereHeight)),
      (float)(1.0 / (1.0 - mAtmosphereHeight)), (float)(1.0 / (1.0 - mAtmosphereHeight))};

  oBoundingBox.SetBounds(fMin, fMax);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::initData() {
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

  mQuadVBO->Bind(GL_ARRAY_BUFFER);
  mQuadVBO->BufferData(data.size() * sizeof(float), &(data[0]), GL_STATIC_DRAW);
  mQuadVBO->Release();

  // positions
  mQuadVAO->EnableAttributeArray(0);
  mQuadVAO->SpecifyAttributeArrayFloat(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0, mQuadVBO);

  // create textures ---------------------------------------------------------
  for (auto const& viewport : GetVistaSystem()->GetDisplayManager()->GetViewports()) {
    GBufferData bufferData;

    bufferData.mDepthBuffer = new VistaTexture(GL_TEXTURE_RECTANGLE);
    bufferData.mDepthBuffer->Bind();
    bufferData.mDepthBuffer->SetWrapS(GL_CLAMP);
    bufferData.mDepthBuffer->SetWrapT(GL_CLAMP);
    bufferData.mDepthBuffer->SetMinFilter(GL_NEAREST);
    bufferData.mDepthBuffer->SetMagFilter(GL_NEAREST);
    bufferData.mDepthBuffer->Unbind();

    bufferData.mColorBuffer = new VistaTexture(GL_TEXTURE_RECTANGLE);
    bufferData.mColorBuffer->Bind();
    bufferData.mColorBuffer->SetWrapS(GL_CLAMP);
    bufferData.mColorBuffer->SetWrapT(GL_CLAMP);
    bufferData.mColorBuffer->SetMinFilter(GL_NEAREST);
    bufferData.mColorBuffer->SetMagFilter(GL_NEAREST);
    bufferData.mColorBuffer->Unbind();

    mGBufferData[viewport.second] = bufferData;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void VistaAtmosphere::updateTransmittanceTexture() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace csp::atmospheres
