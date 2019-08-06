////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_ATMOSPHERE_VISTA_ATMOSPHERE_HPP
#define CSP_ATMOSPHERE_VISTA_ATMOSPHERE_HPP

#include <VistaBase/VistaVectorMath.h>
#include <VistaKernel/GraphicsManager/VistaOpenGLDraw.h>

#include <unordered_map>

class VistaViewport;
class VistaGLSLShader;
class VistaVertexArrayObject;
class VistaBufferObject;
class VistaTexture;
class VistaOpenGLNode;

namespace cs::graphics {
class ShadowMap;
}

namespace csp::atmospheres {

/// This class draws a configurable atmosphere. Just put an OpenGLNode into your SceneGraph at the
/// very same position as your planet. Set its scale to the same size as your planet.
class VistaAtmosphere : public IVistaOpenGLDraw {
 public:
  /// When using these presets, all the configuration options are set accordingly.
  enum class Preset { eEarth, eMars, eBlueParadise, eSmog };

  explicit VistaAtmosphere(Preset ePreset = Preset::eEarth);
  explicit VistaAtmosphere(const std::string& sConfigFile);
  ~VistaAtmosphere() override;

  /// Loads a given preset. All values below will be adjusted.
  void loadPreset(Preset ePreset);
  void loadConfigFile(const std::string& sConfigFile);

  /// When set, the shader will make lookups in order to generate light shafts.
  void setShadowMap(cs::graphics::ShadowMap const* pShadowMap);

  /// When set, the shader will draw this texture at the given altitude.
  void setCloudTexture(VistaTexture* pTexture, float fAltitude);

  /// The model space incoming light direction. Default is (1, 0, 0).
  VistaVector3D getSunDirection() const;
  void          setSunDirection(const VistaVector3D& vValue);

  /// Returns a value [0..1] which approximates the overall brightness of the atmosphere. Will be
  /// close to zero in outer space or in the planets shadow, close to one on the bright surface of
  /// the planet. It uses the camera data from the last rendering call. Use this value for fake HDR
  /// effects, such as reducing star brightness.
  float getApproximateSceneBrightness() const;

  /// How many samples are taken along the view ray. Trades quality for performance. Default is 15.
  int  getPrimaryRaySteps() const;
  void setPrimaryRaySteps(int iValue);

  /// How many samples are taken along the sun rays. Trades quality for performance. Default is 3.
  int  getSecondaryRaySteps() const;
  void setSecondaryRaySteps(int iValue);

  /// The brightness of the sun. Default is 15.
  float getSunIntensity() const;
  void  setSunIntensity(float fValue);

  /// The maximum height of the atmosphere above the planets surface relative to the planets radius.
  /// Default depends on the preset; for Earth 60.0 / 6360.0 is assumed.
  double getAtmosphereHeight() const;
  void   setAtmosphereHeight(double dValue);

  /// The scale height for Mie scattering above the planets surface relative to the planets radius.
  /// Default depends on the preset; for Earth 1.2 / 6360.0 is assumed.
  double getMieHeight() const;
  void   setMieHeight(double dValue);

  /// The Mie scattering values. Default depends on the preset; for Earth (21.0, 21.0, 21.0) is
  /// assumed.
  VistaVector3D getMieScattering() const;
  void          setMieScattering(const VistaVector3D& vValue);

  /// The Mie scattering anisotropy. Default depends on the preset; for Earth 0.76 is assumed.
  double getMieAnisotropy() const;
  void   setMieAnisotropy(double dValue);

  /// The scale height for Rayleigh scattering above the planets surface relative to the planets
  /// radius. Default depends on the preset; for Earth 8.0 / 6360.0 is assumed.
  double getRayleighHeight() const;
  void   setRayleighHeight(double dValue);

  /// The Rayleigh scattering values. Default depends on the preset; for Earth (5.8, 13.5, 21.1) is
  /// assumed.
  VistaVector3D getRayleighScattering() const;
  void          setRayleighScattering(const VistaVector3D& vValue);

  /// The Rayleigh scattering anisotropy. Default depends on the preset; for Earth 0.0 is assumed.
  double getRayleighAnisotropy() const;
  void   setRayleighAnisotropy(double dValue);

  /// If true, an artificial disc is drawn in the suns direction.
  bool getDrawSun() const;
  void setDrawSun(bool bEnable);

  /// If true, a ocean is drawn at water level.
  bool getDrawWater() const;
  void setDrawWater(bool bEnable);

  /// The height of the ocean level. In atmosphere space, that means 0 equals sea level, larger
  /// values increase the sea level. If set to AtmosphereHeight, the ocean will be at atmosphere the
  /// boundary.
  float getWaterLevel() const;
  void  setWaterLevel(float fValue);

  /// A value of 0 will multiply the planet surface with the extinction factor of the sun color,
  /// making the night side completely dark. A value of 1 will result in no extinction of the
  /// incoming light.
  float getAmbientBrightness() const;
  void  setAmbientBrightness(float fValue);

  /// If true, tonemapping is performed on the atmospheric color.
  bool getUseToneMapping() const;
  void setUseToneMapping(bool bEnable, float fExposure, float fGamma);

  /// If true, the depth buffer is assumed to contain linear depth values. This significantly
  /// reduces artifacts for large scenes.
  bool getUseLinearDepthBuffer() const;
  void setUseLinearDepthBuffer(bool bEnable);

  /// Optional draw method if the atmosphere is not attached to an OpenGLNode the provided
  /// matModelMatrix will be multiplied with GL_MODELVIEW_MATRIX.
  void draw(VistaTransformMatrix const& matModelMatrix);

  bool Do() override;
  bool GetBoundingBox(VistaBoundingBox& bb) override;

 private:
  void initData();
  void updateTransmittanceTexture();
  void updateShader();

  cs::graphics::ShadowMap const* mShadowMap = nullptr;

  VistaTexture* mCloudTexture  = nullptr;
  float         mCloudAltitude = 0.f;

  VistaTexture*    mTransmittanceTexture = nullptr;
  VistaGLSLShader* mAtmoShader           = nullptr;

  struct GBufferData {
    VistaTexture* mDepthBuffer = nullptr;
    VistaTexture* mColorBuffer = nullptr;
  };

  std::unordered_map<VistaViewport*, GBufferData> mGBufferData;

  VistaVertexArrayObject* mQuadVAO = nullptr;
  VistaBufferObject*      mQuadVBO = nullptr;

  bool          mTransmittanceTextureDirty = true;
  bool          mShaderDirty               = true;
  bool          mDrawSun                   = true;
  bool          mDrawWater                 = false;
  float         mWaterLevel                = 0.0;
  float         mAmbientBrightness         = 0.2;
  double        mAtmosphereHeight          = 1.0;
  int           mPrimaryRaySteps           = 15;
  int           mSecondaryRaySteps         = 4;
  float         mSunIntensity              = 1.f;
  VistaVector3D mSunDirection              = VistaVector3D(1, 0, 0);

  double        mMieHeight     = 0.0;
  VistaVector3D mMieScattering = VistaVector3D(1, 1, 1);
  double        mMieAnisotropy = 0.0;

  double        mRayleighHeight     = 0.0;
  VistaVector3D mRayleighScattering = VistaVector3D(1, 1, 1);
  double        mRayleighAnisotropy = 0.0;

  float mApproximateBrightness = 0.0;

  bool  mUseLinearDepthBuffer = false;
  bool  mUseToneMapping       = true;
  float mExposure             = 0.6;
  float mGamma                = 2.2;

  static const std::string cAtmosphereVert;
  static const std::string cAtmosphereFrag0;
  static const std::string cAtmosphereFrag1;
};
} // namespace csp::atmospheres
#endif // CSP_ATMOSPHERE_VISTA_ATMOSPHERE_HPP