////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_ATMOSPHERE_HPP
#define CSP_ATMOSPHERE_HPP

#include "../../../src/cs-scene/CelestialObject.hpp"
#include "Plugin.hpp"

#include "VistaAtmosphere.hpp"

#include <VistaKernel/GraphicsManager/VistaOpenGLDraw.h>

#include <memory>

namespace cs::core {
class GraphicsEngine;
}

namespace csp::atmospheres {

/// This is the atmosphere of a single planet/moon. For more information see VistaAtmosphere.
class Atmosphere : public cs::scene::CelestialObject, public IVistaOpenGLDraw {
 public:
  Atmosphere(std::shared_ptr<cs::core::GraphicsEngine> const& pGraphicsEngine,
      std::shared_ptr<Plugin::Properties> const& pProperties, std::string const& sCenterName,
      std::string const& sFrameName, double tStartExistence, double tEndExistence);
  ~Atmosphere() override = default;

  void setSun(std::shared_ptr<const cs::scene::CelestialObject> const& sun);
  void setCloudTexture(std::shared_ptr<VistaTexture> const& texture, double height);

  // csp::atmospheres::VistaAtmosphere const& getAtmosphere() const;
  VistaAtmosphere& getAtmosphere();

  void update(double tTime, cs::scene::CelestialObserver const& oObs) override;

  bool Do() override;
  bool GetBoundingBox(VistaBoundingBox& bb) override;

 private:
  std::shared_ptr<const cs::scene::CelestialObject> mSun;
  std::shared_ptr<cs::core::GraphicsEngine>         mGraphicsEngine;
  std::shared_ptr<Plugin::Properties>               mProperties;
  VistaAtmosphere                                   mAtmosphere;
  std::shared_ptr<VistaTexture>                     mCloudTexture;
  double                                            mCloudHeight = 0.001;
  glm::dvec3                                        mRadii;
};

} // namespace csp::atmospheres

#endif // CSP_ATMOSPHERE_HPP
