////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CSP_ATMOSPHERE_HPP
#define CSP_ATMOSPHERE_HPP

#include "../../../src/cs-scene/CelestialObject.hpp"
#include "AtmosphereRenderer.hpp"
#include "Plugin.hpp"

#include <glm/glm.hpp>

namespace csp::atmospheres {

/// This is a wrapper around a AtmosphereRenderer, adding SPICE based positioning.
class Atmosphere : public cs::scene::CelestialObject {
 public:
  Atmosphere(std::shared_ptr<Plugin::Properties> const& pProperties,
      Plugin::Settings::Atmosphere const& settings, std::string const& sCenterName,
      std::string const& sFrameName, double tStartExistence, double tEndExistence);
  ~Atmosphere();

  /// Access the internal AtmosphereRender to configure atmosphere parameters.
  AtmosphereRenderer&       getRenderer();
  AtmosphereRenderer const& getRenderer() const;

  /// This is called once a frame by the solar system. It updates the atmosphere's position based on
  /// the SPICE kernels for the center and frame specified at construction time.
  void update(double time, cs::scene::CelestialObserver const& oObs) override;

 private:
  AtmosphereRenderer                  mRenderer;
  std::shared_ptr<Plugin::Properties> mProperties;
  VistaOpenGLNode*                    mAtmosphereNode;
};

} // namespace csp::atmospheres

#endif // CSP_ATMOSPHERE_HPP
