/* global IApi, CosmoScout */

/**
 * Atmosphere Api
 */
class AtmosphereApi extends IApi {
  /**
   * @inheritDoc
   */
  name = 'atmosphere';

  /**
   * @inheritDoc
   */
  init() {
    CosmoScout.gui.initSlider('set_atmosphere_quality', 1, 30, 1, [7]);
    CosmoScout.gui.initSlider('set_water_level', -2, 2, 0.01, [0]);
  }
}

(() => {
  CosmoScout.init(AtmosphereApi);
})();
