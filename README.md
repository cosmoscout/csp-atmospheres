# Atmospheres for CosmoScout VR

A CosmoScout VR plugin for drawing atmospheres around celestial bodies. It calculates single Mie- and Rayleigh scattering via raycasting in real-time. This plugin is built as part of CosmoScout's build process. See the [main repository](https://github.com/cosmoscout/cosmoscout-vr) for instructions.

## Configuration

This plugin can be enabled with the following configuration in your `settings.json`:

```javascript
{
  ...
  "plugins": {
    ...
    "csp-atmospheres": {
      "atmospheres": {
        <anchor name>: {
          "atmosphereHeight": <float>,             // Relative to the planets radius.
          "mieAnisotropy": <float>,
          "mieHeight": <float>,
          "mieScatteringB": <float>,
          "mieScatteringG": <float>,
          "mieScatteringR": <float>,
          "rayleighAnisotropy": <float>,
          "rayleighHeight": <float>,
          "rayleighScatteringB": <float>,
          "rayleighScatteringG": <float>,
          "rayleighScatteringR": <float>,
          "sunIntensity": <float>
          "cloudHeight": <float>,                 // Optional Relative to the planets radius.
          "cloudTexture": <path to cloud texture> // optional
        },
        ... <more atmospheres> ...
      }
    }
  }
}
```

## MIT License

Copyright (c) 2019 German Aerospace Center (DLR)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
