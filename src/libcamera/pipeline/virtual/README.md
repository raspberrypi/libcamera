# Virtual Pipeline Handler

Virtual pipeline handler emulates fake external camera(s) for testing.

## Parse config file and register cameras

- A sample config file is located at `src/libcamera/pipeline/virtual/data/virtual.yaml`.
- If libcamera is installed, the config file should be installed at
  `share/libcamera/pipeline/virtual/virtual.yaml`.

### Config File Format
The config file contains the information about cameras' properties to register.
The config file should be a yaml file with dictionary of the cameraIds
associated with their properties as top level. The default value will be applied
when any property is empty.

Each camera block is a dictionary, containing the following keys:
- `supported_formats` (list of `VirtualCameraData::Resolution`, optional):
  List of supported resolution and frame rates of the emulated camera
    - `width` (`unsigned int`, default=1920): Width of the window resolution.
      This needs to be even.
    - `height` (`unsigned int`, default=1080): Height of the window resolution.
    - `frame_rates` (list of `int`, default=`[30,60]` ): Range of the frame
      rate (per second). If the list contains one value, it's the lower bound
      and the upper bound. If the list contains two values, the first is the
      lower bound and the second is the upper bound. No other number of values
      is allowed.
- `test_pattern` (`string`): Which test pattern to use as frames. The options
  are "bars", "lines". Cannot be set with `frames`.
  - The test patterns are "bars" which means color bars, and "lines" which means
    diagonal lines.
- `frames` (dictionary):
  - `path` (`string`): Path to an image, or path to a directory of a series of
    images. Cannot be set with `test_pattern`.
    - The path to an image has ".jpg" extension.
    - The path to a directory ends with "/". The name of the images in the
      directory are "{n}.jpg" with {n} is the sequence of images starting with 0.
- `location` (`string`, default="front"): The location of the camera. Support
  "CameraLocationFront", "CameraLocationBack", and "CameraLocationExternal".
- `model` (`string`, default="Unknown"): The model name of the camera.

Check `data/virtual.yaml` as the sample config file.

### Implementation

`Parser` class provides methods to parse the config file to register cameras
in Virtual Pipeline Handler. `parseConfigFile()` is exposed to use in
Virtual Pipeline Handler.

This is the procedure of the Parser class:
1. `parseConfigFile()` parses the config file to `YamlObject` using `YamlParser::parse()`.
    - Parse the top level of config file which are the camera ids and look into
      each camera properties.
2. For each camera, `parseCameraConfigData()` returns a camera with the configuration.
    - The methods in the next step fill the data with the pointer to the Camera object.
    - If the config file contains invalid configuration, this method returns
      nullptr. The camera will be skipped.
3. Parse each property and register the data.
    - `parseSupportedFormats()`: Parses `supported_formats` in the config, which
      contains resolutions and frame rates.
    - `parseFrameGenerator()`: Parses `test_pattern` or `frames` in the config.
    - `parseLocation()`: Parses `location` in the config.
    - `parseModel()`: Parses `model` in the config.
4. Back to `parseConfigFile()` and append the camera configuration.
5. Returns a list of camera configurations.
