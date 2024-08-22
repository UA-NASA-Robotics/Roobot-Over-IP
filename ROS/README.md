# ROS Integration

## lib

ROI interface code

## ROS Interfaces

### Topics

#### Valid Configuration

`/roi/valid_configuration`

- Type: `std_msgs/String`
- Denotes if the current configuration is valid. If the configuration is invalid, the reason is included in the message. ie `true` or `false: <reason>`

### Services

### Actions

### Parameters

The ROI ROS interface is dynamically configured by accepting multiple parameters. Available parameters are:

- `config_filepath`:
  This is the main parameter defining what modules are connected to ROS, and how they are accessed. It is technically a string, but should be a complete root filepath. The file should be a JSON file with the following structure:
  ```json
  [
    {
      "namespace": "front",
      "modules": [
        {
          "type": "headLight",
          "octet": "16",
          "parameters": {
            "lightSubDeviceID": "5"
          }
        },
        {
          "type": "generalGPIO",
          "octet": "15",
          "parameters": {
            "subDeviceModes": [
              "3",
              "3",
              "3",
              "3",
              "3",
              "3",
              "3",
              "3",
              "1",
              "1",
              "1",
              "1",
              "1",
              "1",
              "1",
              "1"
            ],
            "subDeviceValues": [
              "1",
              "0",
              "1",
              "1",
              "1",
              "1",
              "1",
              "1",
              "0",
              "0",
              "0",
              "0",
              "0",
              "0",
              "0",
              "0"
            ]
          }
        }
      ]
    }
  ]
  ```
  Each module will generate services/topics/actions of the correct type, and they are accessed based on the `namespace` and `type` fields. ie `/roi/*namespace*/*type*` The `octet` field is used to determine the address of the module on the ROI bus. The `parameters` field is used to pass any additional parameters to the module. The `config_filepath` parameter is required.
