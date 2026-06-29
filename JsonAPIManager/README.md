# JSON API Manager

A small CJson based accessor API for the peripheral manager. See the API scheme below.

### Version

beta - in development

## Brief

This is a JSON based HTTP server which accepts POST requests on port 8000 by default. This driver relies heavily on ESP32-IDF specific HTTP server code to operate and I'm not likely to port it any time soon...

## How To Use

The JSON API scheme follows. The minimum fields for any command are `cmd_type`, `periph_id` and `param_id`. Other fields may be required. Non-required fields will be ignored.

### Command Types

| Command       |   Value  |  Periph_id Value |   Param_id Value |      Response     |
|--------------:|---------:|-----------------:|-----------------:|------------------:|
| INFO          |   0      |         0        |           0      |     Device Info   |
|               |   0      |         N (>0)   |           0      |     Periph Info   |
|               |   0      |         N (>0)   |           M (>0) |     Param Info    |
| GET           |   1      |         N (>0)   |           M (>0) | GET Response(data)|
| SET           |   2      |         N (>0)   |           M (>0) |   ACK Response    |
| ACT           |   3      |         N (>0)   |           M (>0) |   ACK Response    |

---

The main request types are INFO, GET, SET, and ACT. 

- An INFO command will generate a response depending on the periph_id and param_id value
- If both values are zero, the API will return a device info packet - this includes a list of device parameter ids. 
- If the Peripheral id is set to one of these values and the param id is zero, the device will return a Peripheral Info packet. This includes a list of parameter ids. 
- If both the peripheral and parameter id are non-zero and contained in these delivered lists, the device will return a parameter info packet.
- In this way, the full device can be enumerated.

### Response Types

#### Error Response

An error response will be generated for a number of reasons including json errors, invalid ids, on-board timeouts etc. 

| Field     |  Type     |   Purpose         |
|----------:|----------:|------------------:|
| rsp_type  | Integer   | Packet ID         |
| err_code  | Integer   | Unique error id   |
| error     | String    | Error description*|   (*if strings are enabled in the API else empty)

---

#### Device Info Response

Returned when an INFO command is sent with both `periph_id` and `param_id` set to zero.

This packet contains basic information about the device along with a list of available peripheral ids. These ids may then be used to enumerate the device further.

| Field       | Type            | Purpose                           |
|------------:|----------------:|----------------------------------:|
| rsp_type    | Integer         | Response packet identifier        |
| name        | String          | Device name                       |
| dev_id      | Integer         | Device identifier                 |
| periph_num  | Integer         | Number of peripherals             |
| periph_ids  | Integer Array   | List of available peripheral ids  |

---

#### Peripheral Info Response

Returned when an INFO command is sent with a valid `periph_id` and `param_id` equal to zero.

This packet describes a peripheral and provides a list of parameter ids belonging to that peripheral.

| Field        | Type            | Purpose                          |
|-------------:|----------------:|---------------------------------:|
| rsp_type     | Integer         | Response packet identifier       |
| name         | String          | Peripheral name                  |
| periph_id    | Integer         | Peripheral identifier            |
| periph_type  | Integer         | Peripheral type                  |
| param_num    | Integer         | Number of parameters             |
| param_ids    | Integer Array   | List of available parameter ids  |

---

#### Parameter Info Response

Returned when an INFO command is sent with valid `periph_id` and `param_id` values.

This packet describes a single parameter including its supported operations and data type.

| Field       | Type      | Purpose                                 |
|------------:|----------:|----------------------------------------:|
| rsp_type    | Integer   | Response packet identifier              |
| periph_id   | Integer   | Peripheral identifier                   |
| param_id    | Integer   | Parameter identifier                    |
| param_name  | String    | Human readable parameter name           |
| param_max   | Integer   | Maximum supported value                 |
| methods     | Integer   | Bitfield describing supported methods   |
| data_type   | Integer   | Parameter data type                     |

---

#### Get Response

Returned following a successful GET request.

The `data` field type depends on the parameter's `data_type`.

| Field       | Type              | Purpose                    |
|------------:|------------------:|---------------------------:|
| rsp_type    | Integer           | Response packet identifier |
| periph_id   | Integer           | Peripheral identifier      |
| param_id    | Integer           | Parameter identifier       |
| data_type   | Integer           | Returned data type         |
| data        | Variable          | Parameter value            |

---

#### Ack Response

Returned following a successful SET or ACT request.

| Field       | Type      | Purpose                    |
|------------:|----------:|---------------------------:|
| rsp_type    | Integer   | Response packet identifier |
| periph_id   | Integer   | Peripheral identifier      |
| param_id    | Integer   | Parameter identifier       |

---

##### Fields

Several packet fields appear throughout the API:

| Field       | Description |
|------------:|------------:|
| cmd_type    | Command to execute (INFO, GET, SET or ACT) |
| rsp_type    | Response packet type identifier |
| periph_id   | Peripheral identifier |
| param_id    | Parameter identifier |
| data_type   | Data encoding for the associated parameter |
| data        | Parameter value. Type depends on `data_type`. |
| methods     | Bitfield describing which operations are supported by a parameter. |



## Data Types

The `data_type` field describes the encoding of a parameter value. Integer values use the MSB as a signed flag.

| Data Type | Value | Description |
|----------:|------:|------------:|
| DATATYPE_NONE | `0x00` | No data |
| DATATYPE_UINT8 | `0x01` | Unsigned 8-bit integer |
| DATATYPE_INT8 | `0x81` | Signed 8-bit integer |
| DATATYPE_UINT16 | `0x02` | Unsigned 16-bit integer |
| DATATYPE_INT16 | `0x82` | Signed 16-bit integer |
| DATATYPE_UINT32 | `0x03` | Unsigned 32-bit integer |
| DATATYPE_INT32 | `0x83` | Signed 32-bit integer |
| DATATYPE_FLOAT | `0x04` | IEEE-754 single precision float |
| DATATYPE_DOUBLE | `0x08` | IEEE-754 double precision float |
| DATATYPE_STRING | `0x0A` | UTF-8 string |
| DATATYPE_BOOL | `0x0B` | Boolean value |
| DATATYPE_INVALID | `0xFF` | Invalid data type |

---

## Peripheral Types

The `periph_type` field identifies the function of a peripheral.

| Peripheral Type | Value | Description |
|----------------:|------:|------------:|
| PTYPE_ADDR_LEDS | `0x01` | Addressable LED peripheral |
| PTYPE_STD_LED | `0x02` | Standard LED peripheral |
| PTYPE_ACCEL_SENSOR | `0x03` | Accelerometer / gyroscope / motion sensor |
| PTYPE_ENVIRO_SENSOR | `0x04` | Environmental sensor (temperature, humidity, pressure, etc.) |
| PTYPE_DISTANCE_SENSOR | `0x05` | Distance or proximity sensor |
| PTYPE_POWER_SENSOR | `0x06` | Voltage / current measurement peripheral |
| PTYPE_ADC | `0x07` | Analogue-to-digital converter |
| PTYPE_IO | `0x08` | General purpose I/O |
| PTYPE_DISPLAY | `0x09` | Display peripheral (OLED, LED, ePaper, etc.) |
| PTYPE_COMMS | `0x0A` | Communications peripheral (Bluetooth, radio, etc.) |
| PTYPE_NONE | `0xFF` | Undefined peripheral type |

---

## Parameter Flags

The `methods` field is a bitmask describing which operations are supported by a parameter.

| Flag | Value | Description |
|-----:|------:|------------:|
| GET_FLAG | `0x01` | Parameter supports GET requests |
| SET_FLAG | `0x02` | Parameter supports SET requests |
| ACT_FLAG | `0x04` | Parameter supports ACT requests |
| STREAM_FLAG | `0x08` | Parameter supports streaming |

Multiple flags may be OR'd together. For example, a value of `0x03` indicates that a parameter supports both GET and SET operations.

---

## Response Types

The `rsp_type` field identifies the type of response packet returned by the API.

| Response Type | Value | Description |
|-------------:|------:|------------:|
| RSP_TYPE_DEV_INFO | `0` | Device information response |
| RSP_TYPE_PERIPH_INFO | `1` | Peripheral information response |
| RSP_TYPE_PARAM_INFO | `2` | Parameter information response |
| RSP_TYPE_DATA | `3` | Successful GET response containing data |
| RSP_TYPE_ACK | `4` | Acknowledge response for successful SET or ACT commands |
| RSP_TYPE_ERR | `5` | Error response |
| RSP_TYPE_STREAM | `6` | Streamed data response |


## Credits