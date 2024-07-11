/*
   This file is part of ArduinoIoTCloud.

   Copyright 2019 ARDUINO SA (http://www.arduino.cc/)

   This software is released under the GNU General Public License version 3,
   which covers the main part of arduino-cli.
   The terms of this license can be found at:
   https://www.gnu.org/licenses/gpl-3.0.en.html

   You can be released from the requirements of the above licenses by purchasing
   a commercial license. Buying such a license is mandatory if you want to modify or
   otherwise use the software for commercial activities involving the Arduino
   software without disclosing the source code of your own applications. To purchase
   a commercial license, send an email to license@arduino.cc.
*/

/******************************************************************************
   INCLUDE
 ******************************************************************************/

#include "Arduino_NotecardConnectionHandler.h"

#if defined(USE_NOTECARD) /* Only compile if the Notecard is present */

#include <Arduino.h>
#include <Arduino_DebugUtils.h>
#include <Wire.h>

/******************************************************************************
   DEFINES
 ******************************************************************************/

#define NOTEFILE_BASE_NAME "arduino_iot_cloud"
#define NOTEFILE_DATABASE_LORA_PORT 80
#define NOTEFILE_INBOUND_LORA_PORT 79
#define NOTEFILE_OUTBOUND_LORA_PORT 83
#define NOTEFILE_SSL_DATABASE NOTEFILE_BASE_NAME ".dbs"
#define NOTEFILE_SSL_INBOUND NOTEFILE_BASE_NAME ".qis"
#define NOTEFILE_SSL_OUTBOUND NOTEFILE_BASE_NAME ".qos"

/******************************************************************************
   STLINK DEBUG OUTPUT
 ******************************************************************************/

// Provide Notehub debug output via STLINK serial port when available
#if defined(LOG_MEMORY_USAGE)
  #include <malloc.h>

  void logMemoryUsage (const char *ctx_, bool enter_ = false) {
    struct mallinfo mi = mallinfo();
    Debug.print(DBG_DEBUG, F("[MEMORY][%s %s] Allocated: %d bytes"), (enter_ ? ">>>>" : "<<<<"), ctx_, mi.uordblks);
  }
#endif
#if defined(ARDUINO_SWAN_R5) || defined(ARDUINO_CYGNET)
  #define STLINK_DEBUG
  HardwareSerial stlinkSerial(PIN_VCP_RX, PIN_VCP_TX);
#endif

/******************************************************************************
   TYPEDEF
 ******************************************************************************/

struct NotecardConnectionStatus
{
  NotecardConnectionStatus(void) : transport_connected(0), connected_to_notehub(0), notecard_error(0), host_error(0), reserved(0) { }
  NotecardConnectionStatus(uint_fast8_t x) : transport_connected(x & 0x01), connected_to_notehub(x & 0x02), notecard_error(x & 0x04), host_error(x & 0x08), reserved(x & 0xF0) { }
  NotecardConnectionStatus & operator=(uint_fast8_t x) {
      transport_connected  = (x & 0x01);
      connected_to_notehub = (x & 0x02);
      notecard_error       = (x & 0x04);
      host_error           = (x & 0x08);
      reserved             = (x & 0xF0);
      return *this;
  }
  operator uint_fast8_t () const {
      return ((reserved << 4) | (host_error << 3) | (notecard_error << 2) | (connected_to_notehub << 1) | (transport_connected));
  }

  bool transport_connected  : 1;
  bool connected_to_notehub : 1;
  bool notecard_error       : 1;
  bool host_error           : 1;
  uint_fast8_t reserved     : 4;
};
static_assert(sizeof(NotecardConnectionStatus) == sizeof(uint_fast8_t));

/******************************************************************************
   CTOR/DTOR
 ******************************************************************************/

NotecardConnectionHandler::NotecardConnectionHandler(
  const String & project_uid,
  bool en_hw_int,
  bool keep_alive,
  uint32_t i2c_address,
  uint32_t i2c_max,
  TwoWire & wire,
  const String & notehub_url
) :
  ConnectionHandler{keep_alive, NetworkAdapter::NOTECARD},
  _serial(nullptr),
  _wire(&wire),
  _inbound_buffer(nullptr),
  _conn_start_ms(0),
  _i2c_address(i2c_address),
  _i2c_max(i2c_max),
  _uart_speed(0),
  _inbound_buffer_index(0),
  _inbound_buffer_size(0),
  _en_hw_int(en_hw_int),
  _topic_type{TopicType::Invalid},
  _notecard{},
  _device_id{},
  _notecard_uid{},
  _notehub_url(notehub_url),
  _project_uid(project_uid)
{ }

NotecardConnectionHandler::NotecardConnectionHandler(
  const String & project_uid,
  HardwareSerial & serial,
  uint32_t speed,
  bool en_hw_int,
  bool keep_alive,
  const String & notehub_url
) :
  ConnectionHandler{keep_alive, NetworkAdapter::NOTECARD},
  _serial(&serial),
  _wire(nullptr),
  _inbound_buffer(nullptr),
  _conn_start_ms(0),
  _i2c_address(0),
  _i2c_max(0),
  _uart_speed(speed),
  _inbound_buffer_index(0),
  _inbound_buffer_size(0),
  _en_hw_int(en_hw_int),
  _topic_type{TopicType::Invalid},
  _notecard{},
  _device_id{},
  _notecard_uid{},
  _notehub_url(notehub_url),
  _project_uid(project_uid)
{ }

/******************************************************************************
   PUBLIC MEMBER FUNCTIONS
 ******************************************************************************/

const String & NotecardConnectionHandler::syncArduinoDeviceId (const String & device_id_)
{
  // Validate the connection state is not in an initialization state
  if (check() == NetworkConnectionState::INIT)
  {
    Debug.print(DBG_ERROR, F("Failed to sync Arduino Device ID. Connection has not been initialized."));
    return device_id_;
  }

  // Prefer the incoming Device ID. If the value is known to be invalid, then
  // fetch the Device ID from the Notecard Connection Handler (cloud cache)
  if ((device_id_ == "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx" || device_id_.length() == 0)
  && (updateUidCache() && _device_id.length() > 0)) {
    Debug.print(DBG_VERBOSE, F("Resolved Arduino Device ID with cached ID."));
  } else {
    _device_id = device_id_;
    Debug.print(DBG_VERBOSE, F("Resolved Arduino Device ID with provided ID."));

    // Report the Device ID to the Cloud
    Debug.print(DBG_VERBOSE, F("Updating cache..."));
    if (J *req = _notecard.newRequest("hub.set"))
    {
      if (_device_id.length() > 0) {
        JAddStringToObject(req, "sn", _device_id.c_str());
      } else {
        JAddStringToObject(req, "sn", "-");
      }
      if (J *rsp = _notecard.requestAndResponse(req)) {
        // Check the response for errors
        if (NoteResponseError(rsp)) {
          const char *err = JGetString(rsp, "err");
          Debug.print(DBG_ERROR, F("%s"), err);
        } else {
          Debug.print(DBG_VERBOSE, F("Cache updated successfully."));
        }
        JDelete(rsp);
      } else {
        Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
      }
    }

    // Perform manual hub sync to ensure the Device ID is updated immediately
    Debug.print(DBG_VERBOSE, F("Updating cloud..."));
    if (J *rsp = _notecard.requestAndResponse(_notecard.newRequest("hub.sync"))) {
      // Check the response for errors
      if (NoteResponseError(rsp)) {
        const char *err = JGetString(rsp, "err");
        Debug.print(DBG_ERROR, F("%s"), err);
      } else {
        Debug.print(DBG_VERBOSE, F("Cloud updated successfully."));
      }
      JDelete(rsp);
    } else {
      Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
    }
  }

  Debug.print(DBG_DEBUG, F("Synchronized Arduino Device ID: %s"), _device_id.c_str());

  return _device_id;
}

int NotecardConnectionHandler::syncSecretDeviceKey (const String & secret_device_key_)
{
  int result;

  // Validate the connection state is not in an initialization state
  if (check() == NetworkConnectionState::INIT)
  {
    Debug.print(DBG_ERROR, F("Failed to sync Secret Device Key. Connection has not been initialized."));
    result = NotecardCommunicationError::NOTECARD_ERROR_GENERIC;
  } else if (J *req = _notecard.newRequest("var.set")) {
    JAddStringToObject(req, "file", NOTEFILE_SSL_DATABASE);
    JAddStringToObject(req, "name", "arduino_iot_cloud_secret_key");
    if (secret_device_key_.length() > 0) {
      JAddStringToObject(req, "text", secret_device_key_.c_str());
    }
    JAddBoolToObject(req, "sync", true);
    if (J *rsp = _notecard.requestAndResponse(req)) {
      // Check the response for errors
      if (NoteResponseError(rsp)) {
        const char *err = JGetString(rsp, "err");
        Debug.print(DBG_ERROR, F("%s"), err);
        result = NotecardCommunicationError::NOTECARD_ERROR_GENERIC;
      } else {
        Debug.print(DBG_VERBOSE, F("Secret key updated successfully."));
        result = NotecardCommunicationError::NOTECARD_ERROR_NONE;
      }
      JDelete(rsp);
    } else {
      Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
      result = NotecardCommunicationError::NOTECARD_ERROR_GENERIC;
    }
  } else {
    Debug.print(DBG_ERROR, F("Failed to allocate request: var.set"));
    result = NotecardCommunicationError::HOST_ERROR_OUT_OF_MEMORY;
  }

  return result;
}

/******************************************************************************
   PUBLIC INTERFACE MEMBER FUNCTIONS
 ******************************************************************************/

bool NotecardConnectionHandler::available()
{
  bool buffered_data = (_inbound_buffer_index < _inbound_buffer_size);
  bool flush_required = !buffered_data && _inbound_buffer_size;

  // When the buffer is empty, look for a Note in the
  // NOTEFILE_SSL_INBOUND file to reload the buffer.
  if (!buffered_data) {
    // Reset the buffer
    free(_inbound_buffer);
    _inbound_buffer = nullptr;
    _inbound_buffer_index = 0;
    _inbound_buffer_size = 0;

    // Do NOT attempt to buffer the next Note immediately after buffer
    // exhaustion (a.k.a. flush required). Returning `false` between Notes,
    // will break the read loop, force the CBOR buffer to be parsed, and the
    // property containers to be updated.
    if (!flush_required) {
      // Reload the buffer
      J *note = getNote(true);
      if (note) {
        if (J *body = JGetObject(note, "body")) {
          _topic_type = static_cast<TopicType>(JGetInt(body, "topic"));
          if (_topic_type == TopicType::Invalid) {
            Debug.print(DBG_WARNING, F("Note does not contain a topic"));
          } else {
            buffered_data = JGetBinaryFromObject(note, "payload", &_inbound_buffer, &_inbound_buffer_size);
            if (!buffered_data) {
              Debug.print(DBG_WARNING, F("Note does not contain payload data"));
            } else {
              Debug.print(DBG_DEBUG, F("New payload buffered with size: %d"), _inbound_buffer_size);
            }
          }
        } else {
          _topic_type = TopicType::Invalid;
        }
        JDelete(note);
      }
    }
  }

  return buffered_data;
}

unsigned long NotecardConnectionHandler::getTime(void)
{
  unsigned long result;

  if (J *rsp = _notecard.requestAndResponse(_notecard.newRequest("card.time"))) {
    if (NoteResponseError(rsp)) {
      const char *err = JGetString(rsp, "err");
      Debug.print(DBG_ERROR, F("%s\n"), err);
      result = 0;
    } else {
      result = JGetInt(rsp, "time");
    }
    JDelete(rsp);
  } else {
    result = 0;
  }

  return result;
}

int NotecardConnectionHandler::read()
{
  int result;

  if (_inbound_buffer_index < _inbound_buffer_size) {
    result = _inbound_buffer[_inbound_buffer_index++];
  } else {
    result = NotecardCommunicationError::NOTECARD_ERROR_NO_DATA_AVAILABLE;
  }

  return result;
}

int NotecardConnectionHandler::write(const uint8_t * buf, size_t size)
{
  int result;

  if (J * req = _notecard.newRequest("note.add")) {
    JAddStringToObject(req, "file", NOTEFILE_SSL_OUTBOUND);
    if (buf) {
      JAddBinaryToObject(req, "payload", buf, size);
    }
    // Queue the Note when `_keep_alive` is disabled
    if (_keep_alive) {
      JAddBoolToObject(req, "sync", true);
    }
    if (J *body = JAddObjectToObject(req, "body")) {
      JAddIntToObject(body, "topic", static_cast<int>(_topic_type));
      J * rsp = _notecard.requestAndResponse(req);
      if (NoteResponseError(rsp)) {
        const char *err = JGetString(rsp, "err");
        Debug.print(DBG_ERROR, F("%s\n"), err);
        result = NotecardCommunicationError::NOTECARD_ERROR_GENERIC;
      } else {
        result = NotecardCommunicationError::NOTECARD_ERROR_NONE;
        Debug.print(DBG_INFO, F("Message sent correctly!"));
      }
      JDelete(rsp);
    } else {
      JFree(req);
      result = NotecardCommunicationError::HOST_ERROR_OUT_OF_MEMORY;
    }
  } else {
    result = NotecardCommunicationError::HOST_ERROR_OUT_OF_MEMORY;
  }

  return result;
}

/******************************************************************************
   PROTECTED STATE MACHINE FUNCTIONS
 ******************************************************************************/

NetworkConnectionState NotecardConnectionHandler::update_handleInit()
{
  NetworkConnectionState result = NetworkConnectionState::INIT;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif
#if defined(STLINK_DEBUG)
  // Output Notecard logs to the STLINK serial port
  stlinkSerial.end();  // necessary to handle multiple initializations (e.g. reconnections)
  stlinkSerial.begin(115200);
  const size_t usb_timeout_ms = 3000;
  for (const size_t start_ms = millis(); !stlinkSerial && (millis() - start_ms) < usb_timeout_ms;);
  _notecard.setDebugOutputStream(stlinkSerial);
#endif

  // Initialize the Notecard based on the configuration
  if (_serial) {
    _notecard.begin(*_serial, _uart_speed);
  } else {
    _notecard.begin(_i2c_address, _i2c_max, *_wire);
  }

  // Configure the ATTN pin to be used as an interrupt to indicate when a Note
  // is available to read. `getNote()` will only arm the interrupt if no old
  // Notes are available. If `ATTN` remains unarmed, it signals the user
  // application that outstanding Notes are queued and need to be processed.
  if (J *note = getNote(false)) {
    JDelete(note);
  }

  // Set the project UID
  if (NetworkConnectionState::INIT == result) {
    if (configureConnection(true)) {
      result = NetworkConnectionState::INIT;
    } else {
      result = NetworkConnectionState::ERROR;
    }
  }

#if defined(BOARD_HAS_SECRET_KEY)
  // Set database template to support LoRa/Satellite Notecard
  if (NetworkConnectionState::INIT == result) {
    if (J *req = _notecard.newRequest("note.template")) {
      JAddStringToObject(req, "file", NOTEFILE_SSL_DATABASE);
      JAddStringToObject(req, "format", "compact");               // Support LoRa/Satellite Notecards
      JAddIntToObject(req, "port", NOTEFILE_DATABASE_LORA_PORT);  // Support LoRa/Satellite Notecards
      if (J *body = JAddObjectToObject(req, "body")) {
        JAddStringToObject(body, "text", TSTRINGV);
        JAddNumberToObject(body, "value", TFLOAT64);
        JAddBoolToObject(body, "flag", TBOOL);
        if (J *rsp = _notecard.requestAndResponse(req)) {
          // Check the response for errors
          if (NoteResponseError(rsp)) {
            const char *err = JGetString(rsp, "err");
            Debug.print(DBG_ERROR, F("%s"), err);
            result = NetworkConnectionState::ERROR;
          } else {
            result = NetworkConnectionState::INIT;
          }
          JDelete(rsp);
        } else {
          Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
          result = NetworkConnectionState::ERROR; // Assume the worst
        }
      } else {
        Debug.print(DBG_ERROR, "Failed to allocate request: note.template:body");
        JFree(req);
        result = NetworkConnectionState::ERROR; // Assume the worst
      }
    } else {
      Debug.print(DBG_ERROR, "Failed to allocate request: note.template");
      result = NetworkConnectionState::ERROR; // Assume the worst
    }
  }
#endif

  // Set inbound template to support LoRa/Satellite Notecard
  if (NetworkConnectionState::INIT == result) {
    if (J *req = _notecard.newRequest("note.template")) {
      JAddStringToObject(req, "file", NOTEFILE_SSL_INBOUND);
      JAddStringToObject(req, "format", "compact");              // Support LoRa/Satellite Notecards
      JAddIntToObject(req, "port", NOTEFILE_INBOUND_LORA_PORT);  // Support LoRa/Satellite Notecards
      if (J *body = JAddObjectToObject(req, "body")) {
        JAddIntToObject(body, "topic", TUINT8);
        if (J *rsp = _notecard.requestAndResponse(req)) {
          // Check the response for errors
          if (NoteResponseError(rsp)) {
            const char *err = JGetString(rsp, "err");
            Debug.print(DBG_ERROR, F("%s"), err);
            result = NetworkConnectionState::ERROR;
          } else {
            result = NetworkConnectionState::INIT;
          }
          JDelete(rsp);
        } else {
          Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
          result = NetworkConnectionState::ERROR; // Assume the worst
        }
      } else {
        Debug.print(DBG_ERROR, "Failed to allocate request: note.template:body");
        JFree(req);
        result = NetworkConnectionState::ERROR; // Assume the worst
      }
    } else {
      Debug.print(DBG_ERROR, "Failed to allocate request: note.template");
      result = NetworkConnectionState::ERROR; // Assume the worst
    }
  }

  // Set outbound template to remove payload size restrictions
  if (NetworkConnectionState::INIT == result) {
    if (J *req = _notecard.newRequest("note.template")) {
      JAddStringToObject(req, "file", NOTEFILE_SSL_OUTBOUND);
      JAddStringToObject(req, "format", "compact");               // Support LoRa/Satellite Notecards
      JAddIntToObject(req, "port", NOTEFILE_OUTBOUND_LORA_PORT);  // Support LoRa/Satellite Notecards
      if (J *body = JAddObjectToObject(req, "body")) {
        JAddIntToObject(body, "topic", TUINT8);
        if (J *rsp = _notecard.requestAndResponse(req)) {
          // Check the response for errors
          if (NoteResponseError(rsp)) {
            const char *err = JGetString(rsp, "err");
            Debug.print(DBG_ERROR, F("%s"), err);
            result = NetworkConnectionState::ERROR;
          } else {
            result = NetworkConnectionState::INIT;
          }
          JDelete(rsp);
        } else {
          Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
          result = NetworkConnectionState::ERROR; // Assume the worst
        }
      } else {
        Debug.print(DBG_ERROR, "Failed to allocate request: note.template:body");
        JFree(req);
        result = NetworkConnectionState::ERROR; // Assume the worst
      }
    } else {
      Debug.print(DBG_ERROR, "Failed to allocate request: note.template");
      result = NetworkConnectionState::ERROR; // Assume the worst
    }
  }

  // Get the device UID
  if (NetworkConnectionState::INIT == result) {
    if (!updateUidCache()) {
      result = NetworkConnectionState::ERROR;
    } else {
      Debug.print(DBG_INFO, F("Notecard has been initialized."));
      if (_keep_alive) {
        _conn_start_ms = ::millis();
        Debug.print(DBG_INFO, F("Starting network connection..."));
        result = NetworkConnectionState::CONNECTING;
      } else {
        Debug.print(DBG_INFO, F("Network is disconnected."));
        result = NetworkConnectionState::DISCONNECTED;
      }
    }
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

NetworkConnectionState NotecardConnectionHandler::update_handleConnecting()
{
  NetworkConnectionState result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  // Check the connection status
  const NotecardConnectionStatus conn_status = connected();

  // Update the connection state
  if (!conn_status.connected_to_notehub) {
    if ((::millis() - _conn_start_ms) > NOTEHUB_CONN_TIMEOUT_MS) {
      Debug.print(DBG_ERROR, F("Timeout exceeded, connection to the network failed."));
      Debug.print(DBG_INFO, F("Retrying in \"%d\" milliseconds"), CHECK_INTERVAL_TABLE[static_cast<unsigned int>(NetworkConnectionState::CONNECTING)]);
      result = NetworkConnectionState::INIT;
    } else {
      // Continue awaiting the connection to Notehub
      if (conn_status.transport_connected) {
        Debug.print(DBG_INFO, F("Establishing connection to Notehub..."));
      } else {
        Debug.print(DBG_INFO, F("Connecting to the network..."));
      }
      result = NetworkConnectionState::CONNECTING;
    }
  } else {
    Debug.print(DBG_INFO, F("Connected to Notehub!"));
    result = NetworkConnectionState::CONNECTED;
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

NetworkConnectionState NotecardConnectionHandler::update_handleConnected()
{
  NetworkConnectionState result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  const NotecardConnectionStatus conn_status = connected();
  if (!conn_status.connected_to_notehub) {
    if (!conn_status.transport_connected) {
      Debug.print(DBG_ERROR, F("Connection to the network lost."));
    } else {
      Debug.print(DBG_ERROR, F("Connection to Notehub lost."));
    }
    result = NetworkConnectionState::DISCONNECTED;
  } else {
    result = NetworkConnectionState::CONNECTED;
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

NetworkConnectionState NotecardConnectionHandler::update_handleDisconnecting()
{
  NetworkConnectionState result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  Debug.print(DBG_ERROR, F("Connection to the network lost."));
  result = NetworkConnectionState::DISCONNECTED;

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

NetworkConnectionState NotecardConnectionHandler::update_handleDisconnected()
{
  NetworkConnectionState result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  if (_keep_alive)
  {
    Debug.print(DBG_ERROR, F("Attempting reconnection..."));
    result = NetworkConnectionState::INIT;
  }
  else
  {
    if (configureConnection(false)) {
      result = NetworkConnectionState::CLOSED;
      Debug.print(DBG_INFO, F("Closing connection..."));
    } else {
      result = NetworkConnectionState::ERROR;
      Debug.print(DBG_INFO, F("Error closing connection..."));
    }
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

/******************************************************************************
   PRIVATE MEMBER FUNCTIONS
 ******************************************************************************/

bool NotecardConnectionHandler::armInterrupt (void) const {
  bool result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  if (J *req = _notecard.newRequest("card.attn")) {
    JAddStringToObject(req, "mode","rearm,files");
    if (J *files = JAddArrayToObject(req, "files")) {
      JAddItemToArray(files, JCreateString(NOTEFILE_SSL_INBOUND));
      if (J *rsp = _notecard.requestAndResponse(req)) {
        // Check the response for errors
        if (NoteResponseError(rsp)) {
          const char *err = JGetString(rsp, "err");
          // This error must be ignored. As of LTSv6, `rearm` is not idempotent.
          // For now, we are counting on the fact that it is highly unlikely any
          // severe errors would occur in isolation. Once the Notecard firmware
          // is updated to support idempotent `rearm` requests, this error will
          // be handled as a failure.
          Debug.print(DBG_VERBOSE, F("%s"), err);
          result = true;  // Ignore the error
          // Debug.print(DBG_ERROR, F("%s\n"), err);
          // result = false;
        } else {
          result = true;
        }
        JDelete(rsp);
      } else {
        Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
        result = false;
      }
    } else {
      Debug.print(DBG_ERROR, "Failed to allocate request: card.attn:files");
      JFree(req);
      result = false;
    }
  } else {
    Debug.print(DBG_ERROR, "Failed to allocate request: card.attn");
    result = false;
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

bool NotecardConnectionHandler::configureConnection (bool connect) const {
  bool result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  if (J *req = _notecard.newRequest("hub.set")) {
    JAddStringToObject(req, "host", _notehub_url.c_str());
    JAddStringToObject(req, "product", _project_uid.c_str());
    if (connect) {
      JAddStringToObject(req, "mode", "continuous");
      JAddIntToObject(req, "inbound", 15);  // Unnecessary fail safe value
      JAddBoolToObject(req, "sync", true);
    } else {
      JAddStringToObject(req, "mode", "periodic");
      JAddIntToObject(req, "inbound", 1440);  //TODO: Revisit this value
      JAddIntToObject(req, "outbound", -1);
      JAddStringToObject(req, "vinbound", "-");
      JAddStringToObject(req, "voutbound", "-");
    }
    if (J *rsp = _notecard.requestAndResponseWithRetry(req, 30)) {
      // Check the response for errors
      if (NoteResponseError(rsp)) {
        const char *err = JGetString(rsp, "err");
        Debug.print(DBG_ERROR, F("%s"), err);
        result = false;
      } else {
        result = true;
      }
      JDelete(rsp);
    } else {
      Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
      result = false; // Assume the worst
    }
  } else {
    Debug.print(DBG_ERROR, "Failed to allocate request: hub.set");
    result = false; // Assume the worst
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

uint_fast8_t NotecardConnectionHandler::connected (void) const {
  NotecardConnectionStatus result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  // Query the connection status from the Notecard
  if (J *rsp = _notecard.requestAndResponse(_notecard.newRequest("hub.status"))) {
    // Ensure the transaction doesn't return an error
    if (NoteResponseError(rsp)) {
      const char *err = JGetString(rsp, "err");
      Debug.print(DBG_ERROR, F("%s"),err);
      result.notecard_error = true;
    } else {
      // Parse the transport connection status
      result.transport_connected = (strstr(JGetString(rsp,"status"),"{connected}") != nullptr);

      // Parse the status of the connection to Notehub
      result.connected_to_notehub = JGetBool(rsp,"connected");

      // Set the Notecard error status
      result.notecard_error = false;
      result.host_error = false;
    }

    // Free the response
    JDelete(rsp);
  } else {
    Debug.print(DBG_ERROR, F("Failed to acquire Notecard connection status."));
    result.transport_connected = false;
    result.connected_to_notehub = false;
    result.notecard_error = false;
    result.host_error = true;
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

J * NotecardConnectionHandler::getNote (bool pop) const {
  J * result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  // Look for a Note in the NOTEFILE_SSL_INBOUND file
  if (J *req = _notecard.newRequest("note.get")) {
    JAddStringToObject(req, "file", NOTEFILE_SSL_INBOUND);
    if (pop) {
      JAddBoolToObject(req, "delete", true);
    }
    if (J *note = _notecard.requestAndResponse(req)) {
      // Ensure the transaction doesn't return an error
      if (NoteResponseError(note)) {
        const char *jErr = JGetString(note, "err");
        if (NoteErrorContains(jErr, "{note-noexist}")) {
          // The Notefile is empty, thus no Note is available.
          if (_en_hw_int) {
            armInterrupt();
          }
        } else {
          // Any other error indicates that we were unable to
          // retrieve a Note, therefore no Note is available.
        }
        result = nullptr;
        JDelete(note);
      } else {
        // The Note was successfully retrieved, and it now
        // becomes the callers responsibility to free it.
        result = note;
      }
    } else {
      Debug.print(DBG_ERROR, F("Failed to receive response from Notecard."));
      result = nullptr;
    }
  } else {
    Debug.print(DBG_ERROR, "Failed to allocate request: note.get");
    // Failed to retrieve a Note, therefore no Note is available.
    result = nullptr;
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

bool NotecardConnectionHandler::updateUidCache (void) {
  bool result;
#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__, true);
#endif

  // This operation is safe to perform before a sync has occurred, because the
  // Notecard UID is static and the cloud value of Serial Number is ultimately
  // given preference to the value reported by the device.

  // Read the Notecard UID from the Notehub configuration
  if (J *rsp = _notecard.requestAndResponse(_notecard.newRequest("hub.get"))) {
    // Check the response for errors
    if (NoteResponseError(rsp)) {
      const char *err = JGetString(rsp, "err");
      Debug.print(DBG_ERROR, F("Failed to read Notecard UID"));
      Debug.print(DBG_ERROR, F("Error: %s"), err);
      result = false;
    } else {
      _notecard_uid = JGetString(rsp, "device");
      _device_id = JGetString(rsp, "sn");
      Debug.print(DBG_DEBUG, F("Cached Notecard UID: <%s> and Arduino Device ID: <%s>"), _notecard_uid.c_str(), _device_id.c_str());
      result = true;
    }
    JDelete(rsp);
  } else {
    Debug.print(DBG_ERROR, F("Failed to read Notecard UID"));
    result = false;
  }

#if defined(LOG_MEMORY_USAGE)
  logMemoryUsage(__FUNCTION__);
#endif
  return result;
}

#endif
