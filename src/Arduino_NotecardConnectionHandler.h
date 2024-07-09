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

//TODO: Understand how `_keep_alive` is updated and used in the `ConnectionHandler` class

#ifndef ARDUINO_NOTECARD_CONNECTION_HANDLER_H_
#define ARDUINO_NOTECARD_CONNECTION_HANDLER_H_

/******************************************************************************
   INCLUDE
 ******************************************************************************/

#include <stdint.h>
#include <Notecard.h>

#include "Arduino_ConnectionHandler.h"

#if defined(USE_NOTECARD) /* Only compile if the Notecard is present */

/******************************************************************************
   CLASS DECLARATION
 ******************************************************************************/

class NotecardConnectionHandler final : public ConnectionHandler
{
  public:
    enum class TopicType : uint8_t {
      Invalid = 0,
      Command,
      Thing,
      Notehub = 255
    };

    typedef enum {
      NOTECARD_ERROR_NONE                 = 0,
      NOTECARD_ERROR_NO_DATA_AVAILABLE    = -1,
      NOTECARD_ERROR_GENERIC              = -2,
      HOST_ERROR_OUT_OF_MEMORY            = -3,
    } NotecardCommunicationError;

    static const uint32_t NOTEHUB_CONN_TIMEOUT_MS = 185000;

    NotecardConnectionHandler(
      const String & project_uid,
      bool en_hw_int = false,
      bool keep_alive = true,
      uint32_t i2c_address = NOTE_I2C_ADDR_DEFAULT,
      uint32_t i2c_max = NOTE_I2C_MAX_DEFAULT,
      TwoWire & wire = Wire,
      const String & notehub_url = "-"
    );

    NotecardConnectionHandler(
      const String & project_uid,
      HardwareSerial & serial,
      uint32_t speed = 9600,
      bool en_hw_int = false,
      bool keep_alive = true,
      const String & notehub_url = "-"
    );

    // Accessors for Unique Hardware Identifiers
    const String & getArduinoDeviceId(void) const {
      return _device_id;
    }
    const String & getNotecardUid(void) const {
      return _notecard_uid;
    }

    // Identify the target topic for R/W operations
    TopicType getTopicType(void) const {
      return _topic_type;
    }
    void setTopicType(TopicType topic) {
      _topic_type = topic;
    }

    // ConnectionHandler interface
    virtual unsigned long getTime() override;
    virtual int write(const uint8_t *buf, size_t size) override;
    virtual int read() override;
    virtual bool available() override;

  protected:

    virtual NetworkConnectionState update_handleInit         () override;
    virtual NetworkConnectionState update_handleConnecting   () override;
    virtual NetworkConnectionState update_handleConnected    () override;
    virtual NetworkConnectionState update_handleDisconnecting() override;
    virtual NetworkConnectionState update_handleDisconnected () override;

  private:

    // Private members
    HardwareSerial * _serial;
    TwoWire * _wire;
    uint8_t * _inbound_buffer;
    uint32_t _conn_start_ms;
    uint32_t _i2c_address;
    uint32_t _i2c_max;
    uint32_t _uart_speed;
    uint32_t _inbound_buffer_index;
    uint32_t _inbound_buffer_size;
    bool _en_hw_int;
    TopicType _topic_type;
    Notecard _notecard;
    String _device_id;
    String _notecard_uid;
    String _notehub_url;
    String _project_uid;

    // Private methods
    bool armInterrupt (void) /* const */;
    bool configureConnection (bool connect) /* const */;
    uint_fast8_t connected (void) /* const */;
    J * getNote (bool pop = false) /* const */;
    bool updateUidCache (void);
};

#endif /* #ifdef USE_NOTECARD */

#endif /* ARDUINO_NOTECARD_CONNECTION_HANDLER_H_ */
