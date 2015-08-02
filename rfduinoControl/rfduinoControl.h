enum igniterStateEnum {IGNITER_STATE_LOCKED,      // Igniter forced off regardless of inputs. 
                       IGNITER_STATE_OFF,         // Igniter allowed to fire but not triggered.
                       IGNITER_STATE_ON,          // Igniter currently triggered and firing.
                       IGNITER_STATE_TURNING_OFF, // Igniter trigger has been released but igniter is not off yet.
                       IGNITER_STATE_COOLDOWN,    // Igniter has been on for maximum time and cannot fire until it is untriggered.
};

enum returnMsgEnum {RETURN_MSG_STRING = 0x00,
                    RETURN_MSG_UPDATE = 0x01,
                    RETURN_MSG_FAULT = 0x02,
};
 
 enum ledStates {LEDSTATE_START,
                 LEDSTATE_BEACON_ON,
                 LEDSTATE_BEACON_OFF,
                 LEDSTATE_LOW_VOLTAGE,
                 LEDSTATE_LOW_VOLTAGE_BLINK_ON,
                 LEDSTATE_LOW_VOLTAGE_BLINK_OFF,
                 LEDSTATE_CONNECTING,
                 LEDSTATE_CONNECTING_RED_ON,
                 LEDSTATE_CONNECTING_RED_OFF,
                 LEDSTATE_CONNECTING_GREEN_ON,
                 LEDSTATE_CONNECTING_GREEN_OFF
 };
 
 enum returnStatus {RETURN_STATUS_LOW_VOLTAGE = 0x01,
 };
 
