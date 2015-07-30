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
 
 enum ledStates {LEDSTATE_1_ON,
                 LEDSTATE_1_WAIT,
                 LEDSTATE_2_ON,
                 LEDSTATE_2_WAIT,
                 LEDSTATE_LOW_VOLTAGE_BLINK_ON,
                 LEDSTATE_LOW_VOLTAGE_BLINK_OFF
 };
 
 enum returnStatus {RETURN_STATUS_LOW_VOLTAGE = 0x01,
 };
 
