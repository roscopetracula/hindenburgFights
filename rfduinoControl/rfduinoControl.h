enum igniterStateEnum {IGNITER_STATE_LOCKED,      // Igniter forced off regardless of inputs. 
                       IGNITER_STATE_OFF,         // Igniter allowed to fire but not triggered.
                       IGNITER_STATE_ON,          // Igniter currently triggered and firing.
                       IGNITER_STATE_TURNING_OFF, // Igniter trigger has been released but igniter is not off yet.
                       IGNITER_STATE_COOLDOWN,    // Igniter has been on for maximum time and cannot fire until it is untriggered.
};


