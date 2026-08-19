# Nasal/state-init.nas

# Copyright (C) 2026 Philips Nguyen
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

var state_init = {
    active: 0
};

state_init.requires_running = func(state_name) {
    return state_name == "take-off"
        # or state_name == "cruise"
        or state_name == "approach";
};

state_init.finalize_running_state = func(state_name) {
    # Always make sure the starter is released.
    setprop("/controls/engines/engine[0]/starter", 0);

    # Keep this minimal for now.
    if (state_name == "take-off") {
        # After engine runs, reduce throttle to idle and release brakes for rolling take-off
        setprop("/controls/engines/engine[0]/throttle", 0);
        setprop("/controls/gears/brake-parking", 0);

        # Engine is up now, so electrical generation should be available.
        setprop("/fdm/jsbsim/systems/powerplant-controls/electrical/switches/alternator", 1);

        # Ensure take-off flaps are set as part of state finalization.
        # Selector 1 is the equivalent to 10 degree detent.
        setprop("/fdm/jsbsim/systems/airframe-controls/flaps/selector", 1);
    } elsif (state_name == "approach") {
        setprop("/controls/engines/engine[0]/throttle", 0.425);
        setprop("/controls/engines/engine[0]/mixture", 1);
        setprop("/controls/engines/engine[0]/propeller-pitch", 1);
    }
};

state_init.poll_until_stable_running = func(state_name, remaining) {
    if (getprop("/engines/engine[0]/running", 0) == 1 and getprop("/engines/engine[0]/rpm", 0) >= 700) {
        state_init.finalize_running_state(state_name);
        state_init.active = 0;
        return;
    }

    if (remaining <= 0) {
        setprop("/controls/engines/engine[0]/starter", 0);

        if (state_name == "take-off") {
            setprop("/controls/engines/engine[0]/throttle", 0);
            setprop("/controls/gears/brake-parking", 0);
        } elsif (state_name == "approach") {
            setprop("/controls/engines/engine[0]/throttle", 0.425);
            setprop("/controls/engines/engine[0]/mixture", 1);
            setprop("/controls/engines/engine[0]/propeller-pitch", 1);
        }

        state_init.active = 0;
        return;
    }

    settimer(func {
        state_init.poll_until_stable_running(state_name, remaining - 1);
    }, 0.1);
};

state_init.poll_until_running = func(state_name, remaining) {
    if (getprop("/engines/engine[0]/running", 0) == 1) {
        setprop("/controls/engines/engine[0]/starter", 0);

        settimer(func(){
            state_init.poll_until_stable_running(state_name, 20);
        }, 0.1);
        return;
    }

    if (remaining <= 0) {
        # Give up cleanly instead of leaving the starter held forever.
        setprop("/controls/engines/engine[0]/starter", 0);

        if (state_name == "take-off") {
            setprop("/controls/engines/engine[0]/throttle", 0);
            setprop("/controls/gears/brake-parking", 0);
        } elsif (state_name == "approach") {
            setprop("/controls/engines/engine[0]/throttle", 0.425);
            setprop("/controls/engines/engine[0]/mixture", 1);
            setprop("/controls/engines/engine[0]/propeller-pitch", 1);
        }
        state_init.active = 0;
        return;
    }

    settimer(func {
        state_init.poll_until_running(state_name, remaining - 1);
    }, 0.1);
};

state_init.arm_running_state = func(state_name) {
    if (state_init.active) {
        return;
    }

    state_init.active = 1;

    # If the engine is already running, just apply the final state cleanup.
    if (getprop("/engines/engine[0]/running", 0) == 1) {
        state_init.finalize_running_state(state_name);
        state_init.active = 0;
        return;
    }

    # if the aircraft state is for take off, start the engine by adding a bit of throttle and set parking brake so it doesn't drift
    if (state_name == "take-off") {
        setprop("/controls/engines/engine[0]/throttle", 0.2);
        setprop("/controls/gears/brake-parking", 1);
    } elsif (state_name == "approach") {
        setprop("/controls/engines/engine[0]/throttle", 0.3);
        setprop("/controls/engines/engine[0]/mixture", 1);
        setprop("/controls/engines/engine[0]/propeller-pitch", 1);
    }

    # Runtime part only. Static setup belongs in the overlay.
    setprop("/controls/engines/engine[0]/starter", 1);

    # Poll for up to ~8 seconds.
    state_init.poll_until_running(state_name, 80);
};

state_init.on_fdm_initialized = func {
    var aircraft_state = getprop("/sim/aircraft-state", "");

    if (state_init.requires_running(aircraft_state)) {
        state_init.arm_running_state(aircraft_state);
    }
};

# Keep this listener installed so it also works on later FDM re-inits/resets.
setlistener("/sim/signals/fdm-initialized", func {
    state_init.on_fdm_initialized();
}, 0, 0);
