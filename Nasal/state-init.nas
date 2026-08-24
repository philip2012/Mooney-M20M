# Nasal/state-init.nas
#
# Copyright (C) 2026 Philips Nguyen
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

var state_init = {
    active: 0
};

# /sim/aircraft-state contains the launcher-selected state on initial load,
# but may become empty during an in-sim reset such as Shift-Esc.
var initial_state_property = "/sim/aircraft-state";

# Preserve the selected state across FlightGear resets.
var preserved_state_property = "/systems/mooney-m20m/state/selected";
var preserved_state_node =
    props.globals.getNode(preserved_state_property, 1);

preserved_state_node.setAttribute("preserve", 1);


state_init.requires_running = func(state_name) {
    return state_name == "take-off"
        # or state_name == "cruise"
        or state_name == "approach";
};


state_init.resolve_aircraft_state = func {
    var live_state = getprop(initial_state_property, "");

    # Initial load: remember the launcher-selected aircraft state.
    if (live_state != nil and live_state != "") {
        preserved_state_node.setValue(live_state);
        return live_state;
    }

    # FDM reinitialization: fall back to the preserved state.
    var preserved_state = preserved_state_node.getValue();

    if (preserved_state != nil and preserved_state != "") {
        return preserved_state;
    }

    return "";
};


state_init.restore_running_prerequisites = func(state_name) {
    if (!state_init.requires_running(state_name)) {
        return;
    }

    setprop("/controls/engines/engine[0]/mixture", 1);
    setprop("/controls/engines/engine[0]/propeller-pitch", 1);
    setprop("/controls/engines/engine[0]/magnetos", 3);
    setprop("/controls/engines/engine[0]/master-bat", 1);
    setprop("/controls/engines/engine[0]/master-alt", 1);
};


state_init.finalize_running_state = func(state_name) {
    setprop("/controls/engines/engine[0]/starter", 0);

    if (state_name == "take-off") {
        setprop("/controls/engines/engine[0]/throttle", 0);
        setprop("/controls/gears/brake-parking", 0);

        # Selector 1 is the take-off flap detent.
        setprop(
            "/fdm/jsbsim/systems/airframe-controls/flaps/selector",
            1
        );

    } elsif (state_name == "approach") {
        setprop("/controls/engines/engine[0]/throttle", 0.425);
        setprop("/controls/engines/engine[0]/mixture", 1);
        setprop("/controls/engines/engine[0]/propeller-pitch", 1);
    }
};


state_init.finalize_failed_start = func(state_name) {
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
};


state_init.poll_until_stable_running = func(state_name, remaining) {
    if (
        getprop("/engines/engine[0]/running", 0) == 1
        and
        getprop("/engines/engine[0]/rpm", 0) >= 700
    ) {
        state_init.finalize_running_state(state_name);
        state_init.active = 0;
        return;
    }

    if (remaining <= 0) {
        state_init.finalize_failed_start(state_name);
        return;
    }

    settimer(func {
        state_init.poll_until_stable_running(
            state_name,
            remaining - 1
        );
    }, 0.1);
};


state_init.poll_until_running = func(state_name, remaining) {
    if (getprop("/engines/engine[0]/running", 0) == 1) {
        setprop("/controls/engines/engine[0]/starter", 0);

        settimer(func {
            state_init.poll_until_stable_running(
                state_name,
                20
            );
        }, 0.1);

        return;
    }

    if (remaining <= 0) {
        state_init.finalize_failed_start(state_name);
        return;
    }

    settimer(func {
        state_init.poll_until_running(
            state_name,
            remaining - 1
        );
    }, 0.1);
};


state_init.arm_running_state = func(state_name) {
    if (state_init.active) {
        return;
    }

    state_init.active = 1;

    # State overlays are not necessarily replayed during FDM reinit.
    state_init.restore_running_prerequisites(state_name);

    # Avoid restarting an engine that is already stably running.
    if (
        getprop("/engines/engine[0]/running", 0) == 1
        and
        getprop("/engines/engine[0]/rpm", 0) >= 700
    ) {
        state_init.finalize_running_state(state_name);
        state_init.active = 0;
        return;
    }

    if (state_name == "take-off") {
        setprop("/controls/engines/engine[0]/throttle", 0.2);
        setprop("/controls/gears/brake-parking", 1);

    } elsif (state_name == "approach") {
        setprop("/controls/engines/engine[0]/throttle", 0.3);
    }

    # Allow the FlightGear property-rule bridge to propagate restored
    # controls into the JSBSim-local powerplant system before cranking.
    settimer(func {
        if (!state_init.active) {
            return;
        }

        setprop("/controls/engines/engine[0]/starter", 1);
        state_init.poll_until_running(state_name, 80);

    }, 0.1);
};


state_init.on_fdm_initialized = func {
    var aircraft_state = state_init.resolve_aircraft_state();

    if (!state_init.requires_running(aircraft_state)) {
        return;
    }

    # Let JSBSim and FlightGear-facing engine outputs settle after
    # FDM creation/reinitialization.
    settimer(func {
        state_init.arm_running_state(aircraft_state);
    }, 0.2);
};


# Apply runtime state initialization after the initial FDM load and after
# subsequent FDM resets such as Shift-Esc.
setlistener("/sim/signals/fdm-initialized", func(node) {
    if (!node.getBoolValue()) {
        return;
    }

    state_init.on_fdm_initialized();

}, 0, 1);