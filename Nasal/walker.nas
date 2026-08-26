# Copyright (C) 2026 Philips Nguyen
#
# Mooney M20M Walker integration.
# Applies an aircraft-specific initial heading after Generic Walker
# finishes switching to Walk View.

var walker_outside =
    props.globals.getNode("/sim/walker/outside", 1);


var normalize_heading = func(heading) {
    while (heading >= 360)
        heading -= 360;

    while (heading < 0)
        heading += 360;

    return heading;
};


var apply_exit_heading = func {
    if (!walker_outside.getBoolValue())
        return;

    if (getprop("/sim/current-view/name") != "Walk View")
        return;

    var heading =
        getprop("/sim/current-view/heading-offset-deg");

    if (heading == nil)
        return;

    var new_heading =
        normalize_heading(heading + 90);

    print(
        "[M20M Walker] Adjusting exit heading: ",
        heading,
        " -> ",
        new_heading
    );

    setprop(
        "/sim/current-view/heading-offset-deg",
        new_heading
    );

    setprop(
        "/sim/current-view/goal-heading-offset-deg",
        new_heading
    );

    settimer(func {
        print(
            "[M20M Walker] Final heading actual=",
            getprop("/sim/current-view/heading-offset-deg"),
            " goal=",
            getprop("/sim/current-view/goal-heading-offset-deg")
        );
    }, 0.2);
};


setlistener(walker_outside, func {
    if (!walker_outside.getBoolValue())
        return;

    print("[M20M Walker] Walker exited aircraft");

    # Generic Walker completes get_out() synchronously.
    # Wait briefly so its final Walk View heading assignment
    # cannot overwrite our aircraft-specific adjustment.
    settimer(apply_exit_heading, 0.1);
});


print("[M20M Walker] Integration loaded");