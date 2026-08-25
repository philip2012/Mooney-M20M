# Nasal/nd.nas
#
# Experimental Canvas ND for the Mooney M20M Bravo.
#
# This currently exists alongside the legacy model/XML ND.
# It does not replace or modify the cockpit ND.

var NDCanvas = {
    width: 768,
    height: 576,

    canvas: nil,
    root: nil,
    heading_text: nil,
    heading_scale: nil,
    heading_labels: [],
    heading_bug: nil,
    heading_left_arrow: nil,
    heading_right_arrow: nil,

    heading_mode_group: nil,
    wx_overlay_group: nil,

    sat_text: nil,
    tas_text: nil,
    gs_text: nil,

    range_left_text: nil,
    range_right_text: nil,

    range_outer_arc: nil,
    range_inner_arc: nil,
    aircraft_symbol: nil,

    nav_source_text: nil,
    nav_distance_text: nil,

    tcas_text: nil,

    vor_overlay_text: nil,
    apt_overlay_text: nil,

    heading_prop: props.globals.getNode("orientation/heading-magnetic-deg"),
    selected_heading_prop: props.globals.getNode("autopilot/settings/heading-bug-deg"),
    heading_bug_error_prop: props.globals.getNode("autopilot/internal/heading-bug-error-deg"),

    mfd_map_prop: props.globals.getNode(
        "instrumentation/primus2000/dc840/mfd-map"
    ),

    mfd_wx_prop: props.globals.getNode(
        "instrumentation/primus2000/dc840/mfd-wx"
    ),

    sat_prop: props.globals.getNode("environment/temperature-degc"),
    tas_prop: props.globals.getNode("instrumentation/airspeed-indicator/true-speed-kt"),
    gs_prop: props.globals.getNode("velocities/groundspeed-kt"),

    range_prop: props.globals.getNode("instrumentation/efis/inputs/range-nm"),

    nav_type_prop: props.globals.getNode("autopilot/internal/nav-type"),
    nav_distance_prop: props.globals.getNode("autopilot/internal/nav-distance"),

    tcas_prop: props.globals.getNode(
        "instrumentation/primus2000/dc840/tcas"
    ),

    main_bus_volts_prop: props.globals.getNode(
        "systems/mooney-m20m/electrical/bus/main-volts"
    ),

    update_timer: nil,
    initialized: 0,

    init: func {
        if (me.initialized) {
            return;
        }

        # A previous popup may have destroyed the old Canvas.
        # Start every fresh Canvas with fresh retained-element handles.
        me.heading_labels = [];

        me.canvas = canvas.new({
            "name": "M20M-ND",
            "size": [me.width, me.height],
            "view": [me.width, me.height],
            "mipmapping": 1
        });

        me.canvas.setColorBackground(0, 0, 0, 1);

        me.root = me.canvas.createGroup();

        me.heading_text = me.root
            .createChild("text", "heading")
            .setTranslation(me.width / 2, 50)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Bold.ttf")
            .setFontSize(36, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("HDG ---");

        me.heading_text.enableUpdate();

        #
        # Heading scale
        #

        me.heading_scale = me.root.createChild(
            "group",
            "heading-scale"
        );

        me.heading_scale.set(
            "clip",
            "rect(70, 620, 145, 148)"
        );

        me.heading_scale.set(
            "clip-frame",
            canvas.Element.PARENT
        ); 

        # Horizontal reference line.
        me.root
            .createChild("path", "heading-baseline")
            .moveTo(150, 120)
            .lineTo(618, 120)
            .setColor(1, 1, 1)
            .setStrokeLineWidth(2);

        # Five major ticks: -20, -10, current, +10, +20 degrees.
        for (var i = -2; i <= 2; i += 1) {
            var x = (me.width / 2) + (i * 90);

            var tick_height = 14;

            if (i == 0) {
                tick_height = 22;
            }

            me.heading_scale
                .createChild("path")
                .moveTo(x, 120)
                .lineTo(x, 120 - tick_height)
                .setColor(1, 1, 1)
                .setStrokeLineWidth(2);

            var label = me.heading_scale
                .createChild("text")
                .setTranslation(x, 86)
                .setAlignment("center-center")
                .setFont("LiberationFonts/LiberationSans-Regular.ttf")
                .setFontSize(22, 1.0)
                .setColor(1, 1, 1, 1)
                .setText("---");

            label.enableUpdate();

            append(me.heading_labels, label);
        }

        # Fixed aircraft/reference marker.
        me.root
            .createChild("path", "heading-reference")
            .moveTo((me.width / 2) - 7, 132)
            .lineTo(me.width / 2, 120)
            .lineTo((me.width / 2) + 7, 132)
            .close()
            .setColorFill(1, 1, 1)
            .setStrokeLineWidth(0);

        #
        # MAP-mode heading symbology.
        #
        # Keep all selected-heading elements under one retained group so
        # display-mode visibility does not need to toggle each child.
        #

        me.heading_mode_group = me.root
            .createChild("group", "heading-mode");

        # Selected-heading bug.
        # Geometry is local to its own origin; update() moves the element.
        me.heading_bug = me.heading_mode_group
            .createChild("path", "selected-heading-bug")
            .moveTo(-10, 0)
            .lineTo(0, 11)
            .lineTo(10, 0)
            .close()
            .setColorFill(1, 0, 1)
            .setStrokeLineWidth(0);

        me.heading_bug.setTranslation(
            me.width / 2,
            136
        );

        #
        # Off-scale selected-heading arrows.
        #

        me.heading_left_arrow = me.heading_mode_group
            .createChild("path", "heading-left-arrow")
            .moveTo(0, 0)
            .lineTo(14, -9)
            .lineTo(14, 9)
            .close()
            .setColorFill(1, 0, 1)
            .setStrokeLineWidth(0)
            .setTranslation(145, 121)
            .setVisible(0);

        me.heading_right_arrow = me.heading_mode_group
            .createChild("path", "heading-right-arrow")
            .moveTo(0, 0)
            .lineTo(-14, -9)
            .lineTo(-14, 9)
            .close()
            .setColorFill(1, 0, 1)
            .setStrokeLineWidth(0)
            .setTranslation(623, 121)
            .setVisible(0);

        #
        # Static data readouts.
        #

        me.sat_text = me.root
            .createChild("text", "sat")
            .setTranslation(145, 520)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(24, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("SAT --- C");

        me.sat_text.enableUpdate();

        me.tas_text = me.root
            .createChild("text", "tas")
            .setTranslation(610, 490)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(24, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("TAS ---");

        me.tas_text.enableUpdate();

        me.gs_text = me.root
            .createChild("text", "gs")
            .setTranslation(610, 525)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(24, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("GS ---");

        me.gs_text.enableUpdate();

        #
        # ND range geometry.
        #
        # Geometry is retained and created once. Range selection changes
        # the represented distance, not the physical Canvas geometry.
        #

        var arc_center_x = me.width / 2;
        var arc_center_y = 475;
        var outer_radius = 235;
        var inner_radius = outer_radius / 2;

        me.range_outer_arc = me.root
            .createChild("path", "range-outer-arc");

        me.range_inner_arc = me.root
            .createChild("path", "range-inner-arc");

        #
        # Forward-looking 120-degree arcs: -60 ... +60 degrees.
        # Canvas Y increases downward, so cosine is subtracted from Y.
        #
        for (var angle = -60; angle <= 60; angle += 2) {
            var rad = angle * math.pi / 180.0;

            var outer_x = arc_center_x + math.sin(rad) * outer_radius;
            var outer_y = arc_center_y - math.cos(rad) * outer_radius;

            var inner_x = arc_center_x + math.sin(rad) * inner_radius;
            var inner_y = arc_center_y - math.cos(rad) * inner_radius;

            if (angle == -60) {
                me.range_outer_arc.moveTo(outer_x, outer_y);
                me.range_inner_arc.moveTo(inner_x, inner_y);
            } else {
                me.range_outer_arc.lineTo(outer_x, outer_y);
                me.range_inner_arc.lineTo(inner_x, inner_y);
            }
        }

        me.range_outer_arc
            .setColor(1, 1, 1)
            .setStrokeLineWidth(2);

        me.range_inner_arc
            .setColor(1, 1, 1)
            .setStrokeLineWidth(1);

        #
        # Fixed own-aircraft symbol.
        #

        me.aircraft_symbol = me.root
            .createChild("path", "aircraft-symbol")
            .moveTo(384, 423)
            .lineTo(384, 455)
            .moveTo(370, 440)
            .lineTo(398, 440)
            .moveTo(376, 455)
            .lineTo(392, 455)
            .setColor(1, 1, 1)
            .setStrokeLineWidth(3);

        #
        # ND range indications.
        #
        # The legacy ND contains independent left/right digit groups,
        # both driven by instrumentation/efis/inputs/range-nm.
        #

        me.range_left_text = me.root
            .createChild("text", "range-left")
            .setTranslation(235, 265)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(22, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("---");

        me.range_left_text.enableUpdate();

        me.range_right_text = me.root
            .createChild("text", "range-right")
            .setTranslation(533, 265)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(22, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("---");

        me.range_right_text.enableUpdate();

        #
        # NAV source / distance.
        #
        # Legacy ND supports VOR1 and VOR2 source indications.
        #

        me.nav_source_text = me.root
            .createChild("text", "nav-source")
            .setTranslation(145, 425)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(22, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("");

        me.nav_source_text.enableUpdate();

        me.nav_distance_text = me.root
            .createChild("text", "nav-distance")
            .setTranslation(145, 458)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(22, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("---");

        me.nav_distance_text.enableUpdate();

        #
        # TCAS mode annunciation.
        #
        # Mirrors the legacy MFD.tcas-off / MFD.tcas-auto states.
        #

        me.tcas_text = me.root
            .createChild("text", "tcas-mode")
            .setTranslation(610, 425)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(22, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("TCAS OFF");

        me.tcas_text.enableUpdate();

        #
        # Legacy VOR / airport overlay state.
        #
        # The original ND uses textured MFD.vor and MFD.apt objects.
        # These text labels are temporary Canvas scaffolding; the original
        # artwork/symbology can replace them during visual-parity cleanup.
        #

        me.wx_overlay_group = me.root
            .createChild("group", "wx-overlays");

        me.wx_overlay_group.setVisible(0);

        me.vor_overlay_text = me.wx_overlay_group
            .createChild("text", "vor-overlay")
            .setTranslation(275, 350)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(20, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("VOR");

        me.apt_overlay_text = me.wx_overlay_group
            .createChild("text", "apt-overlay")
            .setTranslation(493, 350)
            .setAlignment("center-center")
            .setFont("LiberationFonts/LiberationSans-Regular.ttf")
            .setFontSize(20, 1.0)
            .setColor(1, 1, 1, 1)
            .setText("APT");

        me.update_timer = maketimer(1.0 / 30.0, func {
            me.update();
        });

        me.update_timer.start();

        me.initialized = 1;

        print("M20M ND: Canvas prototype initialized");
    },

    shutdown: func {
        #
        # The development popup is currently the Canvas's only placement.
        # Closing that window destroys the GUI-side Canvas resources, so
        # invalidate our retained handles and rebuild on the next showND().
        #

        if (me.update_timer != nil) {
            me.update_timer.stop();
            me.update_timer = nil;
        }

        me.canvas = nil;
        me.root = nil;

        me.heading_text = nil;
        me.heading_scale = nil;
        me.heading_labels = [];
        me.heading_bug = nil;
        me.heading_left_arrow = nil;
        me.heading_right_arrow = nil;

        me.heading_mode_group = nil;
        me.wx_overlay_group = nil;

        me.sat_text = nil;
        me.tas_text = nil;
        me.gs_text = nil;

        me.range_left_text = nil;
        me.range_right_text = nil;

        me.range_outer_arc = nil;
        me.range_inner_arc = nil;
        me.aircraft_symbol = nil;

        me.nav_source_text = nil;
        me.nav_distance_text = nil;

        me.tcas_text = nil;

        me.vor_overlay_text = nil;
        me.apt_overlay_text = nil;

        me.initialized = 0;

        print("M20M ND: Canvas prototype shut down");
    },

    update: func {
        if (!me.initialized) {
            return;
        }

        #
        # Display power.
        #
        # Match the legacy ND's 24 V main-bus threshold.
        #

        var main_bus_volts = me.main_bus_volts_prop.getValue();

        if (main_bus_volts == nil or main_bus_volts < 24) {
            me.root.setVisible(0);
            return;
        }

        me.root.setVisible(1);

        var heading = me.heading_prop.getValue();

        if (heading == nil) {
            me.heading_text.updateText("HDG ---");
            return;
        }

        var rounded_heading = math.mod(
            int(heading + 0.5),
            360
        );

        me.heading_text.updateText(
            sprintf("HDG %03d", rounded_heading)
        );

        #
        # Heading scale labels.
        #
        # The scale geometry itself is static. Only the displayed values
        # change, avoiding repeated Canvas element creation.
        #

        # Use the nearest 10-degree heading as the center major tick.
        #
        # Example:
        #   aircraft heading = 209
        #   center heading   = 210
        #   tape offset      = -1 degree
        #
        # The scale then moves continuously underneath the fixed
        # aircraft/reference marker.

        var center_heading = int(
            (heading + 5.0) / 10.0
        ) * 10;

        var tape_offset = heading - center_heading;

        # 90 px between 10-degree ticks = 9 px/degree.
        # Snap the tape to whole Canvas pixels.
        # Moving rasterized text through fractional pixels can cause
        # visible glyph shimmer/tearing.
        var tape_x = math.floor(
            (-tape_offset * 9.0) + 0.5
        );

        me.heading_scale.setTranslation(
            tape_x,
            0
        );

        for (var i = -2; i <= 2; i += 1) {
            var tape_heading = math.mod(
                center_heading + (i * 10) + 360,
                360
            );

            me.heading_labels[i + 2].updateText(
                sprintf("%03d", tape_heading)
            );
        }

        #
        # Static data readouts.
        #

        var sat = me.sat_prop.getValue();

        if (sat == nil) {
            me.sat_text.updateText("SAT --- C");
        } else {
            me.sat_text.updateText(
                sprintf("SAT %d C", int(math.floor(sat + 0.5)))
            );
        }

        var tas = me.tas_prop.getValue();

        if (tas == nil) {
            me.tas_text.updateText("TAS ---");
        } else {
            me.tas_text.updateText(
                sprintf("TAS %03d", int(math.floor(tas + 0.5)))
            );
        }

        var gs = me.gs_prop.getValue();

        if (gs == nil) {
            me.gs_text.updateText("GS ---");
        } else {
            me.gs_text.updateText(
                sprintf("GS %03d", int(math.floor(gs + 0.5)))
            );
        }

        #
        # ND range indications.
        #

        var range_nm = me.range_prop.getValue();

        if (range_nm == nil) {
            me.range_left_text.updateText("---");
            me.range_right_text.updateText("---");
        } else {
            var range_text = sprintf(
                "%d",
                int(math.floor(range_nm + 0.5))
            );

            me.range_left_text.updateText(range_text);
            me.range_right_text.updateText(range_text);
        }

        #
        # NAV source / distance.
        #

        var nav_type = me.nav_type_prop.getValue();

        if (nav_type == "VOR1") {
            me.nav_source_text.updateText("VOR1");
        } elsif (nav_type == "VOR2") {
            me.nav_source_text.updateText("VOR2");
        } else {
            me.nav_source_text.updateText("");
        }

        var nav_distance = me.nav_distance_prop.getValue();

        if (nav_distance == nil) {
            me.nav_distance_text.updateText("---");
        } else {
            me.nav_distance_text.updateText(
                sprintf(
                    "%03d",
                    int(math.floor(nav_distance + 0.5))
                )
            );
        }

        #
        # TCAS mode annunciation.
        #

        var tcas_enabled = me.tcas_prop.getValue();

        if (tcas_enabled) {
            me.tcas_text.updateText("TCAS AUTO");
        } else {
            me.tcas_text.updateText("TCAS OFF");
        }

        #
        # Legacy VOR / airport overlay state.
        #
        # Both original objects use instrumentation/primus2000/dc840/mfd-wx.
        #

        var wx_overlay = me.mfd_wx_prop.getValue();

        me.wx_overlay_group.setVisible(
            wx_overlay ? 1 : 0
        );

        #
        # Selected-heading bug.
        #
        # Use the same normalized error property as the legacy ND.
        #

        var bug_error = me.heading_bug_error_prop.getValue();
        var map_mode = me.mfd_map_prop.getValue();

        me.heading_mode_group.setVisible(
            map_mode ? 1 : 0
        );

        if (!map_mode) {
            # Group visibility handles the entire MAP-mode branch.
        } elsif (bug_error == nil) {
            me.heading_bug.setVisible(0);
            me.heading_left_arrow.setVisible(0);
            me.heading_right_arrow.setVisible(0);
        } elsif (bug_error < -20) {
            me.heading_bug.setVisible(0);
            me.heading_left_arrow.setVisible(1);
            me.heading_right_arrow.setVisible(0);
        } elsif (bug_error > 20) {
            me.heading_bug.setVisible(0);
            me.heading_left_arrow.setVisible(0);
            me.heading_right_arrow.setVisible(1);
        } else {
            me.heading_left_arrow.setVisible(0);
            me.heading_right_arrow.setVisible(0);
            me.heading_bug.setVisible(1);

            me.heading_bug.setTranslation(
                (me.width / 2) + (bug_error * 9),
                136
            );
        }
    }
};

var nd_window = nil;

var showND = func {
    #
    # Only one development popup at a time.
    #
    if (nd_window != nil) {
        nd_window.raise();
        return;
    }

    NDCanvas.init();

    nd_window = canvas.Window.new(
        [768, 576],
        "dialog"
    );

    nd_window.set("title", "Mooney M20M ND Prototype");
    nd_window.set("resize", 1);
    nd_window.setCanvas(NDCanvas.canvas);

    #
    # Window.del() is called when the Canvas dialog is closed.
    # Clean up before invoking the normal Canvas Window destructor.
    #
    nd_window.del = func {
        NDCanvas.shutdown();
        nd_window = nil;

        call(
            canvas.Window.del,
            [],
            me
        );
    };

    nd_window.raise();
};
