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

    heading_prop: props.globals.getNode("orientation/heading-magnetic-deg"),
    selected_heading_prop: props.globals.getNode("autopilot/settings/heading-bug-deg"),

    update_timer: nil,
    initialized: 0,

    init: func {
        if (me.initialized) {
            return;
        }

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

        # Selected-heading bug.
        # Geometry is local to its own origin; update() moves the element.
        me.heading_bug = me.root
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

        me.update_timer = maketimer(1.0 / 30.0, func {
            me.update();
        });

        me.update_timer.start();

        me.initialized = 1;

        print("M20M ND: Canvas prototype initialized");
    },

    update: func {
        if (!me.initialized) {
            return;
        }

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
        # Selected-heading bug.
        #

        var selected_heading = me.selected_heading_prop.getValue();

        if (selected_heading == nil) {
            me.heading_bug.setVisible(0);
            return;
        }

        var bug_error = selected_heading - heading;

        # Normalize angular error into -180 ... +180.
        while (bug_error > 180) {
            bug_error -= 360;
        }

        while (bug_error < -180) {
            bug_error += 360;
        }

        # Prototype scale:
        # 9 pixels per degree, visible across +/-20 degrees.
        if (bug_error >= -20 and bug_error <= 20) {
            me.heading_bug.setVisible(1);

            me.heading_bug.setTranslation(
                (me.width / 2) + (bug_error * 9),
                136
            );
        } else {
            me.heading_bug.setVisible(0);
        }
    }
};

var nd_window = nil;

var showND = func {
    NDCanvas.init();

    nd_window = canvas.Window.new(
        [768, 576],
        "dialog"
    );

    nd_window.set("title", "Mooney M20M ND Prototype");
    nd_window.set("resize", 1);
    nd_window.setCanvas(NDCanvas.canvas);
    nd_window.raise();
};
