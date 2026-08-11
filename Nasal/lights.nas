var strobe_flash = aircraft.light.new(
    "/sim/model/lights/strobe-flash",
    [0.5, 0.3]
);

strobe_flash.interval = 0.1;
strobe_flash.switch(1);