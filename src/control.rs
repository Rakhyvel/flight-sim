use apricot::input::Input;

/// What the pilot is commanding
pub struct Controls {
    pub pitch: f32,    // -1 to 1
    pub roll: f32,     // -1 to 1
    pub yaw: f32,      // -1 to 1
    pub throttle: f32, // 0 to 1
    pub pan: f32,

    // Internal target states for smoothing
    target_pitch: f32,
    target_roll: f32,
    target_yaw: f32,
    target_pan: f32,
}

impl Controls {
    pub fn new() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            throttle: 0.7,
            pan: 0.0,

            target_pitch: 0.0,
            target_roll: 0.0,
            target_yaw: 0.0,
            target_pan: 0.0,
        }
    }

    pub fn update_input(&mut self, input: &Input, dt: f32) {
        self.target_roll = 0.0;
        self.target_pitch = 0.0;
        self.target_yaw = 0.0;
        self.target_pan = 0.0;

        if input.key_down(sdl2::keyboard::Scancode::LShift) {
            self.throttle += 0.01;
        }
        if input.key_down(sdl2::keyboard::Scancode::LCtrl) {
            self.throttle -= 0.01;
        }
        self.throttle = self.throttle.clamp(0.0, 1.0);

        if input.key_down(sdl2::keyboard::Scancode::W) {
            self.target_pitch -= 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::S) {
            self.target_pitch += 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::A) {
            self.target_yaw += 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::D) {
            self.target_yaw -= 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::Q) {
            self.target_roll -= 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::E) {
            self.target_roll += 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::Left) {
            self.target_pan -= 1.0;
        }
        if input.key_down(sdl2::keyboard::Scancode::Right) {
            self.target_pan += 1.0;
        }

        // Slew rate / smoothing so keyboard controls act like a spring-loaded stick (about 5.0 speed)
        let smoothing = 5.0 * dt;
        self.pitch += (self.target_pitch - self.pitch) * smoothing.min(1.0);
        self.roll += (self.target_roll - self.roll) * smoothing.min(1.0);
        self.yaw += (self.target_yaw - self.yaw) * smoothing.min(1.0);
        self.pan += (self.target_pan - self.pan) * smoothing.min(1.0);

        if let Some(js) = input.joystick() {
            let axes: Vec<String> = (0..js.num_axes())
                .map(|i| format!("{:+.2}", js.axis(i)))
                .collect();
            println!("{}: [{}]", js.name, axes.join(", "));
        }

        if let Some(js) = input.joystick() {
            let stick_roll = deadzone(js.axis(0), 0.1);
            let stick_pitch = deadzone(js.axis(1), 0.1);
            let stick_yaw = deadzone(js.axis(2), 0.1);

            // Add on the stick input
            self.pitch = stick_pitch;
            self.roll = stick_roll;
            self.yaw = -stick_yaw;
            self.throttle = (1.0 - js.axis(3)) * 0.5;
        }
        println!("{:0.2} {:0.2} {:0.2}", self.pitch, self.roll, self.yaw);
    }
}

/// Zero out small stick noise near center, then rescale so it still reaches +-1
fn deadzone(value: f32, size: f32) -> f32 {
    if value.abs() < size {
        0.0
    } else {
        value.signum() * (value.abs() - size) / (1.0 - size)
    }
}
